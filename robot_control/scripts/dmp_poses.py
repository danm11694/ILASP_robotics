"""
Copyright (C) 2019 Michele Ginesi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
"""
import numpy as np
import scipy.integrate
import scipy.interpolate
import copy
import pdb

from cs import CanonicalSystem
from exponential_integration import exp_eul_step
from exponential_integration import phi1
from rotation_matrix import roto_dilatation
import derivative_matrices as der_mtrx
import quaternion as quat

class DMPs_pose(object):
    """
    Implementation of discrete Dynamic Movement Primitives for robot poses
    """

    def __init__(self, n_bfs = 50, dt = 0.01, x_0 = np.zeros(3),
         x_goal = np.ones(3), q_0 = np.array([1.0, 0.0, 0.0, 0.0]),
         q_goal = np.array([0.0, 0.0, 0.0, 1.0]), T = 1.0, K = 1050.0, D = None,
          tol = 0.05, alpha_s = 4.0, basis = 'gaussian', e_0 = 'log', **kwargs):
        """
        n_bfs int    : number of basis functions per DMP (actually, they will be
                       one more)
        dt float     : timestep for simulation
        x0 array     : initial cartesian state of DMPs
        x_goal array : goal cartesian state of DMPs
        q_0 array    : initial quaternion state of DMPs
        q_goal array : goal quaternion state of DMPs
        T float      : final time
        K float      : elastic parameter
        D float      : damping parameter
        tol float    : tolerance
        alpha_s float: constant of the Canonical System
        rescale bool : decide if the rescale property is used
        basis string : type of basis functions
        e_0 string   : determine if compute omega as vector component or
                       quaternion log
        """
        ## Initialize the parameters
        self.tol = tol  # tolerance to stop the evolution of the trajectory
        self.n_bfs = n_bfs  # number of basis function (- 1)
        self.K = K  # elastic term for the cartesian component
        if D is None:
            D = 2 * np.sqrt(self.K)
        self.D = D  # damping term for the cartesian component
        self.x_0 = x_0  # initial cartesian state
        self.x_goal = x_goal  # goal cartesian state
        self.q_0 = q_0  # initial quaternion state
        self.q_goal = q_goal  # goal quaternion state
        self.basis = basis  # class of basis functions
        ## Set up the Canonical System
        self.cs = CanonicalSystem(dt = dt, alpha_s = alpha_s, run_time = T)
        ## Reset the DMP system
        self.reset_state()
        ## Create the parameters for the learning phase
        self.gen_center()
        self.gen_width()
        # set of weights for the Cartesian component
        self.w_cart = np.zeros([3, self.n_bfs + 1])
        # set of weights for the quaternion component
        self.w_quat = np.zeros([3, self.n_bfs + 1])
        self.e_0 = e_0
        # Compute the linear component of the Dynamical System
        self.compute_linear_component()

    def compute_linear_component(self):
        """
        Save in a class variable the linear component of the model
        (only for the Euclidean part, thus Cartesian position and angular
        velocities)
        """
        self.linear_component = np.zeros([9, 9])
        # Cartesian component
        for d in range(3):
            self.linear_component[2 * d, 2 * d] = - self.D
            self.linear_component[2 * d, 2 * d + 1] = - self.K
            self.linear_component[2 * d + 1, 2 * d] = 1.0
        self.linear_component[6:9, 6:9] = - self.D * np.eye(3)

    def compute_scaling_term(self):
        """
        Compute the scaling term D_0 and its inverse
        """
        # Compute the rotation matrix version of starting position and goal
        R_0 = quat.rot_matrix_form(self.q_0)
        R_g = quat.rot_matrix_form(self.q_goal)
        R = np.matmul(R_g, np.transpose(R_0))
        # We now compute the log of R:
        #            / [0, 0, 0]^T  , R = I
        # log (R) = <|
        #            \ \theta n     , otherwise
        #
        #                 / tr (R) - 1  \
        # \theta = arccos | ------------ |
        #                 \      2      /
        #
        #            1          / r_{32} - r_{23} \
        # n = ----------------  | r_{13} - r_{31} |
        #     2 \sin (\theta)   \ r_{21} - r_{12} /
        if ((R == np.identity(3)).all()):
            # FIXME Should be instead returned the identity matrix?
            dd = np.ones(3) #np.zeros(3)
        else:
            tr = R[0,0] + R[1,1] + R[2,2] # tr (R)
            theta = np.arccos(np.clip((tr - 1.) / 2., -1., 1.))
            n = np.ones(3) / 2. / np.sin(theta)
            n[0] *= R[2,1] - R[1,2]
            n[1] *= R[0,2] - R[2,0]
            n[2] *= R[1,0] - R[0,1]
            dd = theta * n
        return np.diag(dd), np.diag(1.0 / dd)

    def gen_center(self):
        """
        Generate the centers of the basis functions
        """
        # Desired activations throughout time
        self.c = np.exp(- self.cs.alpha_s * self.cs.run_time *
                   ((np.cumsum(np.ones([1, self.n_bfs + 1])) - 1) / self.n_bfs))

    def gen_psi(self, s):
        """
        Generates the activity of the basis functions for a given canonical
        system rollout.
        """
        c = np.reshape(self.c, [self.n_bfs + 1, 1])
        w = np.reshape(self.width, [self.n_bfs + 1,1 ])
        if (self.basis == 'gaussian'):
            xi = w * (s - c) * (s - c)
            psi_set = np.exp(- xi)
        else:
            xi = np.abs(w * (s - c))
            if (self.basis == 'mollifier'):
                psi_set = (np.exp(- 1.0 / (1.0 - xi * xi))) * (xi < 1.0)
            elif (self.basis == 'wendland2'):
                psi_set = ((1.0 - xi) ** 2.0) * (xi < 1.0)
            elif (self.basis == 'wendland3'):
                psi_set = ((1.0 - xi) ** 3.0) * (xi < 1.0)
            elif (self.basis == 'wendland4'):
                psi_set = ((1.0 - xi) ** 4.0 * (4.0 * xi + 1.0)) * (xi < 1.0)
            elif (self.basis == 'wendland5'):
                psi_set = ((1.0 - xi) ** 5.0 * (5.0 * xi + 1)) * (xi < 1.0)
            elif (self.basis == 'wendland6'):
                psi_set = ((1.0 - xi) ** 6.0 * 
                              (35.0 * xi ** 2.0 + 18.0 * xi + 3.0)) * (xi < 1.0)
            elif (self.basis == 'wendland7'):
                psi_set = ((1.0 - xi) ** 7.0 *
                               (16.0 * xi ** 2.0 + 7.0 * xi + 1.0)) * (xi < 1.0)
            elif (self.basis == 'wendland8'):
                psi_set = (((1.0 - xi) ** 8.0 *
                         (32.0 * xi ** 3.0 + 25.0 * xi ** 2.0 + 8.0 * xi + 1.0))
                                                                   * (xi < 1.0))
        psi_set = np.nan_to_num(psi_set)
        return psi_set

    def gen_weights(self, f_target_cartesian, f_target_quaternion):
        """
        Generates the set of weights that best realize the desired forcing terms
        """
        # Generate the basis functions
        s_track = self.cs.rollout()
        psi_track = self.gen_psi(s_track)
        # Compute useful quantities
        sum_psi = np.sum(psi_track, 0)
        sum_psi_2 = sum_psi * sum_psi
        s_track_2 = s_track * s_track
        # Set up the minimization problems
        A = np.zeros([self.n_bfs + 1, self.n_bfs + 1])
        b_cartesian = np.zeros([self.n_bfs + 1])
        b_quaternion = np.zeros([self.n_bfs + 1])
        for k in range(self.n_bfs + 1):
            A[k, k] = scipy.integrate.simps(
                   psi_track[k] * psi_track[k] * s_track_2 / sum_psi_2, s_track)
            for h in range(k + 1, self.n_bfs + 1):
                A[k, h] = scipy.integrate.simps(
                            psi_track[k] * psi_track[h] * s_track_2 / sum_psi_2,
                                                                        s_track)
                A[h, k] = A[k, h].copy()
        LU = scipy.linalg.lu_factor(A)
        # The problem is decoupled for each dimension
        for d in range(3):
            # Create the vector of the regression problem
            for k in range(self.n_bfs + 1):
                b_cartesian[k] = scipy.integrate.simps(
                       f_target_cartesian[d] * psi_track[k] * s_track / sum_psi,
                                                                        s_track)
                b_quaternion[k] = scipy.integrate.simps(
                   f_target_quaternion[:, d] * psi_track[k] * s_track / sum_psi,
                                                                        s_track)
            # Solve the minimization problem
            self.w_cart[d] = scipy.linalg.lu_solve(LU, b_cartesian)
            self.w_quat[d] = scipy.linalg.lu_solve(LU, b_quaternion, check_finite = False)
        self.w_cart = np.nan_to_num(self.w_cart)
        self.w_quat = np.nan_to_num(self.w_quat)

    def gen_width(self):
        """
        Set the "widths" for the basis functions.
        """
        if (self.basis == 'gaussian'):
            self.width = 1.0 / np.diff(self.c) / np.diff(self.c)
            self.width = np.append(self.width, self.width[-1])
        else:
            self.width = 1.0 / np.diff(self.c)
            self.width = np.append(self.width[0], self.width)

    def imitate_path(self, pose_des, t_des = None, **kwargs):
        """
        Take a desired pose in time and generates the set of parameters that
        best realize the pose:
         pose_des array shaped n_timeteps x 7
         t_des 1D array of num_timesteps component
        """

        ## Set initial and goal pose
        self.x_0 = pose_des[0][0:3]
        self.x_goal = pose_des[-1][0:3]
        self.q_0 = pose_des[0][3:7]
        self.q_goal = pose_des[-1][3:7]

        ## Set t_span
        if t_des is None:
            # Default value for t_des
            t_des = np.linspace(0.0, self.cs.run_time, pose_des.shape[0])
        else:
            # Warp time to start from zero and end up to T
            t_des -= t_des[0]
            t_des /= t_des[-1]
            t_des *= self.cs.run_time
        time = np.linspace(0., self.cs.run_time, self.cs.timesteps)

        ## Piecewise linear interpolation
        path_gen = scipy.interpolate.interp1d(t_des, pose_des.transpose())
        path = path_gen(time)
        pose_des = path.transpose()

        ## Canonical system evolution
        s_track = self.cs.rollout()

        ## Decouple learning of Cartesian and Quaternion weights
        x_des = pose_des[:, 0:3]
        # Compute first derivative
        D1 = der_mtrx.compute_D1(self.cs.timesteps, self.cs.dt)
        dx_des = np.dot(D1, x_des)
        # Compute second derivative
        D2 = der_mtrx.compute_D2(self.cs.timesteps, self.cs.dt)
        ddx_des = np.dot(D2, x_des)
        f_target_cartesian = ((ddx_des / self.K - (self.x_goal - x_des) +
                         self.D / self.K * dx_des).transpose() +
                         np.reshape((self.x_goal - self.x_0), [3, 1]) * s_track)

        q_des = pose_des[:, 3:7]
        # Normalize q_des
        q_des = quat.normalize(q_des)
        # Estimation of the angular velocity
        eta_des = quat.estimate_omega(q_des, self.cs.dt)
        D1 = der_mtrx.compute_D1(q_des.shape[0], self.cs.dt)
        deta_des = np.dot(D1, eta_des)

        ## Compute the desired forcing term
        s_track = np.reshape(np.exp(- self.cs.alpha_s * time),
                                                            [q_des.shape[0], 1])
        goal_rep = np.repeat([self.q_goal], q_des.shape[0], axis = 0)
        if (self.e_0 is "log"):
            e0_quant = 2.0 * quat.log(
                          quat.product(goal_rep, quat.conjugate(q_des))) \
                          - 2.0 * quat.log(
                          quat.product(self.q_goal, quat.conjugate(self.q_0))) \
                                                                       * s_track
        else:
            e0_quant = (quat.vect(quat.product(goal_rep, quat.conjugate(q_des)))
                - quat.vect(quat.product(self.q_goal, quat.conjugate(self.q_0)))
                                                                      * s_track)
        dyn_sim = deta_des / self.K - e0_quant + self.D / self.K * eta_des
        _, D_0_inv = self.compute_scaling_term()
        f_target_quaternion = np.dot(dyn_sim, D_0_inv.transpose())

        ## Compute the sets of weights
        self.gen_weights(f_target_cartesian, f_target_quaternion)

    def reset_state(self):
        """
        Reset the system state
        """
        # Cartesian reset
        self.x = self.x_0.copy()
        self.dx = np.zeros(3)
        self.ddx = np.zeros(3)
        # Quaternion reset
        self.q = copy.deepcopy(self.q_0)
        self.dq = np.zeros(4)
        self.eta = np.zeros(3)
        self.deta = np.zeros(3)
        # Canonical System reset
        self.cs.reset_state()

    def rollout(self, tau = 1.0):
        """
        Create a system trial
        """
        self.reset_state()
        ## Set up tracking arrays
        # Cartesian
        x_track = np.zeros([1,3])
        x_track[0] = self.x_0
        dx_track = np.zeros([1,3])
        ddx_track = np.zeros([1,3])
        # Quaternion
        q_track = np.zeros([1,4])
        q_track[0] = self.q_0
        eta_track = np.zeros([1,3])
        dq_track = np.zeros([1,4])
        deta_track = np.zeros([1,3])
        # time
        t_track = np.zeros(1)
        flag_stop = False
        while (not flag_stop):
            self.step(tau = tau)
            x_track = np.append(x_track, [self.x], axis = 0)
            dx_track = np.append(dx_track, [self.dx], axis = 0)
            ddx_track = np.append(ddx_track, [self.ddx], axis = 0)
            q_track = np.append(q_track, [self.q], axis = 0)
            dq_track = np.append(q_track, [self.dq], axis = 0)
            eta_track = np.append(eta_track, [self.eta], axis = 0)
            deta_track = np.append(deta_track, [self.deta], axis = 0)
            t_track = np.append(t_track, t_track[-1] + self.cs.dt)
            stop_q = (quat.distance(self.q, self.q_goal) /
                               quat.distance(self.q_0, self.q_goal) <= self.tol)
            stop_c = (np.linalg.norm(self.x - self.x_goal) /
                             np.linalg.norm(self.x_0 - self.x_goal) <= self.tol)
            stop_cs = (self.cs.s <
                        np.exp(- self.cs.alpha_s * self.cs.run_time) + self.tol)
            flag_stop = stop_q and stop_c and stop_cs
        return [x_track, dx_track, ddx_track,
                              q_track, dq_track, eta_track, deta_track, t_track]

    def step(self, tau = 1.0, error = 0.0, external_force = np.zeros(3),
                                                                      **kwargs):
        """
        Perform a unique step of the integration.
        """
        error_coupling = 1.0 / (1.0 + error)
        ## Generate basis functions
        s = copy.deepcopy(self.cs.s)
        psi = self.gen_psi(s)

        ## Compute the forcing terms
        # Cartesian
        f_cart = (np.dot(self.w_cart, psi[:, 0])) / (np.sum(psi[:, 0])) * s
        f_cart = np.nan_to_num(f_cart)

        # Quaternion
        D_0, _ = self.compute_scaling_term()
        D_0 = copy.deepcopy(np.nan_to_num(D_0))
        if ((np.sum(psi) == 0).all()):
            f_quat = np.zeros(3)
        else:
            f_quat = np.dot(D_0,
                           np.dot(self.w_quat, psi) * self.cs.s / (np.sum(psi)))
            f_quat = f_quat[:, 0]

        ### Integration of Cartesian component and Angular velocity
        ## Prepare the integration scheme
        state = np.zeros(9)
        state[range(0, 6, 2)] = self.dx
        state[range(1, 7, 2)] = self.x
        state[6:9] = self.eta
        # Linear component
        A_m = self.linear_component / tau
        P_m = phi1(self.cs.dt * A_m)
        # Affine component
        b_tilde = np.zeros(9)
        b_tilde[range(0, 6, 2)] = self.K * \
              (self.x_goal * (1.0 - s) + self.x_0 * s + f_cart) + external_force
        if (self.e_0 is "log"):
            b_tilde[6:9] = self.K * (2.0 *
                     quat.log(quat.product(self.q_goal, quat.conjugate(self.q)))
                     - 2.0 * self.cs.s *
                     quat.log(
                         quat.product(self.q_goal, quat.conjugate(self.q_0))) +
                                                                   f_quat) / tau
        else:
            b_tilde[6:9] = self.K * \
                (quat.vect(quat.product(self.q_goal, quat.conjugate(self.q)))
                         - self.cs.s * quat.vect(quat.product(
                         self.q_goal, quat.conjugate(self.q_0))) + f_quat) / tau
        b_tilde /= tau
        # Vector field
        vect_field = np.dot(A_m, state) + b_tilde
        # Integration Step
        state += self.cs.dt * np.dot(P_m, vect_field)
        # Extract the info's
        self.dx = state[range(0, 6, 2)]
        self.x = state[range(1, 7, 2)]
        self.ddx = self.K / tau / tau * \
              ((self.x_goal - self.x) - (self.x_goal - self.x_0) * s + f_cart) \
                                                        - self.D / tau * self.dx
        self.eta = state[6:9]

        ### Integration of quaternion position
        # Numerical integration of q
        eta_quat = np.zeros(4)
        eta_quat[1:4] = copy.deepcopy(self.eta)
        # Compute the new value of q
        zeta = quat.exp(- self.cs.dt / 2.0 * eta_quat / tau)
        self.q = quat.product(quat.inverse(zeta), self.q)
        self.q = quat.normalize(self.q)

        ## Run the canonical system
        self.cs.step(tau = tau, error_coupling = error_coupling)

        if (self.e_0 is "log"):
            self.deta = (self.K * 2.0 * quat.log(quat.product(
                   self.q_goal, quat.conjugate(self.q))) -
                   self.cs.s * self.K * 2.0 *
                   quat.log(quat.product(self.q_goal, quat.conjugate(self.q_0)))
                                    - self.D * self.eta + self.K * f_quat) / tau
        else:
            self.deta = (self.K * quat.vect(quat.product(
                  self.q_goal, quat.conjugate(self.q))) -
                  self.cs.s * self.K *
                  quat.vect(quat.product(self.q_goal, quat.conjugate(self.q_0)))
                                    - self.D * self.eta + self.K * f_quat) / tau
        self.dq = quat.product(eta_quat, self.q)
        self.dq /= (2.0 * tau)

        return [self.x, self.dx, self.ddx, self.q, self.eta, self.dq, self.deta]

    if __name__ == "__main__":
        import matplotlib.pyplot as plt
        import dmp_poses
        DMP_1 = dmp_poses.DMPs_pose(n_bfs = 50)
        # Create the desired trajectories
        t_steps = 1000
        t = np.linspace(0.0, 1.0, t_steps)
        q_des = np.zeros([t_steps, 4])
        a1 = 1.0 / 3.0
        a2 = 1.0 / 2.0
        a3 = 1.0 / 6.0
        q_des[:, 0] = a1 * np.cos(t * np.pi)
        q_des[:, 1] = a2 * np.cos(t * np.pi)
        q_des[:, 2] = a3 * np.cos(t * np.pi)
        q_des[:, 3] = np.sqrt(1.0 - (q_des[:, 0] ** 2.0 + q_des[:,1] ** 2.0 + q_des[:,2] ** 2.0))
        q_des = quat.normalize(q_des)
        x_des = np.zeros([t_steps, 3])
        x_des[:, 0] = np.cos(np.pi * t)
        x_des[:, 1] = np.sin(np.pi * t)
        x_des[:, 2] = t
        pose_des = np.concatenate((x_des, q_des), axis = 1)
        DMP_1.imitate_path(pose_des)
        [x_track, dx_track, ddx_track, q_track, dq_track, eta_track, deta_track, t_track] = DMP_1.rollout()
        plt.figure()
        plt.plot(t, q_des[:, 0], '--k')
        plt.plot(t, q_des[:, 1], '--b')
        plt.plot(t, q_des[:, 2], '--g')
        plt.plot(t, q_des[:, 3], '--r')
        plt.plot(t_track, q_track[:, 0], '-k')
        plt.plot(t_track, q_track[:, 1], '-b')
        plt.plot(t_track, q_track[:, 2], '-g')
        plt.plot(t_track, q_track[:, 3], '-r')
        plt.figure()
        plt.plot(t, x_des[:, 0], '--k')
        plt.plot(t, x_des[:, 1], '--b')
        plt.plot(t, x_des[:, 2], '--g')
        plt.plot(t_track, x_track[:, 1], '-b')
        plt.plot(t_track, x_track[:, 0], '-k')
        plt.plot(t_track, x_track[:, 2], '-g')
        plt.show()
