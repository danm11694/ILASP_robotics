"""
Copyright (C) 2018 Michele Ginesi

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.
"""

import numpy as np
import scipy.interpolate
import scipy.integrate
import copy
import pdb

from cs import CanonicalSystem
import quaternion as quat
import derivative_matrices as der_mtrx
import exponential_integration as expint

class DMPs_quaternion(object):
    """
    An implementation of DMPs for quaternion from

    [1] Saveriano M., Franzel F., Lee D. (2019).
        Merging Position and Orientation Motion Primitives
        2019 International Conference on Robotics and Automation (ICRA).
    """

    def __init__(self, n_bfs = 50, q_0 = None, q_goal = None, w = None,
          dt = 0.01, K_z = 1050, D_z = None, tol = 0.01, alpha_s = 4.0, T = 1.0,
                                     basis = 'gaussian', e_0 = 'log', **kwargs):
        """
        n_bfs int        : number of basis functions
        q_0 array        : starting quaternion position
        q_goal array     : goal quaternion position
        w float          : set of weights
        dt               : time step
        K_z float        : elastic constant
        D_z float        : damping constant
        tol float        : tolerance to stop the execution
        alpha_s float    : constant of the canonical system
        T float          : final time
        basis string     : type of basis functions
        e_0 string       : determine if compute omega as vector component or
                           quaternion log
        """
        self.Kz = K_z
        # To guarantee damping properties, we require alpha_z = 4 beta_z
        if D_z is None:
            D_z = 2.0 * np.sqrt(K_z)
        self.Dz = D_z
        self.n_bfs = n_bfs
        self.tol = tol
        if q_0 is None:
            q_0 = np.array([1.0, 0.0, 0.0, 0.0])
        self.q_0 = quat.normalize(q_0)
        if q_goal is None:
            q_goal = np.ones(4) / 2.0
        else:
            self.q_goal = copy.deepcopy(q_goal)
        self.q_goal = quat.normalize(q_goal)
        # Initialize q, dq, eta, deta
        self.q = np.zeros(4)
        self.dq = np.zeros(4)
        self.eta = np.zeros(3)
        self.deta = np.zeros(3)
        # Set up the Canonical System
        self.cs = CanonicalSystem(dt = dt, alpha_s = alpha_s, run_time = T)
        self.basis = basis
        # Reset the DMP system
        self.reset_state()
        self.gen_centers()
        self.gen_width()
        if w is None:
            w = np.zeros([3, self.n_bfs + 1])
        self.w = w
        self.e_0 = e_0

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
            dd = np.zeros(3)
        else:
            tr = R[0,0] + R[1,1] + R[2,2] # tr (R)
            theta = np.arccos((tr - 1.) / 2.)
            n = np.ones(3) / 2. / np.sin(theta)
            n[0] *= R[2,1] - R[1,2]
            n[1] *= R[0,2] - R[2,0]
            n[2] *= R[1,0] - R[0,1]
            dd = theta * n
        return np.diag(dd), np.diag(1.0 / dd)

    def gen_centers(self):
        """
        Set the centres of the basis functions to be spaced evenly throughout
        run time
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
                psi_set = ((1.0 - xi) ** 8.0 *
                     (32.0 * xi ** 3.0 + 25.0 * xi ** 2.0 + 8.0 * xi + 1.0)) * \
                                                                      (xi < 1.0)
        psi_set = np.nan_to_num(psi_set)
        return psi_set

    def gen_weights(self, f_target):
        """
        Generate a set of weights over the basis functions such that the target
        forcing term trajectory is matched.
          f_target should be shaped num_timesteps x 3
        """
        # Generate the basis functions
        s_track = self.cs.rollout()
        psi_track = self.gen_psi(s_track)
        # Compute useful quantities
        sum_psi = np.sum(psi_track, 0)
        sum_psi_2 = sum_psi * sum_psi
        s_track_2 = s_track * s_track
        # Set up the minimization problem
        A = np.zeros([self.n_bfs + 1, self.n_bfs + 1])
        b = np.zeros([self.n_bfs + 1])
        # The matrix does not depend on f
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
        self.w = np.zeros([3, self.n_bfs + 1])
        for d in range(3):
            # Create the vector of the regression problem
            for k in range(self.n_bfs + 1):
                b[k] = scipy.integrate.simps(
                     f_target[:, d] * psi_track[k] * s_track / sum_psi, s_track)
            # Solve the minimization problem
            self.w[d] = scipy.linalg.lu_solve(LU, b)
        self.w = np.nan_to_num(self.w)

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

    def imitate_path(self, q_des, t_des = None):
        """
        Takes in a desired trajectory and generates the set of weights that best
        realize this path.
          q_des should be shaped num_timesteps x 4
          t_des should be a num_timesteps array
        """

        ## Set initial state and goal
        self.q_0 = copy.deepcopy(q_des[0])
        self.q_goal = copy.deepcopy(q_des[-1])

        ## Set t_span
        if t_des is None:
            # Default value for t_des
            t_des = np.linspace(0.0, self.cs.run_time, q_des.shape[0])
        else:
            # Warp time to start from zero and end up to T
            t_des -= t_des[0]
            t_des *= self.cs.run_time / t_des[-1]
        time = np.linspace(0., self.cs.run_time, self.cs.timesteps)

        ## Computing the forcing term

        ## Interpolation of the desired trajectory
        # Interpolation of each component
        path_gen = scipy.interpolate.interp1d(t_des, q_des.transpose())
        path = path_gen(time)
        # Normalization of interpolated path
        q_des = np.transpose(path)
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
            e0_quant = 2.0 * quat.log(quat.product(
                 goal_rep, quat.conjugate(q_des))) - 2.0 * \
                 quat.log(quat.product(self.q_goal, quat.conjugate(self.q_0))) \
                                                                       * s_track
        else:
            e0_quant = (quat.vect(quat.product(goal_rep, quat.conjugate(q_des))) 
                - quat.vect(quat.product(self.q_goal, quat.conjugate(self.q_0)))
                                                                      * s_track)
        dyn_sim = deta_des / self.Kz - e0_quant + self.Dz / self.Kz * eta_des
        _, D_0_inv = self.compute_scaling_term()
        forcing_term = np.dot(dyn_sim, D_0_inv.transpose())

        ## Compute the weights associated to the desired trajectory
        self.gen_weights(forcing_term)

    def reset_state(self):
        """
        Reset the system state
        """
        self.q = copy.deepcopy(self.q_0)
        # Initialize first and second derivative of quaternions to zero
        self.dq = np.zeros(4)
        self.deta = np.zeros(3)
        self.cs.reset_state()

    def rollout(self, tau = 1., **kwargs):
        """
        Generate a system trial
        """
        self.reset_state()
        # Set up tracking vectors and quaternions
        q_track = np.zeros([1, 4])
        dq_track = np.zeros([1, 4])
        eta_track = np.zeros([1, 3])
        deta_track = np.zeros([1, 3])
        q_track[0] = self.q_0
        n = 0
        flag_stop = False
        t_track = np.zeros(1)
        while (not flag_stop):
            q_track_s, eta_track_s, dq_track_s, deta_track_s = self.step(
                      q_track[n], eta_track[n], dq_track[n], deta_track[n], tau)
            q_track = np.append(q_track, [q_track_s], axis = 0)
            eta_track = np.append(eta_track, [eta_track_s], axis = 0)
            dq_track = np.append(dq_track, [dq_track_s], axis = 0)
            deta_track = np.append(deta_track, [deta_track_s], axis = 0)
            t_track = np.append(t_track, t_track[-1] + self.cs.dt)
            n += 1
            err_q = (quat.distance(q_track_s, self.q_goal) /
                              quat.distance(self.q_0, self.q_goal))
            flag_cs = (self.cs.s <
                        np.exp(- self.cs.alpha_s * self.cs.run_time) + self.tol)
            print(err_q)
            flag_stop = ((err_q < self.tol) and flag_cs)
        # I store the final time of the rollout
        return q_track, eta_track, dq_track, deta_track, t_track

    def step(self, q_old, eta_old, dq_old, deta_old, tau, error_coupling = 1.):
        """
        Makes a single step of integration.
          q_old    : 4D array
          eta_old  : 3D array
          dq_old   : 4D array
          deta_old : 3D array
        """

        psi = self.gen_psi(self.cs.s)
        ## At first, implement a Exponential Integrator to compute eta +
        # /!\ The sum of the psi can be zero. In such case the forcing term is
        # fixed to zero
        D_0, _ = self.compute_scaling_term()
        if ((np.sum(psi) == 0).all()):
            f = np.zeros(3)
        else:
            f = np.dot(D_0, np.dot(self.w, psi) * self.cs.s / (np.sum(psi)))
            f = f[:, 0]
        A_m = - self.Dz / tau * np.eye(3)
        P = expint.phi1(self.cs.dt * A_m) # FIXME being the multiple of an id mtrx, it should be computed faster
        if (self.e_0 is "log"):
            b_vect = self.Kz * (2.0 * quat.log(quat.product(
                   self.q_goal, quat.conjugate(q_old))) - 2.0 * self.cs.s *
                   quat.log(quat.product(self.q_goal, quat.conjugate(self.q_0)))
                                                                      + f) / tau
        else:
            b_vect = self.Kz * (quat.vect(quat.product(
                  self.q_goal, quat.conjugate(q_old))) - self.cs.s *
                  quat.vect(quat.product(self.q_goal, quat.conjugate(self.q_0)))
                                                                      + f) / tau
        eta_new = eta_old + self.cs.dt * np.dot(P,
                                                  np.dot(A_m, eta_old) + b_vect)

        ## Secondly, q + is computed using Backward Euler
        # Make a quaternion version of eta
        eta_quat = np.zeros(4)
        eta_quat[1:4] = copy.deepcopy(eta_new)
        # Compute the new value of q
        zeta = quat.exp(- self.cs.dt / 2.0 * eta_quat / tau)
        q_new = quat.product(quat.inverse(zeta), q_old)
        q_new = quat.normalize(q_new)

        ## Run the canonical system
        self.cs.step(tau = tau, error_coupling = error_coupling)

        if (self.e_0 is "log"):
            deta_new = (self.Kz * 2.0 * quat.log(quat.product(
                self.q_goal, quat.conjugate(q_new))) - self.cs.s * self.Kz * 2.0
                 * quat.log(quat.product(self.q_goal, quat.conjugate(self.q_0)))
                                        - self.Dz * eta_new + self.Kz * f) / tau
        else:
            deta_new = (self.Kz * quat.vect(quat.product(
                    self.q_goal, quat.conjugate(q_new))) - self.cs.s * self.Kz *
                    quat.vect(quat.product(
                        self.q_goal, quat.conjugate(self.q_0))) -
                                          self.Dz * eta_new + self.Kz * f) / tau
        dq_new = quat.product(eta_quat, q_new)
        dq_new /= (2.0 * tau)
        return [q_new, eta_new, dq_new, deta_new]

    if __name__ == "__main__":
        import matplotlib.pyplot as plt
        # To use the package in the main folder
        import dmp_quaternion
        # Set the parameters as in Ude et al. 2014
        alpha_s = 4.0
        alpha_z = 48.0
        beta_z = 12.0
        tau = 1.0
        n_bfs = 50
        """
        Without forcing term
        """
        # These first two are form the paper
        q_0 = np.array([0.3717, - 0.4993, - 0.6162, 0.4825])
        q_0 = quat.normalize(q_0)
        q_goal = np.array([0.2471, 0.1797, 0.3182, - 0.8974])
        q_goal = quat.normalize(q_goal)
        w = np.zeros([3, n_bfs])
        DMP = dmp_quaternion.DMPs_quaternion(n_bfs, q_0, q_goal, dt = 0.01, alpha_s = alpha_s, alpha_z = alpha_z, beta_z = beta_z)
        q_track, eta_track, dq_track, deta_track, t_track = DMP.rollout(tau = tau)
        fig = plt.figure()
        plt.plot(t_track, q_track[:, 1], 'r', label = 'u_1')
        plt.plot(t_track, q_track[:, 2], 'b', label = 'u_2')
        plt.plot(t_track, q_track[:, 3], 'g', label = 'u_3')
        plt.plot(t_track, q_track[:, 0], 'k', label = 'v')
        plt.legend(loc = 'best')
        omega_track = eta_track / tau
        fig2 = plt.figure()
        plt.plot(t_track, omega_track[:, 0], 'r', label = 'omega_1')
        plt.plot(t_track, omega_track[:, 1], 'b', label = 'omega_2')
        plt.plot(t_track, omega_track[:, 2], 'g', label = 'omega_3')
        """
        Without forcing term, random q0 and goal
        """
        q_0 = np.random.rand(4)
        q_0 = quat.normalize(q_0)
        q_goal = np.random.rand(4)
        q_goal = quat.normalize(q_goal)
        w = np.zeros([3, n_bfs])
        DMP = dmp_quaternion.DMPs_quaternion(n_bfs, q_0, q_goal, dt = 0.01, alpha_s = alpha_s, alpha_z = alpha_z, beta_z = beta_z)
        q_track, eta_track, dq_track, deta_track, t_track = DMP.rollout(tau = tau)
        fig = plt.figure()
        plt.plot(t_track, q_track[:, 1], 'r', label = 'u_1')
        plt.plot(t_track, q_track[:, 2], 'b', label = 'u_2')
        plt.plot(t_track, q_track[:, 3], 'g', label = 'u_3')
        plt.plot(t_track, q_track[:, 0], 'k', label = 'v')
        plt.legend(loc = 'best')
        omega_track = eta_track / tau
        fig2 = plt.figure()
        plt.plot(t_track, omega_track[:, 0], 'r', label = 'omega_1')
        plt.plot(t_track, omega_track[:, 1], 'b', label = 'omega_2')
        plt.plot(t_track, omega_track[:, 2], 'g', label = 'omega_3')
        plt.legend(loc = 'best')
        """
        With forcing term
        """
        # The goal trajectories are cosine-like
        a1 = 1.0 / 3.0
        a2 = 1.0 / 2.0
        a3 = 1.0 / 6.0
        # Create the trajectories
        tf = np.pi
        t = np.linspace(0.0, 1.0, 10 ** 2)
        u_1 = a1 * np.cos(t * np.pi)
        u_2 = a2 * np.cos(t * np.pi)
        u_3 = a3 * np.cos(t * np.pi)
        v = np.sqrt(1.0 - (u_1 ** 2.0 + u_2 ** 2.0 + u_3 ** 2.0))
        DMP_2 = dmp_quaternion.DMPs_quaternion(n_bfs = n_bfs, dt = 0.01, alpha_s = alpha_s)
        q_des = np.array([v, u_1, u_2, u_3])
        DMP_2.imitate_path(q_des = np.transpose(q_des), t_des = t)
        q_class, eta_class, dq_class, deta_class, t_class = DMP_2.rollout()
        u_1_class = q_class[:, 1]
        u_2_class = q_class[:, 2]
        u_3_class = q_class[:, 3]
        v_class = q_class[:, 0]
        fig3 = plt.figure()
        plt.plot(t, u_1, 'r', label = 'u_1')
        plt.plot(t, u_2, 'b', label = 'u_2')
        plt.plot(t, u_3, 'g', label = 'u_3')
        plt.plot(t, v, 'm', label = 'v')
        plt.plot(t_class, u_1_class, '--r', lw = 2, alpha = 0.5, label = 'u_1_im')
        plt.plot(t_class, u_2_class, '--b', lw = 2, alpha = 0.5, label = 'u_2_im')
        plt.plot(t_class, u_3_class, '--g', lw = 2, alpha = 0.5, label = 'u_3_im')
        plt.plot(t_class, v_class, '--m', lw = 2, alpha = 0.5, label = 'v_im')
        plt.legend(loc = 'best')
        # Test with a new goal position
        q_goal_new = q_goal + 2.0
        q_goal_new = quat.normalize(q_goal_new)
        DMP_2.q_goal = q_goal_new
        q_class_new, eta_class_new, dq_class_new, deta_class_new, t_class_new = DMP_2.rollout()
        u_1_class = q_class_new[:, 1]
        u_2_class = q_class_new[:, 2]
        u_3_class = q_class_new[:, 3]
        v_class = q_class_new[:, 0]
        fig3 = plt.figure()
        plt.plot(t, u_1, 'r', label = 'u_1')
        plt.plot(t, u_2, 'b', label = 'u_2')
        plt.plot(t, u_3, 'g', label = 'u_3')
        plt.plot(t, v, 'm', label = 'v')
        plt.plot(t_class_new, u_1_class, '--r', lw = 2, alpha = 0.5, label = 'u_1_im')
        plt.plot(t_class_new, u_2_class, '--b', lw = 2, alpha = 0.5, label = 'u_2_im')
        plt.plot(t_class_new, u_3_class, '--g', lw = 2, alpha = 0.5, label = 'u_3_im')
        plt.plot(t_class_new, v_class, '--m', lw = 2, alpha = 0.5, label = 'v_im')
        plt.plot(t_class_new[-1], DMP_2.q_goal[1], 'xr', label = 'u_1_new')
        plt.plot(t_class_new[-1], DMP_2.q_goal[2], 'xb', label = 'u_2_new')
        plt.plot(t_class_new[-1], DMP_2.q_goal[3], 'xg', label = 'u_3_new')
        plt.plot(t_class_new[-1], DMP_2.q_goal[0], 'xm', label = 'v_new')
        #plt.legend(loc = 'best')
        plt.show()