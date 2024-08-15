"""
Copyright (C) 2019 Michele Ginesi
Copyright (C) 2018 Daniele Meli
Copyright (C) 2013 Travis DeWolf

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
import scipy.interpolate
import scipy.integrate
import pdb

from cs import CanonicalSystem
from exponential_integration import exp_eul_step
from exponential_integration import phi1
from rotation_matrix import roto_dilatation
import derivative_matrices as der_mtrx

class DMPs_cartesian(object):
    """
    Implementation of discrete Dynamic Movement Primitives in cartesian space,as
    described in
    [1] Park, D. H., Hoffmann, H., Pastor, P., & Schaal, S. (2008, December).
        Movement reproduction and obstacle avoidance with Dynamic movement
        primitives and potential fields.
        In Humanoid Robots, 2008. Humanoids 2008. 8th IEEE-RAS International
        Conference on (pp. 91-98). IEEE.
    [2] Hoffmann, H., Pastor, P., Park, D. H., & Schaal, S. (2009, May).
        Biologically-inspired dynamical systems for movement generation:
        automatic real-time goal adaptation and obstacle avoidance.
        In Robotics and Automation, 2009. ICRA'09. IEEE International Conference
        on (pp. 2587-2592). IEEE.
    """

    def __init__(self,
        n_dmps = 3, n_bfs = 50, dt = 0.01, x0 = None, goal = None, T = 1.0,
        K = 1050, D = None, w = None, tol = 0.01, alpha_s = 4.0,
        rescale = False, basis = 'gaussian', **kwargs):
        """
        n_dmps int   : number of dynamic movement primitives (i.e. dimensions)
        n_bfs int    : number of basis functions per DMP (actually, they will be
                       one more)
        dt float     : timestep for simulation
        x0 array     : initial state of DMPs
        goal array   : goal state of DMPs
        T float      : final time
        K float      : elastic parameter in the dynamical system
        D float      : damping parameter in the dynamical system
        w array      : associated weights
        tol float    : tolerance
        alpha_s float: constant of the Canonical System
        rescale bool : decide if the rescale property is used
        basis string : type of basis functions
        """
        # Tolerance for the accuracy of the movement: the trajectory will stop
        # when || x - g || <= tol
        self.tol = tol
        self.n_dmps = n_dmps
        self.n_bfs = n_bfs
        # Default values give as in [2]
        self.K = K
        if D is None:
            D = 2 * np.sqrt(self.K)
        self.D = D
        # Set up the CS
        self.cs = CanonicalSystem(dt = dt, run_time = T, alpha_s = alpha_s)
        # Create the matrix of the linear component of the problem
        self.compute_linear_part()
        # Set up the DMP system
        if x0 is None:
            x0 = np.zeros(self.n_dmps)
        if goal is None:
            goal = np.zeros(self.n_dmps)
        self.x0 = x0
        self.goal = goal
        self.rescale = rescale
        self.basis = basis
        self.reset_state()
        self.gen_centers()
        self.gen_width()
        # If no weights are give, set them to zero
        if w is None:
            w = np.zeros([self.n_dmps, self.n_bfs + 1])
        self.w = w

    def compute_linear_part(self):
        """
        Compute the linear component of the problem.
        """
        self.linear_part = np.zeros([2 * self.n_dmps, 2 * self.n_dmps])
        for d in range(self.n_dmps):
            self.linear_part[2 * d, 2 * d] = - self.D
            self.linear_part[2 * d, 2 * d + 1] = - self.K
            self.linear_part[2 * d + 1, 2 * d] = 1.

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
         s : array containing the rollout of the canonical system
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
                       (32.0 * xi ** 3.0 + 25.0 * xi ** 2.0 + 8.0 * xi + 1.0)) *
                                                                     (xi < 1.0))
        psi_set = np.nan_to_num(psi_set)
        return psi_set

    def gen_weights(self, f_target):
        """
        Generate a set of weights over the basis functions such that the target
        forcing term trajectory is matched.
          f_target shaped n_dim x n_time_steps
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
        for d in range(self.n_dmps):
            # Create the vector of the regression problem
            for k in range(self.n_bfs + 1):
                b[k] = scipy.integrate.simps(
                        f_target[d] * psi_track[k] * s_track / sum_psi, s_track)
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

    def imitate_path(self,
      x_des, dx_des = None, ddx_des = None, t_des = None, g_w = True, **kwargs):
        """
        Takes in a desired trajectory and generates the set of system parameters
        that best realize this path.
          x_des array shaped num_timesteps x n_dmps
          t_des 1D array of num_timesteps component
          g_w boolean, used to separate the one-shot learning from the
                       regression over multiple demonstrations
        """

        ## Set initial state and goal
        self.x0 = x_des[0].copy()
        self.goal = x_des[-1].copy()

        ## Set t_span
        if t_des is None:
            # Default value for t_des
            t_des = np.linspace(0, self.cs.run_time, x_des.shape[0])
        else:
            # Warp time to start from zero and end up to T
            t_des -= t_des[0]
            t_des /= t_des[-1]
            t_des *= self.cs.run_time
        time = np.linspace(0., self.cs.run_time, self.cs.timesteps)

        ## Piecewise linear interpolation
        # Interpolation function
        path_gen = scipy.interpolate.interp1d(t_des, x_des.transpose())
        # Evaluation of the interpolant
        path = path_gen(time)
        x_des = path.transpose()

        ## Second order estimates of the derivatives
        ## (the last non centered, all the others centered)
        if dx_des is None:
            D1 = der_mtrx.compute_D1(self.cs.timesteps, self.cs.dt)
            dx_des = np.dot(D1, x_des)
        else:
            dpath = np.zeros([self.cs.timesteps, self.n_dmps])
            dpath_gen = scipy.interpolate.interp1d(t_des, dx_des)
            dpath = dpath_gen(time)
            dx_des = dpath.transpose()
        if ddx_des is None:
            D2 = der_mtrx.compute_D2(self.cs.timesteps, self.cs.dt)
            ddx_des = np.dot(D2, x_des)
        else:
            ddpath = np.zeros([self.cs.timesteps, self.n_dmps])
            ddpath_gen = scipy.interpolate.interp1d(t_des, ddx_des)
            ddpath = ddpath_gen(time)
            ddx_des = ddpath.transpose()

        ## Find the force required to move along this trajectory
        s_track = self.cs.rollout()
        f_target = ((ddx_des / self.K - (self.goal - x_des) + 
                  self.D / self.K * dx_des).transpose() +
                  np.reshape((self.goal - self.x0), [self.n_dmps, 1]) * s_track)
        if g_w:
            # Efficiently generate weights to realize f_target
            # (only if not called by paths_regression)
            self.gen_weights(f_target)
            self.reset_state()
            self.learned_position = self.goal - self.x0
        return f_target

    def paths_regression(self, traj_set, t_set = None):
        """
        Takes in a set (list) of desired trajectories (with possibly the
        execution times) and generate the weight which realize the best
        approximation.
          each element of traj_set should be shaped num_timesteps x n_dim
          trajectories
        """
        ## Step 1: Generate the set of the forcing terms
        f_set = np.zeros([len(traj_set), self.n_dmps, self.cs.timesteps])
        g_new = np.ones(self.n_dmps)
        for it in range(len(traj_set)):
            if t_set is None:
                t_des_tmp = None
            else:
                t_des_tmp = t_set[it]
                t_des_tmp -= t_des_tmp[0]
                t_des_tmp /= t_des_tmp[-1]
                t_des_tmp *= self.cs.run_time
            # Alignment of the trajectory so that
            # x0 = [0; 0; ...; 0] and g = [1; 1; ...; 1].
            x_des_tmp = traj_set[it]
            x_des_tmp -= x_des_tmp[0] # translation to x0 = 0
            g_old = x_des_tmp[-1] # original goal position
            R = roto_dilatation(g_old, g_new) # rotodilatation
            # Rescaled and rotated trajectory
            x_des_tmp = np.dot(x_des_tmp, np.transpose(R))
            # Learning of the forcing term for the particular trajectory
            f_tmp = self.imitate_path(x_des = x_des_tmp, t_des = t_des_tmp,
                                                  g_w = False, add_force = None)
            f_set[it, :, :] = f_tmp.copy() # add the new forcing term to the set
        ## Step 2: Learning of the weights using linear regression
        self.w = np.zeros([self.n_dmps, self.n_bfs + 1])
        s_track = self.cs.rollout()
        psi_set = self.gen_psi(s_track)
        psi_sum = np.sum(psi_set, 0)
        psi_sum_2 = psi_sum * psi_sum
        s_track_2 = s_track * s_track
        A = np.zeros([self.n_bfs + 1, self.n_bfs + 1])
        for k in range(self.n_bfs + 1):
            A[k, k] = scipy.integrate.simps(
                 psi_set[k, :] * psi_set[k, :] * s_track_2 / psi_sum_2, s_track)
            for h in range(k + 1, self.n_bfs + 1):
                A[h, k] = scipy.integrate.simps(
                          psi_set[k, :] * psi_set[h, :] * s_track_2 / psi_sum_2,
                                                                        s_track)
                A[k, h] = A[h, k].copy()
        A *= len(traj_set)
        LU = scipy.linalg.lu_factor(A)
        # The weights are learned dimension by dimension
        for d in range(self.n_dmps):
            f_d_set = f_set[:, d, :].copy()
            # Set up the minimization problem
            b = np.zeros([self.n_bfs + 1])
            for k in range(self.n_bfs + 1):
                b[k] = scipy.integrate.simps(
                         np.sum(f_d_set * psi_set[k, :] * s_track / psi_sum, 0),
                                                                        s_track)
            # Solve the minimization problem
            self.w[d, :] = scipy.linalg.lu_solve(LU, b)
        self.learned_position = np.ones(self.n_dmps)

    def reset_state(self, v0 = None, **kwargs):
        """
        Reset the system state
        """
        self.x = self.x0.copy()
        if v0 is None:
            v0 = 0.0 * self.x0
        self.dx = v0
        self.ddx = np.zeros(self.n_dmps)
        self.cs.reset_state()

    def rollout(self, tau = 1.0, v0 = None, **kwargs):
        """
        Generate a system trial, no feedback is incorporated.
          tau scalar, time rescaling constant
          v0 scalar, initial velocity of the system
        """
        # Reset the state of the DMP
        if v0 is None:
            v0 = 0.0 * self.x0
        self.reset_state(v0 = v0)
        # Set up tracking vectors
        x_track = np.zeros((0, self.n_dmps))
        dx_track = np.zeros((0, self.n_dmps))
        ddx_track = np.zeros((0, self.n_dmps))
        # Add the initial value to the tracking vectors
        x_track = np.append(x_track, [self.x0], axis = 0)
        dx_track = np.append(dx_track, [v0], axis = 0)
        ddx_track = np.append(ddx_track, [0.0 * self.x0], axis = 0)
        flag_stop = False # flag to decide if the execution has to be stopped
        t = 0
        t_track = [0]
        # In rollout there is no need to compute phi1 (dt A) all the times
        while (not flag_stop):
            # Run and record timestep
            x_track_s, dx_track_s, ddx_track_s = self.step(tau = tau)
            x_track = np.append(x_track, [x_track_s], axis=0)
            dx_track = np.append(dx_track, [dx_track_s],axis=0)
            ddx_track = np.append(ddx_track, [ddx_track_s],axis=0)
            t_track.append(t_track[-1] + self.cs.dt)
            t += 1
            stop_c = (np.linalg.norm(x_track_s - self.goal) /
                                 np.linalg.norm(self.x0 - self.goal)) < self.tol
            stop_cs = (self.cs.s <
                        np.exp(- self.cs.alpha_s * self.cs.run_time) + self.tol)
            flag_stop = stop_c and stop_cs
        return x_track, dx_track, ddx_track, t_track

    def step(self, tau = 1.0, error = 0.0, external_force = None, **kwargs):
        """
        Run the DMP system for a single timestep.
          P array : phi1 (dt A). If None it is recomputed
          tau float: time rescaling constant
          error float: optional system feedback
          external_force 1D array: external force to add to the system
        """
        error_coupling = 1.0 / (1.0 + error)
        # Generate basis function activation
        s = self.cs.s
        psi = self.gen_psi(s)
        f = np.zeros(self.n_dmps)
        # Initialize the integration scheme
        state = np.zeros(2 * self.n_dmps)
        b_tilde = np.zeros(2 * self.n_dmps)
        f = (np.dot(self.w, psi[:, 0])) / (np.sum(psi[:, 0])) * s
        f = np.nan_to_num(f)
        if self.rescale:
            new_position = self.goal - self.x0
            M = roto_dilatation(self.learned_position, new_position)
            f = np.dot(M, f)
        # Set the state vector and the affine part of the scheme
        # Linear part
        A_m = self.linear_part / tau
        P_m = phi1(self.cs.dt * A_m)
        # State definition
        state[range(0, 2 * self.n_dmps, 2)] = self.dx
        state[range(1, 2 * self.n_dmps + 1, 2)] = self.x
        # Affine part
        b_tilde[range(0, 2 * self.n_dmps, 2)] = \
                               self.K * (self.goal * (1. - s) + self.x0 * s + f)
        if external_force is not None:
            b_tilde[range(0, 2 * self.n_dmps, 2)] += external_force
        b_tilde[range(1, 2 * self.n_dmps + 1, 2)] = 0.0
        b_tilde /= tau
        # Vector field
        vect_field = np.dot(A_m, state) + b_tilde
        # Integration step
        state += self.cs.dt * np.dot(P_m, vect_field)
        self.x = state[range(1, 2 * self.n_dmps + 1, 2)]
        self.dx = state[range(0, 2 * self.n_dmps, 2)]
        self.ddx = (self.K / (tau ** 2.0) * 
                       ((self.goal - self.x) - (self.goal - self.x0) * s + f)) \
                                                        - self.D / tau * self.dx
        # Run canonical system
        s = self.cs.step(tau = tau, error_coupling = error_coupling)
        return self.x, self.dx, self.ddx

    if __name__ == '__main__':
        import matplotlib.pyplot as plt
        from matplotlib import rc
        rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
        rc('text', usetex=True)
        from dmp_cartesian import DMPs_cartesian as dmps
        """
        First demo: lerning and generalization of  trajectory
        """
        t_steps = 10 ** 3
        tf = 1.0
        t = np.linspace (0.0, tf, t_steps)
        x = 2.0 * t
        y = np.sin(2.0 * np.pi * t) + 1.0 + t / 2.0
        x_des = np.zeros([t_steps, 2])
        x_0 = x_des[0]
        x_des[:, 0] = x
        x_des[:, 1] = y
        # Learning the forcing term
        myK = 1000
        dmp = dmps(n_dmps = 2, n_bfs = 50, K = myK, rescale = True, tol = 0.05)
        dmp.imitate_path (x_des = x_des)
        # Classical rollouut
        x_track, _, _, _ = dmp.rollout()
        x_classical = x_track.copy()
        ## Rollout with different initial position
        dmp.x0 = np.array([0.0, 0.0])
        dmp.reset_state()
        x_track, _, _, _ = dmp.rollout()
        x_moved_r = x_track.copy()
        dmp.rescale = False
        x_track, _, _, _ = dmp.rollout()
        x_moved_nr = x_track.copy()
        fig = plt.figure(1)
        plt.plot(x, y, '-', label = 'desired')
        plt.plot(x_classical[:, 0], x_classical[:, 1], ':', label = 'classical imitation')
        plt.plot(x_moved_r[:, 0], x_moved_r[:, 1], '--', label = 'moved goal with rescale')
        plt.plot(x_moved_nr[:, 0], x_moved_nr[:, 1], '--', label = 'moved goal without rescale')
        plt.legend(loc = 'best')
        plt.axis('equal')
        plt.title(r'Different $x_0$')
        ## Rollout with different goal
        dmp.x0 = x_0 # original starting point
        dmp.reset_state()
        dmp.goal = np.array ([- 0.2, - 1.0])
        dmp.rescale = True
        x_track, _, _, _ = dmp.rollout()
        x_moved_r = x_track.copy()
        dmp.rescale = False
        x_track, _, _, _ = dmp.rollout()
        x_moved_nr = x_track.copy()
        fig = plt.figure(2)
        plt.plot(x, y, '-', label = 'desired')
        plt.plot(x_classical[:, 0], x_classical[:, 1], ':', label = 'classical imitation')
        plt.plot(x_moved_r[:, 0], x_moved_r[:, 1], '--', label = 'moved goal with rescale')
        plt.plot(x_moved_nr[:, 0], x_moved_nr[:, 1], '--', label = 'moved goal without rescale')
        plt.legend(loc = 'best')
        plt.axis('equal')
        plt.title(r'Different goal')
        ## Rollout with both different initial position and goals
        dmp.x0 = np.array([0.0, 0.0])
        dmp.reset_state()
        dmp.goal = np.array ([- 0.2, - 1.0])
        dmp.rescale = True
        x_track, _, _, _ = dmp.rollout()
        x_moved_r = x_track.copy()
        dmp.rescale = False
        x_track, _, _, _ = dmp.rollout()
        x_moved_nr = x_track.copy()
        fig = plt.figure(3)
        plt.plot(x, y, '-', label = 'desired')
        plt.plot(x_classical[:, 0], x_classical[:, 1], ':', label = 'classical imitation')
        plt.plot(x_moved_r[:, 0], x_moved_r[:, 1], '--', label = 'moved goal with rescale')
        plt.plot(x_moved_nr[:, 0], x_moved_nr[:, 1], '--', label = 'moved goal without rescale')
        plt.legend(loc = 'best')
        plt.axis('equal')
        plt.title(r"Different $x_0$ and goal")
        plt.show()
        """
        Second demo: regression on multiple trajectories
        """
        num_traj = 50
        traj_set = []
        t_set = []
        # The trajectories will be generated by numerically integrating a dynamical system
        def RK4(x0, m, tf):
            def fun(x):
                x1 = x[0]
                x2 = x[1]
                f1 = x1 ** 3 + x2 ** 2 * x1 - x1 - x2
                f2 = x2 ** 3 + x1 ** 2 * x2 + x1 - x2
                return np.array([f1, f2])
            X = np.zeros([m, len(x0)])
            X[0] = x0.copy()
            dt = tf / (m - 1)
            x = x0
            for n in range(m - 1):
                K1 = fun(x)
                K2 = fun(x + dt * K1 / 2.)
                K3 = fun(x + dt * K2 / 2.)
                K4 = fun(x + dt * K3)
                x += dt * (K1 + 2 * K2 + 2 * K3 + K4) / 6.
                X[n+1] = x
            return X
        # Trajectory set generation
        for i in range(num_traj):
            ## Random select x0, tf, and m
            theta = np.random.rand() * 2.0 * np.pi
            rho = 1.0 - np.random.rand() / 5.0
            x0 = rho * np.array([np.cos(theta), np.sin(theta)])
            tf = 6.0 + 2.0 * (np.random.rand() - 0.5)
            m = int (500 + np.floor(500 * np.random.rand()))
            t_set.append(np.linspace(0.0, tf, m))
            # Execute the trajectory
            X = RK4(x0, m, tf)
            traj_set.append(X.copy())
            # Plot
            plt.figure(1)
            plt.plot(X[:, 0], X[:, 1], '-b', lw = 0.5)
            plt.axis('equal')
            # Plot after translation and roto dilatation
            Z = X - X[0]
            old_pos = Z[- 1]
            R = roto_dilatation(old_pos, np.array([1, 1]))
            Z = np.dot(Z, np.transpose(R))
            plt.figure(2)
            plt.plot(Z[:, 0], Z[:, 1], '-b', lw = 0.5)
            plt.axis('equal')
        # DMP learning and execution
        MP = dmps(n_dmps = 2, n_bfs = 50, K = 1000, alpha_s = 4.0,rescale = True, T = 2.0)
        MP.paths_regression(traj_set, t_set)
        x_track, dx_track, ddx_track, _ = MP.rollout()
        plt.figure(2)
        plt.plot(x_track[:, 0], x_track[:, 1], '-r')
        plt.title(r'Traj. set (blue) and DMP (red) in the "aligned" space')
        # Plot from an arbitrary position
        theta = 2.0 * np.pi * np.random.rand()
        rho = 1.0 - np.random.rand() / 5.0
        MP.x0 = rho * np.array([np.cos(theta), np.sin(theta)])
        MP.goal = np.zeros(2)
        x_track, dx_track, ddx_track, _ = MP.rollout()
        plt.figure(1)
        plt.plot(x_track[:, 0], x_track[:, 1], '-r')
        plt.title(r'Traj. set (blue) and DMP (red) in the "original" space')
        plt.show()