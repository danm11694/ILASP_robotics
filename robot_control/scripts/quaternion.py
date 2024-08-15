"""
Copyright (C) 2018 Michele Ginesi

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
import pdb
import copy
import quaternion as quat

def conjugate(q):
    """
    Compute the conjugate quaterion conj(q) of q
        
    """
    if (np.ndim(q) == 1):
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]])
    else:
        q_conj = copy.deepcopy(q)
        q_conj[:, 1:4] *= - 1.0
    return q_conj

def distance(q_1, q_2):
    """
    Compute the distance between two quaternions q_1 and q_2:
                        /2 pi,                          if q_1 * conj(q_2) = -1
        d(q_1, q_2) = <|
                        \2 || log (q_1 * conj(q_2)) ||, otherwise
    """
    q_prod = product(q_1, conjugate(q_2))
    if ((q_prod[0] == -1) and (q_prod[1:4] == np.zeros(3)).all()):
        dist = 2 * np.pi
    else:
        dist = 2 * np.linalg.norm(log(q_prod))
    return dist

def estimate_omega(q_set, dt):
    """
    Estimate the angular velocity omega using
       q_2 - q_1     1
      ----------- = --- omega * q_1
          dt         2
    """
    h_q = np.diff(q_set, axis = 0)
    omega = np.zeros([q_set.shape[0], 4])
    inv_q = inverse(q_set)
    inv_q = inv_q[0:-1, :]
    omega[0:-1, :] = product(2.0 * h_q / dt, inv_q)
    omega[-1] = omega[-2]
    return omega[:, 1:4]

def exp(q):
    """
    Compute the quaternion exponential
    """
    if (np.ndim(q) == 1):
        q_exp = np.exp(q[0]) * np.array([1, 0, 0, 0])
        norm_u = np.linalg.norm(q[1:4])
        if (norm_u != 0):
            norm_u = np.linalg.norm(q[1:4])
            q_exp[0] = np.cos(norm_u)
            q_exp[1:4] = q[1:4] * np.sin(norm_u) / norm_u
            q_exp *= np.exp(q[0])
    else:
        q_exp = np.zeros([q.shape[0], 4])
        norm_v = np.linalg.norm(q[:, 1:4], 2, axis = 1)
        a_exp = np.exp(q[:, 0])
        q_exp[:, 0] = np.exp(q[:, 0]) * np.cos(norm_v)
        norm_v = np.reshape(norm_v, [q.shape[0], 1])
        a_exp = np.reshape(a_exp, [q.shape[0], 1])
        q_exp[:, 1:4] = a_exp * q[:, 1:4] * np.sin(norm_v) / norm_v
    return q_exp

def inverse(q):
    """
    Compute the inverse of a quaternion.
    """
    if (np.ndim(q) == 1):
        return conjugate(q) / (norm(q) ** 2)
    else:
        norm_q = np.reshape(np.linalg.norm(q, 2, axis = 1), [q.shape[0], 1])
        return conjugate(q) / (norm_q * norm_q)

def log(q):
    """
    Compute the log of a unit quaternion
    """
    if (np.ndim(q) == 1):
        if (q[1:4] == np.zeros(3)).all():
            q_log = np.zeros(3)
        else:
            q_log = np.arccos(np.clip(np.nan_to_num(q[0]), -1., 1.)) * q[1:4] / np.linalg.norm(q[1:4])
    else:
        ac_v = np.reshape(np.arccos(q[:, 0]), [q.shape[0], 1])
        norm_u = np.linalg.norm(q[:, 1:4], 2, axis = 1)
        norm_u = np.reshape(norm_u, [q.shape[0], 1])
        q_log = ac_v * q[:, 1:4] / norm_u
        norm_u = np.reshape(norm_u, norm_u.shape[0])
        q_log[norm_u == 0, :] = 0
        q_log = np.nan_to_num(q_log)
    return q_log

def norm(q):
    """
    Return the norm of the quaternion q
    """
    if (np.ndim(q) == 1):
        return np.linalg.norm(q)
    else:
        return np.linalg.norm(q, 2, axis = 1)

def normalize(q):
    """
    Compute the normalized quaternion
    """
    if (np.ndim(q) == 1):
        return q / np.linalg.norm(q)
    else:
        return q / np.reshape(quat.norm(q), [q.shape[0], 1])

def product(q_1, q_2):
    """
    Return the quaternion product between two quaternions
    """
    if (np.ndim(q_1) == 1):
        v_1 = q_1[0]
        u_1 = q_1[1:4]
        v_2 = q_2[0]
        u_2 = q_2[1:4]
        q_prod = np.zeros(4)
        q_prod[0] = v_1 * v_2 - np.dot(u_1, u_2)
        q_prod[1:4] = v_1 * u_2 + v_2 * u_1 + np.cross(u_1, u_2)
    else:
        v_1 = np.reshape(q_1[:, 0], [q_1.shape[0], 1])
        u_1 = q_1[:, 1:4]
        v_2 = np.reshape(q_2[:, 0], [q_2.shape[0], 1])
        u_2 = q_2[:, 1:4]
        v_prod = v_1 * v_2
        u_dot = np.reshape(np.sum(u_1 * u_2, axis = 1), [q_1.shape[0], 1])
        u_cross = np.cross(u_1, u_2)
        q_prod = np.zeros([q_1.shape[0], 4])
        q_prod[:, 0] = np.reshape(v_prod - u_dot, q_1.shape[0])
        q_prod[:, 1:4] = v_1 * u_2 + v_2 * u_1 + u_cross
    return q_prod

def rot_matrix_form(q):
    """
    Return the rotation matrix wich gives the same rotation of the unit quaternion q
    """
    # Initialize the matrix
    R = np.zeros((3,3))
    # extract the parameter
    v = q[0]
    u1 = q[1]
    u2 = q[2]
    u3 = q[3]
    R[0,0] = v ** 2 + u1 ** 2 - u2 ** 2 - u3 ** 2
    R[0,1] = 2 * (u1 * u2 - v * u3)
    R[0,2] = 2 * (u1 * u3 + v * u2)
    R[1,0] = 2 * (u1 * u2 + v * u3)
    R[1,1] = v ** 2 - u1 ** 2 + u2 ** 2 - u3 ** 2
    R[1,2] = 2 * (u2 * u3 + v * u1)
    R[2,0] = 2 * (u1 * u3 - v * u2)
    R[2,1] = 2 * (u2 * u3 + v * u1)
    R[2,2] = v ** 2 - u1 ** 2 - u2 ** 2 + u3 ** 2
    return R

def vect(q):
    """
    Return the vector component of quaternion q
    """
    if (np.ndim(q) == 1):
        return q[1:4]
    else:
        return q[:, 1:4]