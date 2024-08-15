import numpy as np
import pdb

def arc_length_param(gamma, t_span = None, num_steps = None, **kwargs):
    """
    Perform a parametrization in arch length of the curve.
    The curve should be shaped N x d, with N timesteps and d dimensions
    """
    num_ts_old = np.size(gamma, 0)
    if t_span is None:
        t_span = np.linspace(0.0, 1.0, num_ts_old)
    else:
        # Create the interpolant
        from scipy.interpolate import interp1d
        gamma_fun = interp1d(t_span, np.transpose(gamma))
        t_span = np.linspace(t_span[0], t_span[-1], num_ts_old)
        gamma = np.transpose(gamma_fun(t_span))
    n_ts = np.size(t_span)
    dt = 1.0 / (num_ts_old - 1)
    # Compute gamma'
    import scipy.sparse as sparse
    d1_p = np.ones([n_ts - 1])
    d1_p[0] = 4.
    d1_m = - np.ones([n_ts-1])
    d1_m[-1] = - 4.
    D1 = sparse.diags(np.array([d1_p, d1_m]), [1, -1]).toarray()
    D1[0,0] = - 3.
    D1[0, 2] = -1.
    D1[-1, -3] = 1.
    D1[-1,-1] = 3.
    D1 /= 2 * dt
    dgamma = np.array(D1 * np.matrix(gamma))
    # Compute the arc length
    norm_dgamma = np.linalg.norm(dgamma, axis = 1) # || gamma ' ||
    from scipy.integrate import simps
    al = np.zeros(num_ts_old)
    for i in range(num_ts_old):
        ndg_tmp = norm_dgamma[0:i+1]
        t_tmp = t_span[0:i+1]
        al[i] = simps(ndg_tmp, t_tmp)
    gamma_s_fun = interp1d(al, np.transpose(gamma))
    if num_steps is None:
        num_steps = n_ts
    s_span_eq = np.linspace(al[0], al[-1], num_steps)
    gamma_s = np.transpose(gamma_s_fun(s_span_eq))
    return gamma_s

if __name__ == "__main__":
    from al_param import arc_length_param
    import matplotlib.pyplot as plt
    # Create the trajectory
    ts = 20
    t = np.linspace(0.0, np.sqrt(2.0 * np.pi), ts)
    theta = t ** 2
    rho = 1.0 + t
    x = rho * np.cos(theta)
    y = rho * np.sin(theta)
    plt.figure()
    plt.plot(x, y, '--x')
    gamma = np.zeros([ts, 2])
    gamma[:, 0] = x
    gamma[:, 1] = y
    gamma_s = arc_length_param(gamma = gamma, t_span = t, num_steps = 10)
    plt.figure()
    plt.plot(gamma_s[:, 0], gamma_s[:,1], '--r')
    plt.plot(gamma_s[:, 0], gamma_s[:,1], 'xr')
    plt.show()