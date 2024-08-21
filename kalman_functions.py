import numpy as np

def load_data(filename):
    """
    loads the data from the given filename and returns t, u and z
    It also constructs and loads the right noise matrices.
    """

    # Values returns a numpy representation of the df
    quadrotor_data = np.loadtxt(filename,delimiter=',')
    timestamps = quadrotor_data[:,0]
    u = quadrotor_data[:,1:4]
    z = quadrotor_data[:, 4:7]

    # Get the right noise matrices

    # R is the measurement noise. R is given sigma**2 * I

    #C is the measurement matrix. It is used as H here

    if filename == "kalman_filter_data_velocity.txt":
        sigma_Q = 0.01
        R = np.eye(3) * (0.05)**2

        C = np.array([0,0,0,1,0,0,
                    0,0,0,0,1,0,
                    0,0,0,0,0,1]).reshape(3,6)

    elif filename == "kalman_filter_data_low_noise.txt":
        sigma_Q = 0.1
        R = np.eye(3) * (0.05)**2

        C = np.array([1,0,0,0,0,0,
                    0,1,0,0,0,0,
                    0,0,1,0,0,0]).reshape(3,6)
        
    elif filename == "kalman_filter_data_high_noise.txt":
        sigma_Q = 0.1
        R = np.eye(3) * (0.20)**2

        C = np.array([1,0,0,0,0,0,
                    0,1,0,0,0,0,
                    0,0,1,0,0,0]).reshape(3,6)
        
    else:
        sigma_Q = 0.1
        R = np.eye(3) * (0.01)**2
        C = np.array([1,0,0,0,0,0,
                    0,1,0,0,0,0,
                    0,0,1,0,0,0]).reshape(3,6)

    return timestamps, u, z, sigma_Q, R, C

def get_mean_diff_time(time):
    diff_t = np.mean(np.diff(time))
    return diff_t

def get_system_model(diff_t, sigma_Q, m = 0.027):

    F = np.eye(6)
    F[0, 3] = diff_t
    F[1, 4] = diff_t
    F[2, 5] = diff_t

    G = np.zeros((6, 3))
    G[3:6, :] = np.eye(3) * diff_t / m

    # https://web.archive.org/web/20220202010928/https:/www.kalmanfilter.net/covextrap.html
    # Q was established using the above link
    Q = sigma_Q**2 * (G @ G.T)

    return F, G, Q

def process_step(u,x_hat,F,G,P,Q):
    """
    Standard equations for kalman filtering
    """

    u = u.reshape(3,1)
    x_hat = (F@x_hat) + (G@u)
    P = F @P @ F.T + Q

    return x_hat,P

def update_step(P,H,R,x_hat,z):
    """
    Standard equations for kalman filtering
    """
    z = z.reshape(3,1)
    K = P @ H .T @ (np.linalg.inv(R + H @P @H.T))
    x_hat = x_hat + K@(z - H@x_hat)
    # P = (np.eye(6) - K @H) @ P # This equation can be numerically unstable
    P = (np.eye(6) - K @ H) @ P @ (np.eye(6) - K @ H).T + K @ R @ K.T

    return K,x_hat,P


def kalman_filter(x_hat, P, u, z, diff_t, m, sig, time, R, H):
    F, G, Q = get_system_model(diff_t, m, sig)
    estimates = []

    for i in range(len(time)):
        x_hat, P = process_step(u[i], x_hat, F, G, P, Q)
        gain,x_hat, P = update_step(P, H, R, x_hat,z[i])
        estimates.append(x_hat.flatten())
    return np.array(estimates)



    




