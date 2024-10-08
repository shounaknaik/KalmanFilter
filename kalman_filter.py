import matplotlib.pyplot as plt
import numpy as np
from kalman_functions import load_data, get_mean_diff_time,kalman_filter
import pdb

fig, axs = plt.subplots(3,1, figsize=(12, 10), subplot_kw={'projection': '3d'})


def run_kalman(filename):
    # Load data
    time, u, z, sigma_Q, R, C = load_data(filename)
    m = 0.027

    # Get the mean time difference
    diff_t = get_mean_diff_time(time)

    # pdb.set_trace()
    # Set up initial state x_hat
    x_hat = np.zeros((6,1))
    x_hat[:3] = z[0].reshape(3,1)


    #P[0] is set to a large value since we don't know the initial state
    estimates = kalman_filter(x_hat,np.eye(6)*300,u,z,diff_t,m,sigma_Q,time,R,C)

    return estimates

 ####################################################################

gt_track = run_kalman('kalman_filter_data_mocap.txt')


####################################################################
estimates = run_kalman('kalman_filter_data_high_noise.txt')

#Plot estimated_path
ax = axs[0]
ax.plot(estimates[:, 0], estimates[:, 1], estimates[:, 2],color='b',label = "Estimated")
ax.scatter(gt_track[:,0],gt_track[:,1],gt_track[:,2],color='g', label = "Ground truth", s= 5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title('Estimated trajectory for high noise')
 ####################################################################


estimates = run_kalman('kalman_filter_data_low_noise.txt')

#Plot estimated_path
ax = axs[1]
ax.plot(estimates[:, 0], estimates[:, 1], estimates[:, 2],color='b',label = "Estimated")
ax.scatter(gt_track[:,0],gt_track[:,1],gt_track[:,2],color='g', label = "Ground truth",s= 5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title('Estimated trajectory for low noise')

 ####################################################################

estimates = run_kalman('kalman_filter_data_velocity.txt')

#Plot estimated_path
ax = axs[2]
ax.plot(estimates[:, 0], estimates[:, 1], estimates[:, 2],color='b',label = "Estimated")
ax.scatter(gt_track[:,0],gt_track[:,1],gt_track[:,2],color='g', label = "Ground truth", s= 5)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()
ax.set_title('Estimated trajectory for velocity')


plt.show()







