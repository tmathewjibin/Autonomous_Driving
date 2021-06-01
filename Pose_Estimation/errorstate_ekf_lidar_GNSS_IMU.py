
import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from rotations import angle_normalize, rpy_jacobian_axis_angle, skew_symmetric, Quaternion


#load the data from the pickle files.


with open('data/pt3_data.pkl', 'rb') as file:
    data = pickle.load(file)

# Each element of the data dictionary is stored as an item from the data dictionary, which we
# will store in local variables, described by the following:
#   gt: Data object containing ground truth. with the following fields:
#     a: Acceleration of the vehicle, in the inertial frame
#     v: Velocity of the vehicle, in the inertial frame
#     p: Position of the vehicle, in the inertial frame
#     alpha: Rotational acceleration of the vehicle, in the inertial frame
#     w: Rotational velocity of the vehicle, in the inertial frame
#     r: Rotational position of the vehicle, in Euler (XYZ) angles in the inertial frame
#     _t: Timestamp in ms.
#   imu_f: StampedData object with the imu specific force data (given in vehicle frame).
#     data: The actual data
#     t: Timestamps in ms.
#   imu_w: StampedData object with the imu rotational velocity (given in the vehicle frame).
#     data: The actual data
#     t: Timestamps in ms.
#   gnss: StampedData object with the GNSS data.
#     data: The actual data
#     t: Timestamps in ms.
#   lidar: StampedData object with the LIDAR data (positions only).
#     data: The actual data
#     t: Timestamps in ms.

gt = data['gt']
imu_f = data['imu_f']
imu_w = data['imu_w']
gnss = data['gnss']
lidar = data['lidar']



# plot the ground truth trajectory 

gt_fig = plt.figure()
ax = gt_fig.add_subplot(111, projection='3d')
ax.plot(gt.p[:,0], gt.p[:,1], gt.p[:,2])
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_zlabel('z [m]')
ax.set_title('Ground Truth trajectory')
ax.set_zlim(-1, 5)
plt.show()



# Correct calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.1).
C_li = np.array([
   [ 0.99376, -0.09722,  0.05466],
   [ 0.09971,  0.99401, -0.04475],
   [-0.04998,  0.04992,  0.9975 ]
])

# Incorrect calibration rotation matrix, corresponding to Euler RPY angles (0.05, 0.05, 0.05).
'''C_li = np.array([
     [ 0.9975 , -0.04742,  0.05235],
      [ 0.04992,  0.99763, -0.04742],
      [-0.04998,  0.04992,  0.9975 ] ])'''

t_i_li = np.array([0.5, 0.1, 0.5])

# Transform from the LIDAR frame to the vehicle (IMU) frame.

lidar.data = (C_li @ lidar.data.T).T + t_i_li


#Constants

var_imu_f = 0.10
var_imu_w = 0.25

var_gnss  = 0.01
var_lidar = 1.00  #1.00 part1 #100.00 Part2
R_gnss = np.eye(3) * var_gnss
R_lidar = np.eye(3) * var_lidar

g = np.array([0, 0, -9.81])  # gravity
l_jac = np.zeros([9, 6]) 
#print(l_jac)
l_jac[3:, :] = np.eye(6) 
#print(l_jac)  # motion model noise jacobian
h_jac = np.zeros([3, 9]) 
#print(h_jac)
h_jac[0:3, 0:3] = np.eye(3)  # measurement model jacobian  
F_k = np.eye(9)
Q = np.eye(6)
Q[0:3,0:3] = np.eye(3)* var_imu_f
Q[3:6,3:6] = np.eye(3)* var_imu_w


# Initial Values 

p_est = np.zeros([imu_f.data.shape[0], 3])  # position estimates
v_est = np.zeros([imu_f.data.shape[0], 3])  # velocity estimates
q_est = np.zeros([imu_f.data.shape[0], 4])  # orientation estimates as quaternions
p_cov = np.zeros([imu_f.data.shape[0], 9, 9])

  # covariance matrices at each timestep #WHY THIS SHAPE

# Set initial values.
p_est[0] = gt.p[0]
v_est[0] = gt.v[0]
q_est[0] = Quaternion(euler=gt.r[0]).to_numpy()
p_cov[0] = np.zeros((9,9))  # covariance of estimate
gnss_i  = 0
lidar_i = 0

# Measurement Update 

def measurement_update(sensor_var, p_cov_check, y_k, p_check, v_check, q_check):


    # 3.1 Compute Kalman Gain
    K_k = p_cov_check @(h_jac.T)@(np.linalg.inv((h_jac @ p_cov_check @ h_jac.T) + sensor_var))
     # 3.2 Compute error state

    delta_x = K_k @ (y_k - p_check)

    # 3.3 Correct predicted state
    p_hat = p_check + delta_x[0:3]
    v_hat = v_check + delta_x[3:6]
    q_hat = Quaternion( euler = delta_x[6:9] ).quat_mult_left(q_check)
    # 3.4 Compute corrected covariance()

    p_cov_hat = (np.eye(9) - K_k @ h_jac) @ p_cov_check

    return p_hat, v_hat, q_hat, p_cov_hat

# Filter Loop


for k in range(1, imu_f.data.shape[0]):  # start at 1 b/c we have initial prediction from gt
    delta_t = imu_f.t[k] - imu_f.t[k - 1]  #delta time
    
    C_ns = Quaternion(*q_est[k - 1]).to_mat() 
    
    Q_k = Q * delta_t**2
    # Update state with IMU inputs

    p_est[k] = p_est[k-1] + delta_t * v_est[k-1] + (0.5 * delta_t**2 * (C_ns.dot(imu_f.data[k-1]) + g))
    v_est[k] = v_est[k-1] + (delta_t * ((C_ns.dot(imu_f.data[k-1]) + g)))
    q_est[k] = Quaternion(euler = imu_w.data[k-1] * delta_t).quat_mult_right( q_est[k-1])

    # Propagate uncertainty

    F_k[0:3,3:6] = delta_t * np.eye(3)
    temp = -skew_symmetric(C_ns @ imu_f.data[k-1]) * delta_t
    F_k[3:6,6:9] = temp
    p_cov[k] = F_k @ p_cov[k-1] @ F_k.T + l_jac @ Q_k @ l_jac.T

    # Checking the availability of GNSS and LIDAR measurements

    if np.any(gnss.t == imu_f.t[k]):
        t_gk = np.where(gnss.t == imu_f.t[k])[0][0]
        p_est[k],v_est[k],q_est[k],p_cov[k] = measurement_update(R_gnss, p_cov[k], gnss.data[t_gk], p_est[k], v_est[k], q_est[k])
    if np.any(lidar.t == imu_f.t[k]):
        t_lk = np.where(lidar.t == imu_f.t[k])[0][0]
        p_est[k],v_est[k],q_est[k],p_cov[k] = measurement_update(R_lidar, p_cov[k], lidar.data[t_lk], p_est[k], v_est[k], q_est[k])
    # Update states (save)

#Results and Analysis 

est_traj_fig = plt.figure()
ax = est_traj_fig.add_subplot(111, projection='3d')
ax.plot(p_est[:,0], p_est[:,1], p_est[:,2], label='Estimated')
ax.plot(gt.p[:,0], gt.p[:,1], gt.p[:,2], label='Ground Truth')
ax.set_xlabel('Easting [m]')
ax.set_ylabel('Northing [m]')
ax.set_zlabel('Up [m]')
ax.set_title('Ground Truth and Estimated Trajectory')
ax.set_xlim(0, 200)
ax.set_ylim(0, 200)
ax.set_zlim(-2, 2)
ax.set_xticks([0, 50, 100, 150, 200])
ax.set_yticks([0, 50, 100, 150, 200])
ax.set_zticks([-2, -1, 0, 1, 2])
ax.legend(loc=(0.62,0.77))
ax.view_init(elev=45, azim=-50)
plt.show()


#plot the error for each of the 6 DOF, with estimates for our uncertainty
# included. The error estimates are in blue, and the uncertainty bounds are red and dashed.
# The uncertainty bounds are +/- 3 standard deviations based on our uncertainty (covariance).

error_fig, ax = plt.subplots(2, 3)
error_fig.suptitle('Error Plots')
num_gt = gt.p.shape[0]
p_est_euler = []
p_cov_euler_std = []

# Convert estimated quaternions to euler angles
for i in range(len(q_est)):
    qc = Quaternion(*q_est[i, :])
    p_est_euler.append(qc.to_euler())

    # First-order approximation of RPY covariance
    J = rpy_jacobian_axis_angle(qc.to_axis_angle())
    p_cov_euler_std.append(np.sqrt(np.diagonal(J @ p_cov[i, 6:, 6:] @ J.T)))

p_est_euler = np.array(p_est_euler)
p_cov_euler_std = np.array(p_cov_euler_std)

# Get uncertainty estimates from P matrix
p_cov_std = np.sqrt(np.diagonal(p_cov[:, :6, :6], axis1=1, axis2=2))

titles = ['Easting', 'Northing', 'Up', 'Roll', 'Pitch', 'Yaw']
for i in range(3):
    ax[0, i].plot(range(num_gt), gt.p[:, i] - p_est[:num_gt, i])
    ax[0, i].plot(range(num_gt),  3 * p_cov_std[:num_gt, i], 'r--')
    ax[0, i].plot(range(num_gt), -3 * p_cov_std[:num_gt, i], 'r--')
    ax[0, i].set_title(titles[i])
ax[0,0].set_ylabel('Meters')

for i in range(3):
    ax[1, i].plot(range(num_gt), \
        angle_normalize(gt.r[:, i] - p_est_euler[:num_gt, i]))
    ax[1, i].plot(range(num_gt),  3 * p_cov_euler_std[:num_gt, i], 'r--')
    ax[1, i].plot(range(num_gt), -3 * p_cov_euler_std[:num_gt, i], 'r--')
    ax[1, i].set_title(titles[i+3])
ax[1,0].set_ylabel('Radians')
plt.show()

