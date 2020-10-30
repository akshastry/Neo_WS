#!/usr/bin/env python

#python imports
import traceback
import numpy as np
from scipy.spatial.transform import Rotation as R
import Lidar_Lite
#PX4flow import
from px_comm.msg import OpticalFlow
#ROS imports
import rospy
from geometry_msgs.msg import Twist



# from 


main_running = False

flow_x = 0.0
flow_y = 0.0
ground_distance_sonar = 0.0

#focal length of optic flow camera
focal_length = (16.0/24.0) *1000.0 # (focal length in mm/ pixel size in micrometer) *1000 = focal length in pixels

#distance of the flow sensor and lidar from body center of mass
r_flow_sensor = np.array([0,0,0])
Lidar_offset_from_flow = 0.0

#KF matrices
X_k = np.zeros(6)
P_k = np.zeros((6,6))

F_k = np.zeros((6,6))
G_k = np.zeros((6,3))
A_k = np.zeros((3,3))
Q_k = np.zeros((6,6))

H_k = np.zeros((3,6))
H_k[0,3] = 1
H_k[1,4] = 1
R_k = np.zeros((3,3))

I = np.eye(6)

## covariances
P_k[0,0] = 1000.0
P_k[1,1] = 1000.0
P_k[2,2] = 1000.0
P_k[3,3] = 1000.0
P_k[4,4] = 1000.0
P_k[5,5] = 1000.0

#disturbance through acceleration in body axes
A_k[0,0] = 10**(0)
A_k[1,1] = 10**(0)
A_k[2,2] = 10**(0)

#noise in planar velocities and ground distance measurements
R_k[0,0] = 10**(-4) # u covariance in (m/s)^2
R_k[1,1] = 10**(-4) # v covariance in (m/s)^2
R_k[2,2] = 10**(-4) # ground distance measurement covariance error in m^2

phi 	= 0.0
theta 	= 0.0
psi 	= 0.0
p 		= 0.0
q 		= 0.0
r 		= 0.0
Hz 		= 100
dt 		= 1.0/(1.0*Hz)
I_R_B 	= np.eye(3)
Omega_cross = np.zeros((3,3))

def EKF():
	global X_k, P_k, F_k, Q_k, Z_k, H_k, R_k, I
	# predict
	X_k = np.transpose(np.matmul(F_k, np.reshape(X_k, (-1, 1))))[0]
	P_k = np.matmul(np.matmul(F_k, P_k), np.transpose(F_k)) + Q_k

	#update
	y_tilda = Z_k - np.transpose(np.matmul(H_k, np.reshape(X_k, (-1, 1))))[0]
	S_k 	= np.matmul(np.matmul(H_k, P_k), np.transpose(H_k)) + R_k
	K_k		= np.linalg.solve(S_k, np.matmul(P_k, np.transpose(H_k)))

	X_k 	= X_k + np.transpose(np.matmul(K_k, np.reshape(y_tilda, (-1, 1))))[0]
	P_k		= np.matmul(I - np.matmul(K_k, H_k), P_k)


def px4_callback(data):
	global flow_x, flow_y, ground_distance_sonar, main_running
	if (~main_running):
		ground_distance_sonar = data.ground_distance
		flow_x = data.velocity_x / ground_distance_sonar 
		flow_y = data.velocity_y / ground_distance_sonar

def Fcon_callback(data):
	global phi, theta, psi, p, q, r, main_running
	
	if (~main_running):
		phi 	= data.linear.x
		theta 	= data.linear.y
		psi 	= data.linear.z

		p 	= data.angular.x
		q 	= data.angular.y
		r 	= data.angular.z

def update_Rotmat():
	global I_R_B, Omega_cross, phi, theta, psi, p, q, r

	r = R.from_euler('ZYX', [psi, theta, phi])
	I_R_B = r.as_dcm() #replace with as_matrix for scipy 1.4.0 and after
	Omega_cross = np.array([[0, -r, q], [r, 0, -p], [-q, p, 0]])

def update_Kalman_mat():
	global F_k, G_k, A_k, Q_k, H_k, I_R_B, Omega_cross, dt, phi, theta

	F_k = I
	F_k[0:3, 3:6] = F_k[0:3, 3:6] + I_R_B * dt
	F_k[3:6, 3:6] = F_k[3:6, 3:6] - Omega_cross * dt

	G_k = np.zeros((6,3))
	G_k[0:3,0:3] = 0.5 * dt * dt * I_R_B
	G_k[3,0] = dt
	G_k[4,1] = dt
	G_k[5,2] = dt

	Q_k = np.matmul(np.matmul(G_k, A_k), np.transpose(G_k))

	H_k = np.zeros((3,6))
	H_k[0,3] = 1
	H_k[1,4] = 1
	H_k[2,2] = 1.0/(np.cos(phi) * np.cos(theta))

def main():
	global flow_x, flow_y, ground_distance_sonar, main_running, Hz, X_k, Z_k, r_flow_sensor, Omega_cross, Lidar_offset_from_flow
	L1 = Lidar_Lite.LiDAR_Lite()
	L1.connect(1)
	b = 2.0/(18.0+1.0)
	dist = 1;

	ctrl = Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/px4flow/opt_flow', OpticalFlow, px4_callback)
	rospy.Subscriber('shastry/serialcom/from_Fcon', Twist, Fcon_callback)
	pub = rospy.Publisher('/odometry/outer_loop_cmd', Twist, queue_size=1)

	rate = rospy.Rate(Hz)#100 Hz
	while not rospy.is_shutdown():
		main_running = True
		try:
			# update rotation matrices based on current sensor reading
			update_Rotmat()
			v_flow_sensor = np.matmul(Omega_cross, r_flow_sensor)
			# get data
			ground_distance_Lidar = L1.get_distance() + Lidar_offset_from_flow

			Z_k[0] = flow_x*ground_distance_Lidar - v_flow_sensor[1]
			Z_k[1] = flow_y*ground_distance_Lidar - v_flow_sensor[2]
			Z_k[2] = ground_distance_Lidar + r_flow_sensor[2]
			
			# filter

			update_Kalman_mat()
			EKF()

			# derive control
			T_d 	= 
			phi_d 	= 
			theta_d = 
			psi_d 	= 

			
			# publish to a topic
			ctrl.linear.x 	= 0.0
			ctrl.linear.y 	= 0.0
			ctrl.linear.z 	= T_d
			ctrl.angular.x 	= phi_d
			ctrl.angular.y 	= theta_d
			ctrl.angular.z 	= psi_d

			pub.publish(ctrl);

			# print(ground_distance_sonar*100.0)
		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in px4.py')
			indent = 1

		main_running = False
		rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass