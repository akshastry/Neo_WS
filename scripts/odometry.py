#!/usr/bin/env python

#python imports
import time
import math
import traceback
import numpy as np
from scipy.spatial.transform import Rotation as R
#PX4flow import
from px_comm.msg import OpticalFlow
#ROS imports
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import UInt8



mass = 1.46; # mass of quad
g = 9.81;# acceleration due to gravity
Thrust_sf = 2.1/(mass*g);
Z_d = -0.2;
radio_on = 0
LP_lidar = 0.3
ground_distance = 0.06


Kp_x = 1.0
Ki_x = 0.0025

Kp_y = 1.0
Ki_y = 0.001

Kp_z = 1.5
Kd_z = 0.01
Ki_z = 0.0001

err_sum_x = 0.0
err_sum_y = 0.0
err_sum_z = 0.0

main_running = False

flow_x = 0.0
flow_y = 0.0
ground_distance_sonar = 1.0
ground_distance_Lidar = 1.0
end_time = 0.0

#focal length of optic flow camera
focal_length = (16.0/24.0) *1000.0 # (focal length in mm/ pixel size in micrometer) *1000 = focal length in pixels

#distance of the flow sensor and lidar from body center of mass
r_flow_sensor = np.array([0,-0.011,0.042])
Lidar_offset_from_flow = -0.8/100.0

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

Z_k = np.zeros(3)
z_prev = 0.0

## covariances
P_k[0,0] = 0
P_k[1,1] = 0
P_k[2,2] = 10.0
P_k[3,3] = 10.0
P_k[4,4] = 10.0
P_k[5,5] = 10.0

#disturbance through acceleration in body axes
A_k[0,0] = 10**(-1)
A_k[1,1] = 10**(-1)
A_k[2,2] = 10**(-1)

#noise in planar velocities and ground distance measurements
R_k[0,0] = 1.0*10**(-4) # u covariance in (m/s)^2
R_k[1,1] = 1.0*10**(-4) # v covariance in (m/s)^2
R_k[2,2] = 0.1*10**(-4) # ground distance measurement covariance error in m^2

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
	K_k		= np.linalg.solve(S_k, np.matmul(H_k, P_k))
	K_k 	= np.transpose(K_k)

	X_k 	= X_k + np.transpose(np.matmul(K_k, np.reshape(y_tilda, (-1, 1))))[0]
	# P_k		= P_k - np.matmul(np.matmul(K_k, H_k), P_k)
	# Joseph stabilized version of Kalman, implement square root kalman in future
	T_k		= I - np.matmul(K_k, H_k)
	P_k		= np.matmul(np.matmul(T_k, P_k), T_k.T) + np.matmul(np.matmul(K_k, R_k), K_k.T)
	P_k		= 0.5 * (P_k + P_k.T)
	# C_k     = np.linalg.cholesky(P_k)
	# P_k		= C_k * C_k.T
	# print(P_k - P_k.T)
	# np.linalg.cholesky(P_k)


def px4_callback(data):
	global flow_x, flow_y, ground_distance_sonar, main_running
	if (~main_running):
		if(math.isnan(data.ground_distance) == False and math.isnan(data.flow_x) == False and math.isnan(data.flow_y) == False):
			if(data.ground_distance > 0.0):
				ground_distance_sonar = data.ground_distance
				flow_x = data.velocity_x / ground_distance_sonar 
				flow_y = data.velocity_y / ground_distance_sonar

def lidar_callback(data):
	global ground_distance_Lidar
	if (~main_running):
		if(math.isnan(data.data) == False and data.data > 0.0):
			ground_distance_Lidar = data.data

def Fcon_callback(data):
	global phi, theta, psi, p, q, r, main_running
	
	if (~main_running):
		phi 	= data.linear.x * np.pi/180.0
		# print(phi*180.0/np.pi)
		theta 	= data.linear.y * np.pi/180.0
		psi 	= data.linear.z * np.pi/180.0

		p 	= data.angular.x * np.pi/180.0
		q 	= data.angular.y * np.pi/180.0
		r 	= data.angular.z * np.pi/180.0

def Rdo_callback(data):
	global err_sum_x, err_sum_y, err_sum_z, main_running, Z_d, radio_on, X_k
	
	if (~main_running):
		if(data.data == 0):
			err_sum_x = 0.0
			err_sum_y = 0.0
			err_sum_z = 0.0
		# if(radio_on == 0 and data.data == 1):
		# 	Z_d = X_k[2]
		radio_on = data.data


def update_Rotmat():
	global I_R_B, Omega_cross, phi, theta, psi, p, q, r

	rot_mat = R.from_euler('ZYX', [psi, theta, phi])
	I_R_B = rot_mat.as_dcm() #replace with as_matrix for scipy 1.4.0 and after
	Omega_cross = np.array([[0, -r, q], [r, 0, -p], [-q, p, 0]])

def update_Kalman_mat():
	global F_k, G_k, A_k, Q_k, H_k, I_R_B, Omega_cross, dt, phi, theta, dt

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
	H_k[2,2] = -1.0/(np.cos(phi) * np.cos(theta))


def constrain(x, a, b):
	if(x<a):
		x = a
	if(x>b):
		x = b
	return x

def main():
	global flow_x, flow_y, ground_distance_Lidar, main_running, Hz, X_k, Z_k, r_flow_sensor, Omega_cross, Lidar_offset_from_flow, \
	 dt, end_time, mass, g, Kp_x, Kp_y, Kp_z, Ki_x, Ki_y, Ki_z, Kd_z, err_sum_x, err_sum_y, err_sum_z, radio_on, LP_lidar, \
	 ground_distance, phi, theta, z_prev

	ctrl = Twist();
	states = Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/px4flow/opt_flow', OpticalFlow, px4_callback)
	rospy.Subscriber('/lidarlite/distance', Float64, lidar_callback)
	rospy.Subscriber('/serialcom/attitude_and_rate', Twist, Fcon_callback)
	rospy.Subscriber('/serialcom/radio', UInt8, Rdo_callback)
	end_time = time.time()
	pub = rospy.Publisher('/neo/odometry', Twist, queue_size=1)
	pub1 = rospy.Publisher('/neo/states', Twist, queue_size=1)

	rate = rospy.Rate(Hz)#100 Hz
	while not rospy.is_shutdown():
		main_running = True
		# print(X_k[2])
		try:
			dt = time.time() - end_time
			# print(dt)
			
			# update rotation matrices based on current sensor reading
			# update_Rotmat()
			# update_Kalman_mat()

			# Uodate measurement
			v_flow_sensor = np.matmul(Omega_cross, r_flow_sensor)
			
			ground_distance = LP_lidar * ((ground_distance_Lidar - 5.0)/100.0 + Lidar_offset_from_flow) + (1.0-LP_lidar) * ground_distance
			
			Z_k[0] = -flow_x*ground_distance - v_flow_sensor[1]
			Z_k[1] = -flow_y*ground_distance - v_flow_sensor[2]
			Z_k[2] = ground_distance + r_flow_sensor[2]
			
			# filter
			# EKF()
			
			# derive control
			# zdd = Kp_z * (Z_d - X_k[2]) + Kd_z * (0.0 - X_k[5]) #+ Ki_z * err_sum_z
			z = -Z_k[2]/(np.cos(phi) * np.cos(theta))
			zd = (z - z_prev)/dt
			z_prev = z
			zdd = Kp_z * (Z_d - z) + Kd_z * (0.0 - zd)
			ydd = Kp_y * (0.0 - X_k[4]) + Ki_y * err_sum_y
			xdd = Kp_x * (0.0 - X_k[3]) + Ki_x * err_sum_x

			T_d 	= mass * (g - zdd)
			phi_d 	= ydd/g
			theta_d = -xdd/g
			psi_d 	= 0.0

			err_sum_x = constrain(err_sum_x + (0.0 - X_k[3]), -2.0/Ki_x, 2.0/Ki_x)
			err_sum_y = constrain(err_sum_y + (0.0 - X_k[4]), -2.0/Ki_y, 2.0/Ki_y)
			err_sum_z = constrain(err_sum_z + (Z_d - X_k[2]), -1.0/Ki_z, 1.0/Ki_z)

			phi_d = constrain(phi_d, -np.pi/6.0, np.pi/6.0)
			theta_d = constrain(theta_d, -np.pi/6.0, np.pi/6.0)

			
			# publish to a topic
			ctrl.linear.x 	= radio_on
			ctrl.linear.y 	= 0.0
			ctrl.linear.z 	= T_d * Thrust_sf
			ctrl.angular.x 	= phi_d * 180.0/np.pi
			ctrl.angular.y 	= theta_d * 180.0/np.pi
			ctrl.angular.z 	= psi_d * 180.0/np.pi
			pub.publish(ctrl);

			states.linear.x 	= X_k[0]
			states.linear.y 	= X_k[1]
			states.linear.z 	= X_k[2]
			states.angular.x 	= X_k[3]
			states.angular.y 	= X_k[4]
			states.angular.z 	= X_k[5]
			pub1.publish(states);


			# print(ground_distance_sonar*100.0)
		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in px4.py')
			indent = 1

		end_time = time.time()
		main_running = False

		
		rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass