#!/usr/bin/env python

#python imports
import time
import math
import traceback
import numpy as np
from scipy.spatial.transform import Rotation as R
from threading import Lock
#ROS imports
import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import UInt8, Float64

from datetime import datetime
now = datetime.now() # current date and time
filename = "/home/su/Dropbox/Quadrotor_flight_control/Arduino_playground/LOGS/odom_logs/"
filename += now.strftime("%Y_%m_%d_%H_%M_%S")
filename += "speed=2_1.csv"


mass = 1.46; # mass of quad
g = 9.81;# acceleration due to gravity
Thrust_sf = 2.3/(mass*g);

radio_on = 0


Kp_x = 1.0#0.5#0.2#2.5
Kd_x = 1.5#0.001
Ki_x = 0.005

Kp_y = 1.0#0.5#0.2#2.5
Kd_y = 1.5#0.001
Ki_y = 0.005

Kp_z = 5.0#2.0#2.0
Kd_z = 3.5#1.5#0.9
Ki_z = 0.01#0.01

Kp_yaw = 5.0
Ki_yaw = 0.01


err_sum_x = 0.0
err_sum_y = 0.0
err_sum_z = 0.0
err_sum_yaw = 0.0

err_sum_xint = 0.0
err_sum_yint = 0.0

Ki_xint = 0.00035
Ki_yint = 0.001

#
Kp_x_t = 4.0
Kp_y_t = 4.0
Kp_z_t = 5.0

Kd_x_t = 5.5
Kd_y_t = 5.5
Kd_z_t = 5.0

Ki_x_t = 0.001
Ki_y_t = 0.001
Ki_z_t = 0.01


# desired
X_d = 0.65;
Y_d = 0.0;
Z_d = -0.75;
yaw_d 	= 0.0;


# states
X = 0.0
Y = 0.0
Z = 0.0

VX = 0.0
VY = 0.0
VZ = 0.0

yaw 	= 0.0
pitch 	= 0.0
roll 	= 0.0


#Low Pass
LP_X = 1.0
LP_Y = 1.0
LP_Z = 0.8#0.55

LP_VX = 0.9
LP_VY = 0.9
LP_VZ = 0.9

LP_yaw 	 = 0.9
LP_pitch = 0.9
LP_roll  = 0.9


#Loop rate
Hz 		= 100
dt 		= 1.0/(1.0*Hz)

# Aruco
X_t = 0.0; 
Y_t = 0.0; 
Z_t = 0.0;
X_t1 = 0.0; 
Y_t1 = 0.0; 
Z_t1 = 0.0;

dX_t = 0.1;
dY_t = 0.03;
dZ_t = 0.5; # hieght above the marker to park the vehicle
phi_t 	= 0.0;
theta_t = 0.0;
psi_t 	= 0.0;

LP_aruco = 0.7

LP_aruco1 = 0.7
# autonomy mode or aruco mode
autonomy_mode = True

# initializing aruco_detect_time
aruco_detect_time = 0.0

# kalman predict time
kalman_predict_time = 0.0

# T265_detect time
pose_received_time = 0.0

#file io
fo = open(filename, "a")
print("writing to %s"%(filename))
file_write_ctr = 1

# realsense flag
realsense_connected = False

def Kalman_Filter1(x_k, P_k, u_k, z_k, sigma_P2, sigma_u2, sigma_M2, dt1):

	F_KF = np.array([[1.0, dt1], [0.0, 1.0]])
	F_KF_T = F_KF.T

	Q_KF  = np.array([[1.0/3.0*dt1**3.0, 0.5*dt1**2.0], [0.5*dt1**2.0, 1.0*dt1]])	
	Qu_KF = np.array([[1.0/4.0*dt1**4.0, 0.5*dt1**3.0], [0.5*dt1**3.0, dt1**2.0]])

	x_k[0] = F_KF[0,0] * x_k[0] + F_KF[0,1] * x_k[1] + 0.5*dt1**2 * u_k
	x_k[1] = F_KF[1,0] * x_k[0] + F_KF[1,1] * x_k[1] + dt1 * u_k

	P_k  = np.matmul(np.matmul(F_KF, P_k), F_KF_T)
	P_k1 = P_k + Qu_KF * sigma_u2 + Q_KF * sigma_P2 # sigma_P2 is continuous time white noise intensity

	y_tilda = z_k - x_k[0]
	S_k = P_k1[0,0] + sigma_M2 # sigma_M2 is discrete time gaussian white noise variance

	K_KF = np.zeros(2)
	K_KF[0] = P_k1[0,0]/S_k
	K_KF[1] = P_k1[1,0]/S_k

	x_k[0] = x_k[0] + K_KF[0] * y_tilda
	x_k[1] = x_k[1] + K_KF[1] * y_tilda

	P_k[0][0] = (1.0 - K_KF[0])*P_k1[0][0] + 0.0 * P_k1[1][0]  
	P_k[0][1] = (1.0 - K_KF[0])*P_k1[0][1] + 0.0 * P_k1[1][1] 
	P_k[1][0] = 	 - K_KF[1] *P_k1[0][0] + 1.0 * P_k1[1][0]
	P_k[1][1] = 	 - K_KF[1] *P_k1[0][1] + 1.0 * P_k1[1][1]

	return x_k, P_k

def Kalman_predict(x_k, P_k, u_k, sigma_P2, sigma_u2, dt1):
	F_KF = np.array([[1.0, dt1], [0.0, 1.0]])
	F_KF_T = F_KF.T

	Q_KF  = np.array([[1.0/3.0*dt1**3.0, 0.5*dt1**2.0], [0.5*dt1**2.0, 1.0*dt1]])	
	Qu_KF = np.array([[1.0/4.0*dt1**4.0, 0.5*dt1**3.0], [0.5*dt1**3.0, dt1**2.0]])

	x_k[0] = F_KF[0,0] * x_k[0] + F_KF[0,1] * x_k[1] + 0.5*dt1**2 * u_k
	x_k[1] = F_KF[1,0] * x_k[0] + F_KF[1,1] * x_k[1] + dt1 * u_k

	P_k  = np.matmul(np.matmul(F_KF, P_k), F_KF_T)
	P_k = P_k + Qu_KF * sigma_u2 + Q_KF * sigma_P2 # sigma_P2 is continuous time white noise intensity

	return x_k, P_k

def conn_callback(data):
	global realsense_connected
	if (data.data == 1):
		realsense_connected = True

def pos_vel_callback(data):
	global X, Y, Z, VX, VY, VZ, yaw, pitch
	global pose_received_time

	# 0.155m is the distance of t265 from quad center of mass
	X = (1.0 - LP_X) * X + LP_X * (data.linear.x + 0.155*(1-np.cos(yaw)) + 0.155*(1-np.cos(pitch)));
	Y = (1.0 - LP_Y) * Y + LP_Y * (data.linear.y - 0.155*np.sin(yaw));
	Z = (1.0 - LP_Z) * Z + LP_Z * (data.linear.z + 0.155*np.sin(pitch));

	VX = (1.0 - LP_VX) * VX + LP_VX * data.angular.x;
	VY = (1.0 - LP_VY) * VY + LP_VY * data.angular.y;
	VZ = (1.0 - LP_VZ) * VZ + LP_VZ * data.angular.z;

	pose_received_time = rospy.get_time()

def att_callback(data):
	global yaw, pitch, roll, LP_yaw, LP_pitch, LP_roll
	yaw 	= (1 - LP_yaw) 	 * yaw 	 + LP_yaw 	* data.z
	pitch 	= (1 - LP_pitch) * pitch + LP_pitch * data.y
	roll 	= (1 - LP_roll)  * roll  + LP_roll 	* data.x

aX = 0; aY = 0; aZ = 0;
accel_received_time = 0.0;
std_dev = 0.0
n_std = 0
complete_land = False
descend = False
def accel_callback(data):
	global aX, aY, aZ, accel_received_time
	global complete_land, descend
	# global std_dev, n_std

	aX = (1-0.9) * aX + 0.9 * data.x
	aY = (1-0.9) * aY + 0.9 * data.y
	aZ = (1-0.9) * aZ + 0.9 * data.z

	if(descend == True):
		# print(data.z)
		if(  data.z < -7.0):
			complete_land = True

	# std_dev = np.sqrt((std_dev**2.0 * n_std + aY**2.0)/ (n_std + 1))
	# n_std = n_std + 1
	# print(std_dev)

	accel_received_time = rospy.get_time()

X_t0 = 0
Y_t0 = 0
X_t02 = 0
Y_t02 = 0
VX1 = 0
VY1 = 0
VX2 = 0
VY2 = 0
y_k  = np.array([0.0, 0.0])
Py_k = np.array([[1.0, 0.0],[0.0 ,1.0]])
x_k  = np.array([0.0, 0.0])
Px_k = np.array([[1.0, 0.0],[0.0 ,1.0]])
X_t2 = 0
Y_t2 = 0
Z_t2 = 0
def aruco_callback(data):
	global X_t, Y_t, Z_t, phi_t, theta_t, psi_t
	global X_t1, Y_t1, Z_t1, X_t0, Y_t0, X_t02, Y_t02
	global dX_t, dY_t, dZ_t
	global autonomy_mode, aruco_detect_time, pose_received_time
	global X_d, Y_d, Z_d, yaw_d
	global X, Y, Z, roll, pitch, yaw, VX, VY, VZ
	global LP_aruco, LP_aruco1
	global VX1, VY1, VX2, VY2
	global aX, aY, aZ, accel_received_time

	global fo, filename, file_write_ctr
	global err_sum_y
	global x_k, y_k, Px_k, Py_k, X_t2, Y_t2, Z_t2, kalman_predict_time
	global delay_time

	aruco_detect_time 	= rospy.get_time()
	dt1 = aruco_detect_time - kalman_predict_time
	kalman_predict_time = aruco_detect_time


	#get position
	# X_t = -data.pose.position.y# + 0.125
	# Y_t = data.pose.position.x
	# Z_t = data.pose.position.z
	X_t = (1 - LP_aruco) * X_t + LP_aruco * (-data.pose.position.y)
	Y_t = (1 - LP_aruco) * Y_t + LP_aruco * data.pose.position.x
	Z_t = (1 - LP_aruco) * Z_t + LP_aruco * data.pose.position.z

	# X_d = (1 - 0.5) * X_d + 0.5 * ((X_t - dX_t) + X)
	# X_d = (X_t - dX_t) + X
	# Y_d = (Y_t - dY_t) + Y
	# Z_d = (Z_t - dZ_t) + Z
	# print(Z_d)

	# small angle assumptions in roll and pitch made here,
	# will have to change when testing on larger motion platforms, assumption wont remain true
	# convert from body to inertial frame
	# X_t1 =  X_t * np.cos(yaw) - (Y_t - dY_t) * np.sin(yaw)
	# Y_t1 =  X_t * np.sin(yaw) + (Y_t - dY_t) * np.cos(yaw)
	# Z_t1 = 	Z_t - dZ_t

	body2inert = R.from_euler('ZYX', [yaw, pitch ,roll])
	X_t2, Y_t2, Z_t2 = body2inert.apply([X_t, Y_t, Z_t]) + np.matmul(body2inert.as_dcm() - np.eye(3), [0.14, 0.0, 0.0]) - [0.0, dY_t, 0.0]

	# X1, Y1, Z1 = np.array([X, Y, Z]) + np.array([VX, VY, VZ])*(aruco_detect_time - pose_received_time) \
	# 			 + body2inert.apply([X_t + 0.14, Y_t, Z_t])
	# X2, Y2, Z2 = np.array([X, Y, Z]) + np.array([VX, VY, VZ])*(aruco_detect_time - pose_received_time) + 0.5*np.array([aX, aY, aZ])*(aruco_detect_time - accel_received_time)**2.0\
	# 			 + body2inert.apply([X_t + 0.14, Y_t, Z_t])

	# VX1 = (1-0.25) * VX1 + 0.25 * (X_t1 - X_t0)/dt1
	# VY1 = (1-0.25) * VY1 + 0.25 * (Y_t1 - Y_t0)/dt1
	# X_t0 = X_t1
	# Y_t0 = Y_t1

	# VX2 = (1-0.25) * VX2 + 0.25 * (X_t2 - X_t02)/dt1
	# VY2 = (1-0.25) * VY2 + 0.25 * (Y_t2 - Y_t02)/dt1
	# X_t02 = X_t2
	# Y_t02 = Y_t2

	x_k, Px_k = Kalman_Filter1(x_k, Px_k, -aX, X_t2, 10**(-3.0), 10**(-3.0), 0.1*10**(-4.0), dt1)
	y_k, Py_k = Kalman_Filter1(y_k, Py_k, -aY, Y_t2, 10**(-3.0), 10**(-3.0), 0.1*10**(-4.0), dt1)
	# rospy.loginfo("%f, %f\n", y_k[0], Y_t1)

	#get orientation
	# q0 = data.pose.orientation.w
	# q1 = data.pose.orientation.x
	# q2 = data.pose.orientation.y
	# q3 = data.pose.orientation.z

	# cmra2mrkr = R.from_quat([q1, q2, q3, q0])
	# bdy2cmra  = R.from_euler('ZYX', [np.pi/2.0, 0.0 ,0.0])
	# mrkr2trgt = R.from_euler('ZYX', [np.pi/2.0, 0.0, np.pi]) # mrkr is aruco marker frame, trgt is same frame but with axes parallel to body axes

	# bdy2trgt = bdy2cmra * cmra2mrkr * mrkr2trgt
	# psi, theta, phi = bdy2trgt.as_euler('ZYX')

	# psi_t 	= (1 - LP_aruco1) * psi_t 	+ LP_aruco1 * psi
	# theta_t = (1 - LP_aruco1) * theta_t + LP_aruco1 * theta
	# phi_t 	= (1 - LP_aruco1) * phi_t 	+ LP_aruco1 * phi



	# dyaw_d = 0.01 * (psi_t)# - np.pi/2.0)
	# yaw_d = yaw_d + dyaw_d


	# data = "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n"%(rospy.get_time(), X_t1, Y_t1, Z_t1, X_t2, Y_t2, Z_t2, X1, Y1, Z1, X2, Y2, Z2, VX1, VY1, VX2, VY2, pose_received_time, X, Y, Z, VX, VY, VZ, y_k[0], y_k[1])
	data = "%f,%f,%f,%f,%f,%f,%f\n"%(aruco_detect_time, X_t2, Y_t2, x_k[0], y_k[0], x_k[1], y_k[1])
	
	fo.write(data)
	file_write_ctr = file_write_ctr  + 1
	
	if(file_write_ctr >=500):
		fo.close()
		fo = open(filename, "a")
		file_write_ctr = 1

		
	# print(yaw_d*180.0/np.pi)

	if(autonomy_mode):
		print('entering aruco_mode')
		delay_time = rospy.get_time()
		# err_sum_y = 0
	autonomy_mode = False

def Rdo_callback(data):
	global err_sum_x, err_sum_y, err_sum_z, err_sum_yaw, radio_on,   X_d, Y_d, X, Y
	
	if(data.data == 0):
		err_sum_x = 0.0
		err_sum_y = 0.0
		err_sum_z = 0.0
		err_sum_yaw = 0.0
	# if(radio_on == 0 and data.data == 1):
	# 	X_d = X + 0.5
	# 	Y_d = Y + 0.0
	# 	yawd = yaw
	radio_on = data.data


def constrain(x, a, b):
	if(x<a):
		x = a
	if(x>b):
		x = b
	return x

err_X_prev = 0
err_Y_prev = 0
def autonomy_control():
	global Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, Ki_x, Ki_y, Ki_z
	global Kp_x_t, Kp_y_t, Kp_z_t, Kd_x_t, Kd_y_t, Kd_z_t, Ki_x_t, Ki_y_t, Ki_z_t
	global Kp_yaw, Ki_yaw
	global err_sum_x, err_sum_y, err_sum_z, err_sum_yaw
	global X_d, Y_d, Z_d, yaw_d
	global X, Y, Z, VX, VY, VZ
	global yaw
	global aruco_detect_time, autonomy_mode
	global X_t, Y_t, Z_t, psi_t
	global X_t2, Y_t2, Z_t2
	global dX_t, dY_t, dZ_t

	global err_sum_xint, err_sum_yint, Ki_xint, Ki_yint

	global err_X_prev, err_Y_prev

	global VX2, VY2

	global x_k, Px_k, aX, y_k, Py_k, aY, kalman_predict_time

	global descend

	if (autonomy_mode):
		err_X = X_d - X
		err_Y = Y_d - Y
		err_Z = Z_d - Z
		Z_d_dot = 0.0
	else:
		curr_time = rospy.get_time()
		dt1 	  = curr_time - kalman_predict_time
		kalman_predict_time = curr_time

		x_k, Px_k = Kalman_predict(x_k, Px_k, -aX, 10**(-3.0), 10**(-3.0), dt1)
		y_k, Py_k = Kalman_predict(y_k, Py_k, -aY, 10**(-3.0), 10**(-3.0), dt1)

		err_X = x_k[0]
		err_Y = y_k[0]
		if(descend == False):
			if(x_k[0]**2.0 + y_k[0]**2.0 < 0.05**2.0 and x_k[1]**2.0 + y_k[1]**2.0 < 0.12**2.0):
				descend = True
				err_Z = 0.0
				Z_d_dot = 0.4
			else:
				Z_d = -0.75
				err_Z = Z_d - Z
				Z_d_dot = 0.0
		else:
			err_Z = 0.0
			Z_d_dot = 0.7

		

		# err_Z = Z_t2
	

	# err_Z = Z_d - Z
	err_yaw = yaw_d - yaw

	err_sum_x 	= constrain(err_sum_x + err_X, -1.0/Ki_x, 1.0/Ki_x)
	err_sum_y 	= constrain(err_sum_y + err_Y, -1.0/Ki_y, 1.0/Ki_y)
	err_sum_z 	= constrain(err_sum_z + err_Z, -10.0/Ki_z, 10.0/Ki_z)
	err_sum_yaw = constrain(err_sum_yaw + err_yaw, -10.0/Ki_yaw, 10.0/Ki_yaw)


	# xdd = Kp_x * err_X + 120 * Kd_x * (err_X - err_X_prev) + Ki_x * err_sum_x
	if (autonomy_mode == True or rospy.get_time() - delay_time < 1.0):
		ydd = Kp_y * err_Y + 2*Kd_y * (0.0 - VY) + Ki_y * err_sum_y
		xdd = Kp_x * err_X + 2*Kd_x * (0.0 - VX) + Ki_x * err_sum_x
	else:
		# ydd = 1.0 * err_Y + 80 * 2.5 * (err_Y - err_Y_prev) + 0.0005 * err_sum_y
		# ydd = 3.0 * err_Y + 500 * 2.5 * (err_Y - err_Y_prev) + 0.0001 * err_sum_y + Kd_y * (0.0 - VY) 
		# xdd = Kp_x * err_X + Kd_x * (0.0 - VX) + Ki_x * err_sum_x + 120 * Kd_x * (err_X - err_X_prev)#+ 3 * Kd_x * VX2
		# xdd = Kp_x * err_X + 1.6 * Kd_x * (0.0 - VX) + Ki_x * err_sum_x +  1.5 * Kd_x * VX2
		xdd = Kp_x * err_X + 0.4 * Kd_x * (0.0 - VX) + Ki_x * err_sum_x + 1.8 * Kd_x * x_k[1]
		ydd = Kp_y * err_Y + 0.5 * Kd_y * (0.0 - VY) + Ki_y * err_sum_y + 1.8 * Kd_y * y_k[1]

		# ydd = Kp_y * err_Y + Kd_y * (0.0 - VY) + Ki_y * err_sum_y + 80 * Kd_y * (err_Y - err_Y_prev)
		# print(err_sum_y)
	
	# xdd = Kp_x * err_X + Kd_x * (0.0 - VX) + Ki_x * err_sum_x
	# ydd = Kp_y * err_Y + 2*Kd_y * (0.0 - VY) + Ki_y * err_sum_y
	zdd = Kp_z * err_Z + Kd_z * (Z_d_dot - VZ) + Ki_z * err_sum_z
	# print(err_Z, zdd)

	r_d	= Kp_yaw * err_yaw + Ki_yaw * err_sum_yaw


	err_X_prev = err_X
	err_Y_prev = err_Y

	# print(err_Y)
	

	return xdd, ydd, zdd, r_d

def main():
	global Hz, filename, mass, g, radio_on, pose_received_time

	global x_k_x, x_k_y, x_k_z

	global meas, v2p, yaw_d, psi_t, X_t_F, Y_t_F, Z_t_F, X, VX_t, VY_t, VZ_t

	global autonomy_mode, aruco_detect_time, complete_land, descend

	global Y_t2, y_k, Z, Z_t2

	ctrl 	= Twist();
	states 	= Twist();
	out		= Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/aruco_single/pose', 			   PoseStamped, aruco_callback)	
	rospy.Subscriber('/rs_t265/connected_or_not', 	   UInt8,	 	conn_callback)
	rospy.Subscriber('/rs_t265/position_and_velocity', Twist, 		pos_vel_callback)
	rospy.Subscriber('/rs_t265/attitude', 			   Vector3, 	att_callback)
	rospy.Subscriber('/rs_t265/acceleration', 		   Vector3, 	accel_callback)
	rospy.Subscriber('/serialcom/radio', 			   UInt8, 		Rdo_callback)

	pub  = rospy.Publisher('/neo/control', Twist, queue_size=1)
	pub1 = rospy.Publisher('/neo/states',  Twist, queue_size=1)
	pub2 = rospy.Publisher('/neo/out',     Twist, queue_size=1)

	
	
	

	print('starting control')
	rate = rospy.Rate(Hz)#100 Hz
	while not rospy.is_shutdown():
		try:

			# print(rospy.get_time() - aruco_detect_time)
			if(autonomy_mode == False and rospy.get_time() - aruco_detect_time >= 0.5 and descend == False):
				autonomy_mode = True
				print("leaving aruco")
				Z_d = -0.75
				# err_sum_y = 0
		
			# print(rospy.get_time())
			xdd, ydd, zdd, r_d = autonomy_control()


			T_d 	= mass * (g - zdd)
			# inertial to body frame
			theta_d = -xdd/g*np.cos(yaw) - ydd/g*np.sin(yaw)
			phi_d 	= (ydd/g*np.cos(yaw) - xdd/g*np.sin(yaw))*np.cos(theta_d)

			# theta_d = -xdd/g
			# phi_d 	= ydd/g


			# no integral without takeoff
			if(Z > -0.005 or realsense_connected == False):
				err_sum_z = 0.0
				err_sum_y = 0.0
				err_sum_x = 0.0
				err_sum_yaw = 0.0
				T_d 		= 0.0
				phi_d	  	= 0.0*np.pi/180.0
				theta_d	  	= 0.0*np.pi/180.0
				r_d 		= 0.0


			if(complete_land == True):
				T_d 			= 0.0
				phi_d	  		= 0.0*np.pi/180.0
				theta_d	  		= 0.0*np.pi/180.0
				r_d 			= 0.0

			
			# # publish to a topic
			# if(rospy.get_time() - pose_received_time < 1.0/30.0):
			# 	ctrl.linear.x 	= radio_on
			# else:
			# 	ctrl.linear.x 	= 0
			# 	print('odom data received at less than 30Hz, giving control back to pilot')
			# 	print(1.0/(rospy.get_time() - pose_received_time ))

			ctrl.linear.x    = radio_on
			ctrl.linear.y 	= 0.0
			ctrl.linear.z 	= constrain(T_d * Thrust_sf, 0.0, 2.6)
			ctrl.angular.x 	= constrain(phi_d * 180.0/np.pi, -10.0, 10.0) 
			ctrl.angular.y 	= constrain(theta_d * 180.0/np.pi, -10.0, 10.0)
			ctrl.angular.z 	= constrain(r_d * 180.0/np.pi, -30, 30)
			pub.publish(ctrl);


			# states.linear.x 	= x_k_x[0]
			# states.linear.y 	= VX_t
			# states.linear.z 	= x_k_z[0]
			# states.angular.x 	= x_k_x[1]
			# states.angular.y 	= x_k_y[1]
			# states.angular.z 	= x_k_z[1]
			# pub1.publish(states);


			# out.linear.x 	= Y_t2#roll*180.0/np.pi#meas
			# out.linear.y 	= y_k[0]#x_k_z[0]
			# out.linear.z 	= 0.0#v2p
			# out.angular.x 	= 0.0#yaw_d * 180.0/np.pi
			# out.angular.y 	= 0.0#psi_t * 180.0/np.pi
			# out.angular.z 	= 0.0#0.0
			# pub2.publish(out);



			

		except Exception:
			traceback.print_exc()
			indent = 1
		
		rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass