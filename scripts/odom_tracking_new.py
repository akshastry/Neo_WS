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
filename += "speed=3_0_old_cm.csv"


mass = 1.46; # mass of quad
g = 9.81;# acceleration due to gravity
Thrust_sf = 2.3/(mass*g);

radio_on = 0


Kp_x = 4.0#2.5
Kd_x = 5.5#0.001
Ki_x = 0.001

Kp_y = 4.0#2.5
Kd_y = 5.5#0.001
Ki_y = 0.001

Kp_z = 7.0#2.0
Kd_z = 5.0#0.9
Ki_z = 0.01

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
X_d = 0.1;
Y_d = 0.6;
Z_d = -1.0;
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

LP_VX = 0.7
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
dX_t = 0.1;
dY_t = 0.02;
dZ_t = 0.7; # hieght above the marker to park the vehicle
phi_t 	= 0.0;
theta_t = 0.0;
psi_t 	= 0.0;

LP_aruco = 0.7

LP_aruco1 = 0.5
# autonomy mode or aruco mode
autonomy_mode = True

# initializing aruco_detect_time
aruco_detect_time = 0.0

# T265_detect time
pose_received_time = 0.0

#file io
fo = open(filename, "a")
print("writing to %s"%(filename))
file_write_ctr = 1

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

def aruco_callback(data):
	global X_t, Y_t, Z_t, phi_t, theta_t, psi_t
	global dX_t, dY_t, dZ_t
	global autonomy_mode, aruco_detect_time
	global X_d, Y_d, Z_d, yaw_d
	global X, Y, Z, yaw, VX, VY, VZ
	global LP_aruco, LP_aruco1

	global fo, filename, file_write_ctr

	dt1 = rospy.get_time() - aruco_detect_time
	aruco_detect_time = rospy.get_time()


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

	X_t1 =  X_t * np.cos(yaw) - (Y_t - dY_t) * np.sin(yaw)
	Y_t1 =  X_t * np.sin(yaw) + (Y_t - dY_t) * np.cos(yaw)

	dX_d = 0.1 * X_t1 + 0.05 * (0.0 - VX)
	dY_d = 0.1 * Y_t1 + 0.05 * (0.0 - VY)
	dZ_d = 0.1 * (Z_t - dZ_t) + 0.05 * (0.0 - VZ)  


	X_d = X_d + dX_d 
	Y_d = Y_d + dY_d
	Z_d = Z_d + dZ_d

	#get orientation
	q0 = data.pose.orientation.w
	q1 = data.pose.orientation.x
	q2 = data.pose.orientation.y
	q3 = data.pose.orientation.z

	cmra2mrkr = R.from_quat([q1, q2, q3, q0])
	bdy2cmra  = R.from_euler('ZYX', [np.pi/2.0, 0.0 ,0.0])
	mrkr2trgt = R.from_euler('ZYX', [np.pi/2.0, 0.0, np.pi]) # mrkr is aruco marker frame, trgt is same frame but with axes parallel to body axes

	bdy2trgt = bdy2cmra * cmra2mrkr * mrkr2trgt
	psi, theta, phi = bdy2trgt.as_euler('ZYX')

	psi_t 	= (1 - LP_aruco1) * psi_t 	+ LP_aruco1 * psi
	theta_t = (1 - LP_aruco1) * theta_t + LP_aruco1 * theta
	phi_t 	= (1 - LP_aruco1) * phi_t 	+ LP_aruco1 * phi



	dyaw_d = 0.01 * (psi_t)# - np.pi/2.0)
	yaw_d = yaw_d + dyaw_d


	data = "%f,%f,%f,%f,%f,%f,%f\n"%(rospy.get_time(), X_t, Y_t, Z_t, phi_t, theta_t, psi_t)

	fo.write(data)
	file_write_ctr = file_write_ctr  + 1
	
	if(file_write_ctr >=500):
		fo.close()
		fo = open(filename, "a")
		file_write_ctr = 1

		
	# print(yaw_d*180.0/np.pi)

	# autonomy_mode = False
	# print('entering aruco_mode')

# def Alt_vel_callback(data):
# 	global Z_d, X_d, Y_d
	
# 	Z_d 	= -data.z

# 	if(autonomy_mode):
# 		X_d 	= data.x
# 		Y_d 	= data.y	

# def yawd_callback(data):
# 	global yaw_d, autonomy_mode

# 	if(autonomy_mode):
# 		yaw_d 	= data.data

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
	global dX_t, dY_t, dZ_t

	global err_sum_xint, err_sum_yint, Ki_xint, Ki_yint
	# print(Z)
	# print(rospy.get_time() - aruco_detect_time)
	# if(rospy.get_time() - aruco_detect_time > 1.0/10.0 and autonomy_mode == False):
	# 	# print('entering autonomy_mode')
	# 	# X_d = X_t + X
	# 	# Y_d = Y_t + Y
	# 	autonomy_mode = True

	# if (autonomy_mode):
	xd = (Kp_x/Kd_x - 0.0) * (X_d - X)
	yd = (Kp_y/Kd_y - 0.0) * (Y_d - Y)
	zd = (Kp_z/Kd_z - 0.0) * (Z_d - Z)
	r_d	= Kp_yaw * (yaw_d - yaw) + Ki_yaw * err_sum_yaw

	xd = constrain(xd, -0.5, 0.5)
	yd = constrain(yd, -0.5, 0.5)
	zd = constrain(zd, -0.5, 0.5)

	xdd = Kd_x * (xd - VX) + Ki_x * err_sum_x
	ydd = Kd_y * (yd - VY) + Ki_y * err_sum_y
	zdd = Kd_z * (zd - VZ) + Ki_z * err_sum_z

	err_sum_x 	= constrain(err_sum_x + (xd - VX), -1.0/Ki_x, 1.0/Ki_x)
	err_sum_y 	= constrain(err_sum_y + (yd - VY), -1.0/Ki_y, 1.0/Ki_y)
	err_sum_z 	= constrain(err_sum_z + (zd - VZ), -10.0/Ki_z, 10.0/Ki_z)
	err_sum_yaw = constrain(err_sum_yaw + (yaw_d - yaw), -10.0/Ki_yaw, 10.0/Ki_yaw)
	# else:
	# 	print('not in autonomy_mode')
		# xd = (Kp_x_t/Kd_x_t - 0.0) * (X_t - dX_t)
		# yd = (Kp_y_t/Kd_y_t - 0.0) * (Y_t - dY_t)
		# zd = (Kp_z_t/Kd_z_t - 0.0) * (Z_t - dZ_t)
		# r_d	= Kp_yaw * (yaw_d - yaw) + Ki_yaw * err_sum_yaw

		# xd = constrain(xd, -0.5, 0.5)
		# yd = constrain(yd, -0.5, 0.5)
		# zd = constrain(zd, -0.5, 0.5)

		# xdd = (Kd_x_t + 0.0) * (xd - VX) + Ki_x_t * err_sum_x
		# ydd = (Kd_y_t + 0.0)* (yd - VY) + Ki_y_t * err_sum_y
		# zdd = Kd_z_t * (zd - VZ) + Ki_z_t * err_sum_z

		# err_sum_x 	= constrain(err_sum_x + (xd - VX), -1.0/Ki_x_t, 1.0/Ki_x_t)
		# err_sum_y 	= constrain(err_sum_y + (yd - VY), -1.0/Ki_y_t, 1.0/Ki_y_t)
		# err_sum_z 	= constrain(err_sum_z + (zd - VZ), -10.0/Ki_z_t, 10.0/Ki_z_t)	
		# err_sum_yaw = constrain(err_sum_yaw + (yaw_d - yaw), -10.0/Ki_yaw, 10.0/Ki_yaw)
		
		# err_sum_xint 	= constrain(err_sum_xint + (X_t - dX_t), -2.0/Ki_xint, 2.0/Ki_xint)
		# err_sum_yint 	= constrain(err_sum_yint + (Y_t - dY_t), -2.0/Ki_yint, 2.0/Ki_yint)
		# if(X_t**2.0 + Y_t **2.0 < 0.03**2.0):
			# print('Should Land')

	return xdd, ydd, zdd, r_d

def main():
	global Hz, filename, mass, g, radio_on, pose_received_time

	global x_k_x, x_k_y, x_k_z

	global meas, v2p, yaw_d, psi_t, X_t_F, Y_t_F, Z_t_F, X, VX_t, VY_t, VZ_t

	ctrl 	= Twist();
	states 	= Twist();
	out		= Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/aruco_single/pose', 			   PoseStamped, aruco_callback)	
	rospy.Subscriber('/rs_t265/position_and_velocity', Twist, 		pos_vel_callback)
	rospy.Subscriber('/rs_t265/attitude', 			   Vector3, 	att_callback)
	# rospy.Subscriber('/serialcom/alt_vel_des', 		   Vector3, 	Alt_vel_callback)
	# rospy.Subscriber('/serialcom/yaw_des', 			   Float64, 	yawd_callback)
	rospy.Subscriber('/serialcom/radio', 			   UInt8, 		Rdo_callback)

	pub  = rospy.Publisher('/neo/control', Twist, queue_size=1)
	pub1 = rospy.Publisher('/neo/states',  Twist, queue_size=1)
	pub2 = rospy.Publisher('/neo/out',     Twist, queue_size=1)

	
	
	

	print('starting control')
	rate = rospy.Rate(Hz)#100 Hz
	while not rospy.is_shutdown():
		try:
		
			# print(rospy.get_time())
			xdd, ydd, zdd, r_d = autonomy_control()


			T_d 	= mass * (g - zdd)
			theta_d = -xdd/g*np.cos(yaw) - ydd/g*np.sin(yaw)
			phi_d 	= (ydd/g*np.cos(yaw) - xdd/g*np.sin(yaw))*np.cos(theta_d)		


			# no integral without takeoff
			if(Z > -0.005):
				err_sum_z = 0.0
				err_sum_y = 0.0
				err_sum_x = 0.0
				err_sum_yaw = 0.0
				phi_d	  = 0.0*np.pi/180.0
				theta_d	  = 0.0*np.pi/180.0

			
			# # publish to a topic
			# if(rospy.get_time() - pose_received_time < 1.0/30.0):
			# 	ctrl.linear.x 	= radio_on
			# else:
			# 	ctrl.linear.x 	= 0
			# 	print('odom data received at less than 30Hz, giving control back to pilot')
			# 	print(1.0/(rospy.get_time() - pose_received_time ))

			ctrl.linear.x    = radio_on
			ctrl.linear.y 	= 0.0
			ctrl.linear.z 	= constrain(T_d * Thrust_sf, 0.0, 2.75)
			ctrl.angular.x 	= constrain(phi_d * 180.0/np.pi, -25.0, 25.0) 
			ctrl.angular.y 	= constrain(theta_d * 180.0/np.pi, -25.0, 25.0)
			ctrl.angular.z 	= constrain(r_d * 180.0/np.pi, -30, 30)
			pub.publish(ctrl);


			# states.linear.x 	= x_k_x[0]
			# states.linear.y 	= VX_t
			# states.linear.z 	= x_k_z[0]
			# states.angular.x 	= x_k_x[1]
			# states.angular.y 	= x_k_y[1]
			# states.angular.z 	= x_k_z[1]
			# pub1.publish(states);


			# out.linear.x 	= X#roll*180.0/np.pi#meas
			# out.linear.y 	= pitch*180.0/np.pi#x_k_z[0]
			# out.linear.z 	= yaw*180.0/np.pi#v2p
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