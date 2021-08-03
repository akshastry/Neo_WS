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
X_d = 0.0;
Y_d = 0.0;
Z_d = 1.0;
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

# state machine flags
hover_flag = False
land_flag = True
complete_land = False
realsense_connected = False

#file io
fo = open(filename, "a")
# print("writing to %s"%(filename))
file_write_ctr = 1

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

def accel_callback(data):
	global hover_flag
	if(np.sqrt(data.z**2.0+data.y**2.0+data.x**2.0) > 15.0):
		hover_flag = False

def att_callback(data):
	global yaw, pitch, roll, LP_yaw, LP_pitch, LP_roll
	yaw 	= (1 - LP_yaw) 	 * yaw 	 + LP_yaw 	* data.z
	pitch 	= (1 - LP_pitch) * pitch + LP_pitch * data.y
	roll 	= (1 - LP_roll)  * roll  + LP_roll 	* data.x


def Rdo_callback(data):
	global err_sum_x, err_sum_y, err_sum_z, err_sum_yaw, radio_on,   X_d, Y_d, X, Y
	
	if(data.data == 0):
		err_sum_x = 0.0
		err_sum_y = 0.0
		err_sum_z = 0.0
		err_sum_yaw = 0.0
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

	xd = (Kp_x/Kd_x - 0.0) * (X_d - X)
	yd = (Kp_y/Kd_y - 0.0) * (Y_d - Y)
	zd = (Kp_z/Kd_z - 0.0) * (Z_d - Z)
	r_d	= Kp_yaw * (yaw_d - yaw) + Ki_yaw * err_sum_yaw

	xd = constrain(xd, -0.4, 0.4)
	yd = constrain(yd, -0.4, 0.4)
	zd = constrain(zd, -0.3, 0.3)

	xdd = Kd_x * (xd - VX) + Ki_x * err_sum_x
	ydd = Kd_y * (yd - VY) + Ki_y * err_sum_y
	zdd = Kd_z * (zd - VZ) + Ki_z * err_sum_z

	err_sum_x 	= constrain(err_sum_x + (xd - VX), -1.0/Ki_x, 1.0/Ki_x)
	err_sum_y 	= constrain(err_sum_y + (yd - VY), -1.0/Ki_y, 1.0/Ki_y)
	err_sum_z 	= constrain(err_sum_z + (zd - VZ), -10.0/Ki_z, 10.0/Ki_z)
	err_sum_yaw = constrain(err_sum_yaw + (yaw_d - yaw), -10.0/Ki_yaw, 10.0/Ki_yaw)

	return xdd, ydd, zdd, r_d

def cmd_callback(data):
	global err_sum_y, err_sum_z, err_sum_x, err_sum_yaw, theta_d, phi_d
	global X_d, Y_d, Z_d
	global land_flag, hover_flag
	print(data.data)
	if(data.data == 1):
		print("Hover called")
		Z_d = -1.0
		# no integral without takeoff
		err_sum_z = 0.0
		err_sum_y = 0.0
		err_sum_x = 0.0
		err_sum_yaw = 0.0
		phi_d	  = 0.0*np.pi/180.0
		theta_d	  = 0.0*np.pi/180.0
		r_d 	  = 0.0
		hover_flag = True
	elif(data.data == 2):
		X_d = X_d + 1.0
	elif(data.data == 3):
		Z_d = 0.1
		land_flag = True
	else:
		print("command not recognized")

def main():
	global Hz, filename, mass, g, radio_on, pose_received_time

	global x_k_x, x_k_y, x_k_z

	global meas, v2p, yaw_d, psi_t, X_t_F, Y_t_F, Z_t_F, X, VX_t, VY_t, VZ_t

	global hover_flag, land_flag, complete_land, realsense_connected, Z_d, Z

	ctrl 	= Twist();
	states 	= Twist();
	out		= Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/rs_t265/connected_or_not', 	   UInt8,	 	conn_callback)
	rospy.Subscriber('/rs_t265/position_and_velocity', Twist, 		pos_vel_callback)
	rospy.Subscriber('/rs_t265/accelration', 		   Twist, 		accel_callback)
	rospy.Subscriber('/rs_t265/attitude', 			   Vector3, 	att_callback)
	rospy.Subscriber('/serialcom/radio', 			   UInt8, 		Rdo_callback)
	rospy.Subscriber('/gcs/cmd', 					   UInt8, 		cmd_callback)

	pub  = rospy.Publisher('/neo/control', Twist, queue_size=1)
	pub1 = rospy.Publisher('/neo/states',  Twist, queue_size=1)
	pub2 = rospy.Publisher('/neo/out',     Twist, queue_size=1)

	
	
	T_d 	= 0.0
	theta_d = 0.0
	phi_d 	= 0.0
	r_d 	= 0.0

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
			if(hover_flag == False or realsense_connected ==  False):
				err_sum_z 	= 0.0
				err_sum_y 	= 0.0
				err_sum_x 	= 0.0
				err_sum_yaw = 0.0
				T_d 		= 0.0
				phi_d	  	= 0.0*np.pi/180.0
				theta_d	  	= 0.0*np.pi/180.0
				r_d 		= 0.0
				

			# if(land_flag == True and Z <= 0.1):
			# 	complete_land = True

			# if(complete_land == True):
			# 	T_d = 0.0

			
			# # publish to a topic
			# if(rospy.get_time() - pose_received_time < 1.0/30.0):
			# 	ctrl.linear.x 	= radio_on
			# else:
			# 	ctrl.linear.x 	= 0
			# 	print('odom data received at less than 30Hz, giving control back to pilot')
			# 	print(1.0/(rospy.get_time() - pose_received_time ))

			ctrl.linear.x   = radio_on
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