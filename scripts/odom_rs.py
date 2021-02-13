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
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
from std_msgs.msg import UInt8

from datetime import datetime
now = datetime.now() # current date and time
filename = "/home/su/Dropbox/Quadrotor_flight_control/Arduino_playground/LOGS/odom_logs/"
filename += now.strftime("%Y_%m_%d_%H_%M_%S")
filename += ".csv"


mass = 1.46; # mass of quad
g = 9.81;# acceleration due to gravity
Thrust_sf = 2.1/(mass*g);

radio_on = 0


Kp_x = 4.0#2.5
Kd_x = 3.0#0.001
Ki_x = 0.0035

Kp_y = 4.0#2.5
Kd_y = 3.0#0.001
Ki_y = 0.0035

Kp_z = 4.0#2.0
Kd_z = 1.8#0.9
Ki_z = 0.007

Kp_yaw = 1.0


err_sum_x = 0.0
err_sum_y = 0.0
err_sum_z = 0.0




#
X_d = 0.0;
Y_d = 0.0;
Z_d = -1.0;
yaw_d = 0.0;


#
X = 0.0
Y = 0.0
Z = 0.0

VX = 0.0
VY = 0.0
VZ = 0.0

yaw = 0.0


#
LP_X = 0.5
LP_Y = 0.5
LP_Z = 0.5

LP_VX = 0.5
LP_VY = 0.5
LP_VZ = 0.5

LP_yaw = 0.5


#
phi_d_ac = 0.0
theta_d_ac = 0.0

Hz 		= 100
dt 		= 1.0/(1.0*Hz)

def pos_vel_callback(data):
	global X, Y, Z, VX, VY, VZ, yaw, theta_d_ac

	X = (1 - LP_X) * X + LP_X * (data.linear.x - 0.155*(1-np.cos(yaw)) -0.155*(1-np.cos(theta_d_ac)));
	Y = (1 - LP_Y) * Y + LP_Y * (data.linear.y - 0.155*np.sin(yaw));
	Z = (1 - LP_Z) * Z + LP_Z * (data.linear.z- 0.155*np.sin(theta_d_ac));

	VX = (1 - LP_VX) * VX + LP_VX * data.angular.x;
	VY = (1 - LP_VY) * VY + LP_VY * data.angular.y;
	VZ = (1 - LP_VZ) * VZ + LP_VZ * data.angular.z;

def yaw_callback(data):
	global yaw
	yaw = (1 - LP_yaw) * yaw + LP_yaw * data.data

# def Alt_vel_callback(data):
# 	global Z_d, u_d, v_d
	
# 	u_d 	= data.x
# 	v_d 	= data.y
# 	Z_d 	= -data.z

def Rdo_callback(data):
	global err_sum_x, err_sum_y, err_sum_z, radio_on,   X_d, Y_d, X, Y
	
	if(data.data == 0):
		err_sum_x = 0.0
		err_sum_y = 0.0
		err_sum_z = 0.0
	if(radio_on == 0 and data.data == 1):
		X_d = X
		Y_d = Y
	radio_on = data.data


def constrain(x, a, b):
	if(x<a):
		x = a
	if(x>b):
		x = b
	return x

def main():
	global Hz, filename, mass, g, Kp_yaw, Kp_x, Kp_y, Kp_z, Ki_x, Ki_y, Ki_z, Kd_z, err_sum_x, err_sum_y, err_sum_z, radio_on, \
	 X, Y, Z, VX, VY, VZ, X_d, Y_d, Z_d, yaw_d, yaw, phi_d_ac, theta_d_ac

	ctrl = Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/rs_t265/position_and_velocity', Twist, pos_vel_callback)
	rospy.Subscriber('/rs_t265/yaw', Float64, yaw_callback)
	# rospy.Subscriber('/serialcom/alt_vel_des', Vector3, Alt_vel_callback)
	rospy.Subscriber('/serialcom/radio', UInt8, Rdo_callback)

	pub = rospy.Publisher('/neo/odometry', Twist, queue_size=1)

	rate = rospy.Rate(Hz)#100 Hz
	fo = open(filename, "a")
	print("writing to %s"%(filename))
	file_write_ctr = 1
	while not rospy.is_shutdown():
		try:

			xdd = Kp_x * (X_d - X) + Kd_x * (0.0 - VX) + Ki_x * err_sum_x
			ydd = Kp_y * (Y_d - Y) + Kd_y * (0.0 - VY) + Ki_y * err_sum_y
			zdd = Kp_z * (Z_d - Z) + Kd_z * (0.0 - VZ) + Ki_z * err_sum_z
			r_d	= Kp_yaw * (yaw_d - yaw)

			T_d 	= mass * (g - zdd)
			phi_d 	= ydd/g + 0.0*np.pi/180.0
			theta_d = -xdd/g + 0.7*np.pi/180.0

			theta_d_ac = -(xdd - Ki_x*err_sum_x)/g
			phi_d_ac = (ydd - Ki_y*err_sum_y)/g				
			

			err_sum_x = constrain(err_sum_x + (X_d - X), -0.2/Ki_x, 0.2/Ki_x)
			err_sum_y = constrain(err_sum_y + (Y_d - Y), -1.0/Ki_y, 1.0/Ki_y)
			err_sum_z = constrain(err_sum_z + (Z_d - Z), -10.0/Ki_z, 10.0/Ki_z)

			# no integral without takeoff
			if(Z > -0.25):
				err_sum_z = 0.0
				err_sum_y = 0.0
				err_sum_x = 0.0
				phi_d	  = 0.0*np.pi/180.0
				theta_d	  = 0.7*np.pi/180.0

			
			# publish to a topic
			ctrl.linear.x 	= radio_on
			ctrl.linear.y 	= 0.0
			ctrl.linear.z 	= constrain(T_d * Thrust_sf, 0.0, 3.0)
			ctrl.angular.x 	= constrain(phi_d * 180.0/np.pi, -15.0, 15.0) 
			ctrl.angular.y 	= constrain(theta_d * 180.0/np.pi, -15.0, 15.0)
			ctrl.angular.z 	= constrain(r_d * 180.0/np.pi, -60, 60)
			pub.publish(ctrl);


			data = "%f, ,%f,%f,%f,%f,%f, ,%f,%f,%f,%f,%f, ,%f,%f,%f,%f,%f\n"%(rospy.get_time(), X_d, X, VX, \
			 		Ki_x * err_sum_x, ctrl.angular.y, Y_d, Y, VY, Ki_y * err_sum_y, ctrl.angular.x, Z_d,\
			  		Z, VZ, Ki_z * err_sum_z, ctrl.linear.z)

			fo.write(data)
			file_write_ctr = file_write_ctr  + 1
			
			if(file_write_ctr >=500):
				fo.close()
				fo = open(filename, "a")
				file_write_ctr = 1

		except Exception:
			traceback.print_exc()
			indent = 1
		
		rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass