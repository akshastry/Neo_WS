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
filename += ".csv"


mass = 1.46; # mass of quad
g = 9.81;# acceleration due to gravity
Thrust_sf = 2.1/(mass*g);

radio_on = 0


Kp_x = 4.0#2.5
Kd_x = 5.5#0.001
Ki_x = 0.001

Kp_y = 4.0#2.5
Kd_y = 5.5#0.001
Ki_y = 0.001

Kp_z = 5.0#2.0
Kd_z = 5.0#0.9
Ki_z = 0.01

Kp_yaw = 7.0
Ki_yaw = 0.01


err_sum_x = 0.0
err_sum_y = 0.0
err_sum_z = 0.0
err_sum_yaw = 0.0

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
Z_d = -1.5;
yaw_d 	= 0.0;

Xdot_d = 0.0
Ydot_d = 0.0
Zdot_d = 0.0


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
LP_X = 0.55
LP_Y = 0.35
LP_Z = 0.55

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
phi_t 	= 0.0;
theta_t = 0.0;
psi_t 	= 0.0;

# Kalman
dt_KF 	= 1.0/30.0

F_KF = np.array([[1.0, dt_KF], [0.0, 1.0]])
F_KF_T = F_KF.T

Q_KF = np.array([[0.25*dt_KF**4, 0.5*dt_KF**3], [0.5*dt_KF**3, dt_KF**2]])

x_k_x = np.zeros(2); P_k_x = (10.0**0.0)*np.ones((2,2)); sigma_M2_x = 25*10**(-6.0); sigma_P2_x =  30.0*10**(-6.0);
x_k_y = np.zeros(2); P_k_y = (10.0**0.0)*np.ones((2,2)); sigma_M2_y = 25*10**(-6.0); sigma_P2_y =  40.0*10**(-6.0);
x_k_z = np.zeros(2); P_k_z = (10.0**0.0)*np.ones((2,2)); sigma_M2_z = 25*10**(-6.0); sigma_P2_z = 30.0*10**(-6.0);
x_k_z[0] = 1.5
# temporary
v2p  = 0.0
meas = 0.0

# autonomy mode or aruco mode
autonomy_mode = True

# initializing aruco_detect_time
aruco_detect_time = 0.0

#temp maybe
X_t_F = 0.0
Y_t_F = 0.0
Z_t_F = 0.0

X_t_LP = 0.0
Y_t_LP = 0.0
Z_t_LP = 0.0

VX_t = 0.0
VY_t = 0.0
VZ_t = 0.0

def pos_vel_callback(data):
	global X, Y, Z, VX, VY, VZ, yaw, pitch

	# 0.155m is the distance of t265 from quad center of mass
	X = (1 - LP_X) * X + LP_X * (data.linear.x + 0.155*(1-np.cos(yaw)) + 0.155*(1-np.cos(pitch)));
	Y = (1 - LP_Y) * Y + LP_Y * (data.linear.y - 0.155*np.sin(yaw));
	Z = (1 - LP_Z) * Z + LP_Z * (data.linear.z + 0.155*np.sin(pitch));

	VX = (1 - LP_VX) * VX + LP_VX * data.angular.x;
	VY = (1 - LP_VY) * VY + LP_VY * data.angular.y;
	VZ = (1 - LP_VZ) * VZ + LP_VZ * data.angular.z;

def att_callback(data):
	global yaw, pitch, roll, LP_yaw, LP_pitch, LP_roll
	yaw 	= (1 - LP_yaw) 	 * yaw 	 + LP_yaw 	* data.z
	pitch 	= (1 - LP_pitch) * pitch + LP_pitch * data.y
	roll 	= (1 - LP_roll)  * roll  + LP_roll 	* data.x

def Kalman_Filter(x_k, P_k, u_k, z_k, sigma_P2, sigma_M2):
    global F_KF, F_KF_T, Q_KF, dt_KF

    x_k[0] = F_KF[0,0] * x_k[0] + F_KF[0,1] * x_k[1] + 0.5*dt_KF**2 * u_k
    x_k[1] = F_KF[1,0] * x_k[0] + F_KF[1,1] * x_k[1] + dt_KF * u_k

    P_k  = np.matmul(np.matmul(F_KF, P_k), F_KF_T)
    P_k1 = P_k + Q_KF * sigma_P2

    y_tilda = z_k - x_k[0]
    S_k = P_k1[0,0] + sigma_M2

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

def aruco_callback(data):
	global X_t, Y_t, Z_t, phi_t, theta_t, psi_t,    X_t_F, Y_t_F, Z_t_F
	global x_k_x, x_k_y, x_k_z, P_k_x, P_k_y, P_k_z
	global sigma_P2_x, sigma_P2_y, sigma_P2_z, sigma_M2_x, sigma_M2_y, sigma_M2_z
	global v2p
	global autonomy_mode, aruco_detect_time
	global X, Y, Z, roll, pitch, yaw
	global yaw, yaw_d
	global X_d, Y_d, Z_d
	global Xdot_d, Ydot_d, Zdot_d

	global VX_t, VY_t, VZ_t

	global meas

	dt = rospy.get_time() - aruco_detect_time
	aruco_detect_time = rospy.get_time()
	
	if(dt <= 1/15 and aruco_detect_time != 0 ):
		VX_t = (-data.pose.position.y + 0.125 - X_t) / dt 
		VY_t = (data.pose.position.x 		  - Y_t) / dt
		VZ_t = (data.pose.position.z 		  - Z_t) / dt


	#get position
	X_t = -data.pose.position.y + 0.125
	Y_t = data.pose.position.x
	Z_t = data.pose.position.z





	# in2bdy = R.from_euler('ZYX', [yaw, pitch, roll])
	# X_t, Y_t, Z_t = in2bdy.apply([X_t, Y_t, Z_t])




	# X_t_F = (1 - 1) * X_t_F + 1 * (X_t + X)
	# Y_t_F = (1 - 1) * Y_t_F + 1 * (Y_t + Y)
	# Z_t_F = (1 - 1) * Z_t_F + 1 * (Z_t + Z)

	#get orientation
	q0 = data.pose.orientation.w
	q1 = data.pose.orientation.x
	q2 = data.pose.orientation.y
	q3 = data.pose.orientation.z

	cmra2mrkr = R.from_quat([q1, q2, q3, q0])
	bdy2cmra  = R.from_euler('ZYX', [np.pi/2.0, 0.0 ,0.0])
	mrkr2trgt = R.from_euler('ZYX', [np.pi/2.0, 0.0, np.pi]) # mrkr is aruco marker frame, trgt is same frame but with axes parallel to body axes

	bdy2trgt = bdy2cmra * cmra2mrkr * mrkr2trgt
	psi_t, theta_t, phi_t = bdy2trgt.as_euler('ZYX')

	#filter position
	x_k_x, P_k_x = Kalman_Filter(x_k_x, P_k_x, 0.0, X_t, sigma_P2_x, sigma_M2_x)
	x_k_y, P_k_y = Kalman_Filter(x_k_y, P_k_y, 0.0, Y_t, sigma_P2_y, sigma_M2_y)
	x_k_z, P_k_z = Kalman_Filter(x_k_z, P_k_z, 0.0, Z_t, sigma_P2_z, sigma_M2_z)
	

	meas = Z_t + Z
	v2p = v2p + x_k_z[1] * dt_KF

	# rospy.loginfo('%.3f, %.3f, %.3f, %.3f, %.3f, %.3f', X, Y, Z, phi*180.0/np.pi, theta*180.0/np.pi, psi*180.0/np.pi)

	# autonomy_mode = False
	# yaw_d 	= (1 - 0.6) * yaw_d + 0.6 * (yaw + psi_t)
	
	# X_d 	= x_k_x[0]
	# Y_d   	= x_k_y[0]

	# Xdot_d 	= x_k_x[1] #(1 - 0.1) * Xdot_d + 0.1 * (x_k_x[1]) 
	# Ydot_d 	= x_k_y[1] #(1 - 0.1) * Ydot_d + 0.1 * (x_k_y[1])
	
	


def Alt_vel_callback(data):
	global Z_d, X_d, Y_d
	
	Z_d 	= -data.z

	if(autonomy_mode):
		X_d 	= data.x
		Y_d 	= data.y	

def yawd_callback(data):
	global yaw_d, autonomy_mode

	if(autonomy_mode):
		yaw_d 	= data.data

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
	global Kp_yaw, Ki_yaw
	global err_sum_x, err_sum_y, err_sum_z, err_sum_yaw
	global X_d, Y_d, Z_d, yaw_d
	global X, Y, Z, VX, VY, VZ
	global yaw
	global Xdot_d, Ydot_d, Zdot_d
	global aruco_detect_time

	if(rospy.get_time() - aruco_detect_time > 1/15.0):
		Xdot_d = 0.0
		Ydot_d = 0.0
		Zdot_d = 0.0

	if (autonomy_mode):
		xdd = Kp_x * (X_d - X) + Kd_x * (Xdot_d - VX) + Ki_x * err_sum_x
		ydd = Kp_y * (Y_d - Y) + Kd_y * (Ydot_d - VY) + Ki_y * err_sum_y
	else:
		xdd = Kp_x_t * (X_d - X) + Kd_x_t * (Xdot_d - VX) + Ki_x_t * err_sum_x
		ydd = Kp_y_t * (Y_d - Y) + Kd_y_t * (Ydot_d - VY) + Ki_y_t * err_sum_y

	zdd = Kp_z * (Z_d - Z) + Kd_z * (Zdot_d - VZ) + Ki_z * err_sum_z

	# xd = Kp_x/Kd_x * (X_d - X)
	# yd = Kp_y/Kd_y * (Y_d - Y)
	# zd = Kp_z/Kd_z * (Z_d - Z)
	# xdd = Kd_x * (xd - VX) + Ki_x * err_sum_x
	# ydd = Kd_y * (yd - VY) + Ki_y * err_sum_y
	# zdd = Kd_z * (zd - VZ) + Ki_z * err_sum_z

	r_d	= Kp_yaw * (yaw_d - yaw) + Ki_yaw * err_sum_yaw 

	err_sum_x = constrain(err_sum_x + (Xdot_d - VX), -1.0/Ki_x, 1.0/Ki_x)
	err_sum_y = constrain(err_sum_y + (Ydot_d - VY), -1.0/Ki_y, 1.0/Ki_y)
	err_sum_z = constrain(err_sum_z + (Zdot_d - VZ), -10.0/Ki_z, 10.0/Ki_z)
	err_sum_yaw = constrain(err_sum_yaw + (yaw_d - yaw), -10.0/Ki_yaw, 10.0/Ki_yaw)

	return xdd, ydd, zdd, r_d


def main():
	global Hz, filename, mass, g, radio_on

	global x_k_x, x_k_y, x_k_z

	global meas, v2p, yaw_d, psi_t, X_t_F, Y_t_F, Z_t_F, X, VX_t, VY_t, VZ_t

	ctrl 	= Twist();
	states 	= Twist();
	out		= Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/aruco_single/pose', 			   PoseStamped, aruco_callback)	
	rospy.Subscriber('/rs_t265/position_and_velocity', Twist, 		pos_vel_callback)
	rospy.Subscriber('/rs_t265/attitude', 			   Vector3, 	att_callback)
	rospy.Subscriber('/serialcom/alt_vel_des', 		   Vector3, 	Alt_vel_callback)
	rospy.Subscriber('/serialcom/yaw_des', 			   Float64, 	yawd_callback)
	rospy.Subscriber('/serialcom/radio', 			   UInt8, 		Rdo_callback)

	pub  = rospy.Publisher('/neo/control', Twist, queue_size=1)
	pub1 = rospy.Publisher('/neo/states',  Twist, queue_size=1)
	pub2 = rospy.Publisher('/neo/out',     Twist, queue_size=1)

	rate = rospy.Rate(Hz)#100 Hz
	# fo = open(filename, "a")
	# print("writing to %s"%(filename))
	# file_write_ctr = 1
	while not rospy.is_shutdown():
		try:
		
			xdd, ydd, zdd, r_d = autonomy_control()


			T_d 	= mass * (g - zdd)
			theta_d = -xdd/g*np.cos(yaw) - ydd/g*np.sin(yaw)
			phi_d 	= (ydd/g*np.cos(yaw) - xdd/g*np.sin(yaw))*np.cos(theta_d)		


			# no integral without takeoff
			if(Z > -0.1):
				err_sum_z = 0.0
				err_sum_y = 0.0
				err_sum_x = 0.0
				err_sum_yaw = 0.0
				phi_d	  = 0.0*np.pi/180.0
				theta_d	  = 0.0*np.pi/180.0

			
			# publish to a topic
			ctrl.linear.x 	= radio_on
			ctrl.linear.y 	= 0.0
			ctrl.linear.z 	= constrain(T_d * Thrust_sf, 0.0, 3.0)
			ctrl.angular.x 	= constrain(phi_d * 180.0/np.pi, -25.0, 25.0) 
			ctrl.angular.y 	= constrain(theta_d * 180.0/np.pi, -25.0, 25.0)
			ctrl.angular.z 	= constrain(r_d * 180.0/np.pi, -60, 60)
			pub.publish(ctrl);


			states.linear.x 	= x_k_x[0]
			states.linear.y 	= VX_t
			states.linear.z 	= x_k_z[0]
			states.angular.x 	= x_k_x[1]
			states.angular.y 	= x_k_y[1]
			states.angular.z 	= x_k_z[1]
			pub1.publish(states);


			out.linear.x 	= X#roll*180.0/np.pi#meas
			out.linear.y 	= pitch*180.0/np.pi#x_k_z[0]
			out.linear.z 	= yaw*180.0/np.pi#v2p
			out.angular.x 	= 0.0#yaw_d * 180.0/np.pi
			out.angular.y 	= 0.0#psi_t * 180.0/np.pi
			out.angular.z 	= 0.0#0.0
			pub2.publish(out);



			# data = "%f, ,%f,%f,%f,%f,%f, ,%f,%f,%f,%f,%f, ,%f,%f,%f,%f,%f\n"%(rospy.get_time(), X_d, X, VX, \
			#  		Ki_x * err_sum_x, ctrl.angular.y, Y_d, Y, VY, Ki_y * err_sum_y, ctrl.angular.x, Z_d,\
			#   		Z, VZ, Ki_z * err_sum_z, ctrl.linear.z)

			# fo.write(data)
			# file_write_ctr = file_write_ctr  + 1
			
			# if(file_write_ctr >=500):
			# 	fo.close()
			# 	fo = open(filename, "a")
			# 	file_write_ctr = 1

		except Exception:
			traceback.print_exc()
			indent = 1
		
		rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass