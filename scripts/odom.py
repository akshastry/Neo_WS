#!/usr/bin/env python

#python imports
import time
import math
import traceback
import numpy as np
from scipy.spatial.transform import Rotation as R
from threading import Lock
#PX4flow import
from px_comm.msg import OpticalFlow
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

#mutex
x_mutex = Lock()
y_mutex = Lock()
z_mutex = Lock()

mass = 1.46; # mass of quad
g = 9.81;# acceleration due to gravity
Thrust_sf = 2.1/(mass*g);

Z_d = -1.0; 
u_d = 0.0; 
v_d = 0.0;

radio_on = 0


Kp_x = 3.5#2.5
Kd_x = 0.0#0.001
Ki_x = 0.0035

Kp_y = 3.9#2.5
Kd_y = 0.0#0.001
Ki_y = 0.0035

Kp_z = 4.0#2.0
Kd_z = 1.8#0.9
Ki_z = 0.007

err_sum_x = 0.0
err_sum_y = 0.0
err_sum_z = 0.0



#focal length of optic flow camera
focal_length = (16.0/24.0) *1000.0 # (focal length in mm/ pixel size in micrometer) *1000 = focal length in pixels

#distance of the flow sensor and lidar from body center of mass
r_flow_sensor = np.array([0,-0.021,0.2])
Lidar_offset_from_flow = -0.8/100.0

#KF matrices
x_k_z = np.zeros(2)
P_k_z = np.zeros((2,2))
sigma_P2_z = 10**(0.0)
sigma_M2_z = 1*10**(-4)

x_k_x = np.zeros(2)
P_k_x = np.zeros((2,2))
sigma_P2_x = 10.0*10**(-3.0)
sigma_M2_x = 1*10**(-4)

x_k_y = np.zeros(2)
P_k_y = np.zeros((2,2))
sigma_P2_y = 10.0*10**(-3.0)
sigma_M2_y = 1*10**(-4)


LP_lidar = 0.3
LP_xdot = 0.1
LP_ydot = 0.1

ground_distance = 0.06
xdot = 0.0
ydot = 0.0

phi_d_ac = 0.0
theta_d_ac = 0.0

Hz 		= 100
dt 		= 1.0/(1.0*Hz)

F_KF = np.array([[1.0, dt], [0.0, 1.0]])
Q_KF = np.array([[0.25*dt**4, 0.5*dt**3], [0.5*dt**3, dt**2]])

def x_KF_predict(u_k):
	x_mutex.acquire()

	global x_k_x, P_k_x, F_KF, Q_KF, sigma_P2_x, dt

	x_k_x = np.transpose(np.matmul(F_KF, np.reshape(x_k_x, (-1, 1))))[0]
	x_k_x[0] = x_k_x[0] + 0.5*dt**2 * u_k
	x_k_x[1] = x_k_x[1] + dt * u_k

	P_k1 = np.matmul(F_KF, P_k_x)
	P_k = np.matmul(P_k1, F_KF.T)
	P_k_x = P_k + Q_KF * sigma_P2_x

	print(np.trace(P_k_x))
	x_mutex.release()

def y_KF_predict(u_k):
	y_mutex.acquire()

	global x_k_y, P_k_y, F_KF, Q_KF, sigma_P2_y, dt

	x_k_y = np.transpose(np.matmul(F_KF, np.reshape(x_k_y, (-1, 1))))[0]
	x_k_y[0] = x_k_y[0] + 0.5*dt**2 * u_k
	x_k_y[1] = x_k_y[1] + dt * u_k

	P_k1 = np.matmul(F_KF, P_k_y)
	P_k = np.matmul(P_k1, F_KF.T)
	P_k_y = P_k + Q_KF * sigma_P2_y

	y_mutex.release()

def z_KF_predict(u_k):
	z_mutex.acquire()

	global x_k_z, P_k_z, F_KF, Q_KF, sigma_P2_z, dt

	x_k_z = np.transpose(np.matmul(F_KF, np.reshape(x_k_z, (-1, 1))))[0]
	x_k_z[0] = x_k_z[0] + 0.5*dt**2 * u_k
	x_k_z[1] = x_k_z[1] + dt * u_k

	P_k1 = np.matmul(F_KF, P_k_z)
	P_k = np.matmul(P_k1, F_KF.T)
	P_k_z = P_k + Q_KF * sigma_P2_z

	# print(np.trace(P_k_z))
	z_mutex.release()

def x_KF_update(z_k):
	x_mutex.acquire()

	global x_k_x, P_k_x, sigma_M2_x

	y_tilda = z_k - x_k_x[1]
	S_k = P_k_x[1,1] + sigma_M2_x

	K_KF = np.zeros(2)
	K_KF[0] = P_k_x[0,1]/S_k
	K_KF[1] = P_k_x[1,1]/S_k

	x_k_x[0] = x_k_x[0] + K_KF[0] * y_tilda
	x_k_x[1] = x_k_x[1] + K_KF[1] * y_tilda

	P_k1 = np.zeros((2,2))
	P_k1[0][0] = (1.0 - K_KF[0])*P_k_x[0][0] + 0.0 * P_k_x[1][0]  
	P_k1[0][1] = (1.0 - K_KF[0])*P_k_x[0][1] + 0.0 * P_k_x[1][1] 
	P_k1[1][0] = -K_KF[1]*P_k_x[0][0] + 1.0 * P_k_x[1][0]
	P_k1[1][1] = -K_KF[1]*P_k_x[0][1] + 1.0 * P_k_x[1][1]

	P_k_x = P_k1

	# print(np.trace(P_k_x))
	x_mutex.release()


def y_KF_update(z_k):
	y_mutex.acquire()

	global x_k_y, P_k_y, sigma_M2_y

	y_tilda = z_k - x_k_y[1]
	S_k = P_k_y[1,1] + sigma_M2_y

	K_KF = np.zeros(2)
	K_KF[0] = P_k_y[0,1]/S_k
	K_KF[1] = P_k_y[1,1]/S_k

	x_k_y[0] = x_k_y[0] + K_KF[0] * y_tilda
	x_k_y[1] = x_k_y[1] + K_KF[1] * y_tilda

	P_k1 = np.zeros((2,2))
	P_k1[0][0] = (1.0 - K_KF[0])*P_k_y[0][0] + 0.0 * P_k_y[1][0]  
	P_k1[0][1] = (1.0 - K_KF[0])*P_k_y[0][1] + 0.0 * P_k_y[1][1] 
	P_k1[1][0] = -K_KF[1]*P_k_y[0][0] + 1.0 * P_k_y[1][0]
	P_k1[1][1] = -K_KF[1]*P_k_y[0][1] + 1.0 * P_k_y[1][1]

	P_k_y = P_k1

	y_mutex.release()


def z_KF_update(z_k):
	z_mutex.acquire()

	global x_k_z, P_k_z, sigma_M2_z

	y_tilda = z_k - x_k_z[0]
	S_k = P_k_z[0,0] + sigma_M2_z

	K_KF = np.zeros(2)
	K_KF[0] = P_k_z[0,0]/S_k
	K_KF[1] = P_k_z[1,0]/S_k

	x_k_z[0] = x_k_z[0] + K_KF[0] * y_tilda
	x_k_z[1] = x_k_z[1] + K_KF[1] * y_tilda

	P_k1 = np.zeros((2,2))
	P_k1[0][0] = (1.0 - K_KF[0])*P_k_z[0][0] + 0.0 * P_k_z[1][0]  
	P_k1[0][1] = (1.0 - K_KF[0])*P_k_z[0][1] + 0.0 * P_k_z[1][1] 
	P_k1[1][0] = -K_KF[1]*P_k_z[0][0] + 1.0 * P_k_z[1][0]
	P_k1[1][1] = -K_KF[1]*P_k_z[0][1] + 1.0 * P_k_z[1][1]

	P_k_z = P_k1

	z_mutex.release()
	


def px4_callback(data):
	global flow_x, flow_y, ground_distance, x_k_z, theta_d_ac, phi_d_ac, LP_ydot, LP_xdot, xdot, ydot
	
	if(math.isnan(data.ground_distance) == False and math.isnan(data.flow_x) == False and math.isnan(data.flow_y) == False):
		if(data.ground_distance > 0.0 and data.quality >= 150):
			ground_distance_sonar = data.ground_distance
			flow_x = data.velocity_x / ground_distance_sonar 
			flow_y = data.velocity_y / ground_distance_sonar

			x_vel = flow_x * ground_distance
			y_vel = flow_y * ground_distance

			w = (x_k_z[1] - (np.sin(theta_d_ac)*x_vel - np.cos(theta_d_ac)*np.sin(phi_d_ac)*y_vel))/(np.cos(phi_d_ac) * np.cos(theta_d_ac))
			xdot1 = np.cos(theta_d_ac)*x_vel + np.sin(phi_d_ac)*np.sin(theta_d_ac)*y_vel - np.cos(phi_d_ac)*np.sin(theta_d_ac)*w
			ydot1 = np.cos(phi_d_ac)*y_vel + np.sin(phi_d_ac)*w

			# xdot1 = x_vel
			# ydot1 = y_vel

			xdot = LP_xdot * xdot1 + (1.0-LP_xdot) * xdot
			ydot = LP_ydot * ydot1 + (1.0-LP_ydot) * ydot

			x_KF_update(xdot)
			y_KF_update(ydot)

def lidar_callback(data):
	global ground_distance, LP_lidar, Lidar_offset_from_flow, r_flow_sensor, theta_d_ac, phi_d_ac

	if(math.isnan(data.data) == False and data.data > 0.0):
		ground_distance = LP_lidar * ((data.data - 5.0)/100.0 + Lidar_offset_from_flow) + (1.0-LP_lidar) * ground_distance
		z_k = -(ground_distance + r_flow_sensor[2])/(np.cos(phi_d_ac) * np.cos(theta_d_ac))

		z_KF_update(z_k)

def Alt_vel_callback(data):
	global Z_d, u_d, v_d, main_running
	
	if (~main_running):
		u_d 	= data.x
		v_d 	= data.y
		Z_d 	= -data.z

def Rdo_callback(data):
	global err_sum_x, err_sum_y, err_sum_z, radio_on
	
	if(data.data == 0):
		err_sum_x = 0.0
		err_sum_y = 0.0
		err_sum_z = 0.0
	# if(radio_on == 0 and data.data == 1):
	# 	Z_d = X_k[2]
	radio_on = data.data


def constrain(x, a, b):
	if(x<a):
		x = a
	if(x>b):
		x = b
	return x

def main():
	global Hz, filename, mass, g, Kp_x, Kp_y, Kp_z, Ki_x, Ki_y, Ki_z, Kd_z, err_sum_x, err_sum_y, err_sum_z, radio_on, \
	 x_k_z, x_k_y, x_k_x, Z_d, u_d, v_d, phi_d_ac, theta_d_ac

	ctrl = Twist();
	states = Twist();

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/px4flow/opt_flow', OpticalFlow, px4_callback)
	rospy.Subscriber('/lidarlite/distance', Float64, lidar_callback)
	# rospy.Subscriber('/serialcom/attitude_and_rate', Twist, Fcon_callback)
	rospy.Subscriber('/serialcom/alt_vel_des', Vector3, Alt_vel_callback)
	rospy.Subscriber('/serialcom/radio', UInt8, Rdo_callback)
	end_time = time.time()
	pub = rospy.Publisher('/neo/odometry', Twist, queue_size=1)
	pub1 = rospy.Publisher('/neo/states', Twist, queue_size=1)
	# pub2 = rospy.Publisher('/neo/debug', Float64, queue_size=1)

	rate = rospy.Rate(Hz)#100 Hz
	fo = open(filename, "a")
	print("writing to %s"%(filename))
	file_write_ctr = 1
	while not rospy.is_shutdown():
		try:

			zdd = Kp_z * (Z_d - x_k_z[0]) + Kd_z * (0.0 - x_k_z[1]) + Ki_z * err_sum_z
			ydd = Kp_y * (v_d - x_k_y[1]) + Ki_y * err_sum_y
			xdd = Kp_x * (u_d - x_k_x[1]) + Ki_x * err_sum_x


			T_d 	= mass * (g - zdd)
			phi_d 	= ydd/g + 0.0*np.pi/180.0
			theta_d = -xdd/g + 0.7*np.pi/180.0
			psi_d	= 0.0

			#no flow below a certain altitude so dont predict
			if(x_k_z[0] < -0.25):
				x_KF_predict(xdd - Ki_x*err_sum_x)
				y_KF_predict(ydd - Ki_y*err_sum_y)

			z_KF_predict(zdd - Ki_z*err_sum_z)

			theta_d_ac = -(xdd - Ki_x*err_sum_x)/g
			phi_d_ac = (ydd - Ki_y*err_sum_y)/g				
			

			err_sum_x = constrain(err_sum_x + (u_d - x_k_x[1]), -0.2/Ki_x, 0.2/Ki_x)
			err_sum_y = constrain(err_sum_y + (v_d - x_k_y[1]), -1.0/Ki_y, 1.0/Ki_y)
			err_sum_z = constrain(err_sum_z + (Z_d - x_k_z[0]), -10.0/Ki_z, 10.0/Ki_z)

			# no integral without takeoff
			if(x_k_z[0] > -0.25):
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
			ctrl.angular.z 	= psi_d * 180.0/np.pi
			pub.publish(ctrl);

			states.linear.x 	= x_k_x[0]
			states.linear.y 	= x_k_y[0]
			states.linear.z 	= x_k_z[0]
			states.angular.x 	= x_k_x[1]
			states.angular.y 	= x_k_y[1]
			states.angular.z 	= x_k_z[1]
			pub1.publish(states);

			data = "%f, ,%f,%f,%f,%f,%f, ,%f,%f,%f,%f,%f, ,%f,%f,%f,%f,%f\n"%(rospy.get_time(), u_d, x_k_x[0], x_k_x[1], \
			 		Ki_x * err_sum_x, ctrl.angular.y, v_d, x_k_y[0], x_k_y[1], Ki_y * err_sum_y, ctrl.angular.x, Z_d,\
			  		x_k_z[0], x_k_z[1], Ki_z * err_sum_z, ctrl.linear.z)

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