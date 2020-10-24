#!/usr/bin/env python

import traceback
import numpy as np
import Lidar_Lite
from px_comm.msg import OpticalFlow
import rospy



# from 


main_running = False

flow_x = 0.0
flow_y = 0.0
ground_distance_sonar = 0.0

#focal length of optic flow camera
focal_length = (16.0/24.0) *1000.0 # (focal length in mm/ pixel size in micrometer) *1000 = focal length in pixels

def px4_callback(data):
	global flow_x, flow_y, ground_distance_sonar, main_running
	if (~main_running):
		ground_distance_sonar = data.ground_distance
		flow_x = data.velocity_x / ground_distance_sonar 
		flow_y = data.velocity_y / ground_distance_sonar


def main():
	global flow_x, flow_y, ground_distance_sonar, main_running
	L1 = Lidar_Lite.LiDAR_Lite()
	L1.connect(1)
	b = 2.0/(18.0+1.0)
	dist = 1;

	rospy.init_node('odom', anonymous=True)

	rospy.Subscriber('/px4flow/opt_flow', OpticalFlow, px4_callback)

	rate = rospy.Rate(100)#100 Hz
	while not rospy.is_shutdown():
		main_running = True
		try:
			# get data
			# filter
			# derive control
			# publish to a topic
			print(ground_distance_sonar*100.0)
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