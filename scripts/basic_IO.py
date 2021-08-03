#!/usr/bin/env python

#python imports
import time
import math
import traceback

#ROS imports
import rospy
from std_msgs.msg import UInt8




def main():

	rospy.init_node('gcs', anonymous=True)

	cmd = UInt8()
	cmd.data = 0
	pub = rospy.Publisher('/gcs/cmd', UInt8, queue_size=1)
	


	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		try:

			# initializing string
			phrase = raw_input("enter command:")

			res = phrase.split()

			# printing result
			# print("The list of words is : " + str(res))

			result  = str(res).find('hover')
			result1 = str(res).find('forward')
			result2 = str(res).find('land')
			
			

			# for word in res:
			if result != -1:
			    # print("Substring 'hover' found at index:", result)
			    print("Ready to hover")
			    # call hover function here
			    cmd.data = 1
			    time.sleep(3)
			    pub.publish(cmd)
			    # time.sleep(1)
			    # pub.publish(cmd)
			    # time.sleep(1)
			    # pub.publish(cmd)

			elif result != -1:
			    print("Moving forward")
			    cmd.data = 2
			    time.sleep(3)
			    pub.publish(cmd)
			    
			elif result2 != -1:
			    # print("Substring 'land' found at index:", result2)
			    print("Landing")
			    cmd.data = 3
			    time.sleep(3)
			    pub.publish(cmd)
			    # pub.publish(cmd)
			    # pub.publish(cmd)

			else:
			    print("Command not recognized")						



		except Exception:
			traceback.print_exc()
			indent = 1
	
		rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass