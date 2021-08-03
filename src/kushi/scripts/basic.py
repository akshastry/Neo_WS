#!/usr/bin/env python

#python imports
import time
import math

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
			print("The list of words is : " + str(res))

			result = str(res).find('hover')
			print("Substring 'hover' found at index:", result)

			result2 = str(res).find('stay')
			print("Substring 'stay' found at index:", result2)

			# for word in res:
			if result != -1:
			    print("Ready to hover")
			    # call hover function here
			    cmd.data = 1
			    pub.publish(cmd)
			    
			elif result2 != -1:
			    print("Ready to stay")
			    
			    # call stay function here?
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