import traceback
#Lidar import
import Lidar_Lite_v4
# ROS imports
import rospy
from std_msgs.msg import Float64

def main():
	L1 = Lidar_Lite_v4.LiDAR_Lite_v4()
	L1.connect(1)
	L1.set_high_acc(0x06)

	rospy.init_node('lidar', anonymous=True)
	pub = rospy.Publisher('/lidarlite/distance', Float64, queue_size=1)

	while not rospy.is_shutdown():
		try:
			pub.publish(L1.get_distance());
		except Exception:
			traceback.print_exc()
			#rospy.loginfo('Some error ocurred in px4.py')
			indent = 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass