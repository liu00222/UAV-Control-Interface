#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

pose = PoseStamped()

def callback(data):
	global pose
	pose = data
	#print("I am here")


def listener():
	
	rospy.init_node('listener', anonymous=True)
	
	rospy.Subscriber('/firefly/ground_truth/pose', PoseStamped, callback)
	while True:
		print(pose.pose.position)
	
	rospy.spin()
	

if __name__ == '__main__':
	listener()
