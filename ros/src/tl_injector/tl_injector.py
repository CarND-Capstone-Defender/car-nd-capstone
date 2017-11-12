#!/usr/bin/env python
import rospy
from styx_msgs.msg import TrafficLightDetection
from geometry_msgs.msg import PoseStamped, Pose


import sys, select, termios, tty

moveBindings = {
    'r': (4),
    'y': (2),
    'g': (1),
	'x': (-1),
}

settings = termios.tcgetattr(sys.stdin)

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key



class TLInjector(object):


	def __init__(self):

		rospy.init_node('tl_injector', log_level=rospy.DEBUG)
		
		self.pose = None

		sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		
		self.upcoming_traffic_light_injection = rospy.Publisher(
			'/traffic_waypoint_injection', TrafficLightDetection, queue_size=1)

		rate = rospy.Rate(10)  # try removing this line ans see what happens
		while not rospy.is_shutdown():
			key = getKey()

			if key in moveBindings.keys():
				state = moveBindings[key]

				if state == -1:
					break

				holla = TrafficLightDetection()
				holla.waypoint = 753
				holla.state = state
				
				if self.pose:
					print str(self.pose.pose.position.x) + "|" + str(self.pose.pose.position.y) + "|" + str(self.pose.pose.position.z) + "|"
				
				self.upcoming_traffic_light_injection.publish(holla)

	def pose_cb(self, msg):
		self.pose = msg

if __name__ == '__main__':
    try:
        TLInjector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic injector node.')
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
