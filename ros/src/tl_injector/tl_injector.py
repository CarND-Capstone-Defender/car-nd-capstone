#!/usr/bin/env python
import rospy
from styx_msgs.msg import TrafficLightDetection
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane

import math
import sys, traceback

######################################
# how to run the TL injector ?
# - use the Docker environment
# - attach a new console to a running Docker instance
# - run the injector as separate ROS node
# - type 'r' or 'y' or 'g'
# - subscribe to topic /traffic_waypoint_injection

# Example:
# ./attachToContainer.sh 
# ./runTL_injector.sh
# r

# type 'r' in order to inject a red traffic light
# type 'y' in order to inject a yellow traffic light
# type 'g' in order to inject a green traffic light
# type 'x' in order to stop the TL injector
######################################

import sys, select, termios, tty

moveBindings = {
    'r': (0),
    'y': (1),
    'g': (2),
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
		self.waypoints = None

		sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		self.upcoming_traffic_light_injection = rospy.Publisher(
			'/traffic_waypoint_injection', TrafficLightDetection, queue_size=1)

    # keyboard logger implemented according to https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
		rate = rospy.Rate(10)  # try removing this line ans see what happens
		while not rospy.is_shutdown():
			key = getKey()

			if key in moveBindings.keys():
				state = moveBindings[key]

				if state == -1:
					print 'INFO: key x pressed - TLInjector is exiting...'
					sys.exit(0)

  			if self.pose is not None:
					injectorMsg = TrafficLightDetection()
					injectorMsg.waypoint = self.get_closest_waypoint(self.pose.pose)
					injectorMsg.state = state
					
					if self.pose:
						if state == 0:
							color = 'RED'
						elif state == 1:
							color = 'YELLOW'
						elif state == 2:
							color = 'GREEN'
						else: 
							color = 'UNKNOWN'
							injectorMsg.state = 3
						
						#debugMsg = 'Injecting ' + color + ' traffic light at (x,y) = (' + str(self.pose.pose.position.x) + ',' + str(self.pose.pose.position.y) + ')'
						debugMsg = 'Injecting ' + color + ' traffic light at waypoint = ' + str(injectorMsg.waypoint)
						print debugMsg
					
					self.upcoming_traffic_light_injection.publish(injectorMsg)

	def pose_cb(self, msg):
		self.pose = msg

	def waypoints_cb(self, lane):
			self.waypoints = lane.waypoints

	def get_closest_waypoint(self, pose):
			"""Identifies the closest path waypoint to the given position
					https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
			Args:
					pose (Pose): position to match a waypoint to

			Returns:
					int: index of the closest waypoint in self.waypoints

			"""
			if self.waypoints is None:
					return
			
			minDist = 99999
			minIndex = None

			posx = pose.position.x
			posy = pose.position.y

			for i, waypoint in enumerate(self.waypoints):
					wp_x = waypoint.pose.pose.position.x
					wp_y = waypoint.pose.pose.position.y
					distance = math.sqrt(pow(posx - wp_x,2) + pow(posy - wp_y,2))

					#if we find a closer distance 
					if (distance < minDist): 
							minIndex = i          # use this as next closest waypojnt
							minDist = distance  # use this distance as new closest distance

			# returns the index of the found waypoint
			return minIndex

if __name__ == '__main__':
    try:
        TLInjector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic injector node.')
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
