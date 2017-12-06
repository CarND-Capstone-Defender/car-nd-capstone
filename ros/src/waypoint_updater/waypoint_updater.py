#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightDetection , TrafficLight


import math
import copy

MAX_DECEL = 4.0
MIN_DISTANCE_TO_LIGHT  = 100



'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
	def __init__(self):
		rospy.init_node('waypoint_updater')

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

		# TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

		rospy.Subscriber('/traffic_waypoint', TrafficLightDetection, self.traffic_cb)

		# rospy.Subscriber('/traffic_waypoint_injection', TrafficLightDetection, self.traffic_cb)


		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

		# TODO: Add other member variables you need below
		#current position & base waypoint msgs variables
		self.current_pos = None
		self.base_way = None
		self.tl_position = None
		self.distance_to_light = 0

		self.next_tl_state = TrafficLight.UNKNOWN

		self.tl_detector_running = False


		#rate is same as DBW node rospy rate
		rate = rospy.Rate(50)

		while not rospy.is_shutdown():
			self.do_update()
			rate.sleep()

	def pose_cb(self, msg):
		#save msg of current position to it's variable
		self.current_pos = msg

		# Every time we get a pose message, calculate distance to light.
		if self.tl_position != None:
			tl = self.tl_position
			c = self.current_pos.pose.position
			self.distance_to_light = math.sqrt(pow(tl.x - c.x,2) + pow(tl.y - c.y,2))

	def waypoints_cb(self, waypoints):
		#save msg of base waypoints to it's variable
		self.base_way = waypoints

	def traffic_cb(self, msg):
		# TODO: Callback for /traffic_waypoint message. Implement
		self.tl_detector_running = True

		self.next_tl_state = msg.state
		#rospy.logwarn("Setting Traffic State to %s",self.next_tl_state)


		self.tl_position = self.base_way.waypoints[msg.waypoint].pose.pose.position

		tl = self.tl_position
		c = self.current_pos.pose.position

		self.distance_to_light = math.sqrt(pow(tl.x - c.x,2) + pow(tl.y - c.y,2))
		self.stopping = False


		#rospy.logwarn("Current Pose: Point x:%d y:%d z:%d",c.x,c.y,c.z)
		#rospy.logwarn("Traffic Msg Rx:Light: %d Way Point id: %d",msg.state, msg.waypoint)
		#rospy.logwarn("Next TL Point: Point x:%d y:%d z:%d : Distance to Light:%d",tl.x,tl.y,tl.z,self.distance_to_light)




	def obstacle_cb(self, msg):
		# TODO: Callback for /obstacle_waypoint message. We will implement it later
		pass

	def get_waypoint_velocity(self, waypoint):
		return waypoint.twist.twist.linear.x

	def set_waypoint_velocity(self, waypoints, waypoint, velocity):
		waypoints[waypoint].twist.twist.linear.x = velocity

	def distance(self, waypoints, wp1, wp2):
		dist = 0
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
		for i in range(wp1, wp2+1):
			dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
			wp1 = i
		return dist

	def distance_between_pose(self, p1, p2):
		x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
		return math.sqrt(x*x + y*y + z*z)

	#This function is used to loop over all the points and find the nearest one to the current point and return it's index
	def check_closet_point(self, points, curr_point):
		min_delta = 10000000
		min_idx = 0
		for i in range(len(points)):
			p = points[i].pose.pose.position
			c = curr_point.pose.position
			delta = math.sqrt((p.x - c.x)**2 + (p.y - c.y)**2  + (p.z - c.z)**2)
			if(delta < min_delta):
				min_delta = delta
				min_idx = i
		return min_idx

	def decelerate(self, waypoints):
		last = waypoints[-1]
		last.twist.twist.linear.x = 0.
		for wp in waypoints[:-1][::-1]:
			#dist = self.distance_between_pose(wp.pose.pose.position, last.pose.pose.position)
			#vel = math.sqrt(2 * MAX_DECEL * dist)
			#if vel < 1.:
			# vel = 0.
			#	wp.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)

			# Set velocity to zero, from this point onwards, pid will manage deceleration.
			wp.twist.twist.linear.x = 0.


		return waypoints


	#This function is used to select number of base waypoints to be added to the final waypoint msg to be published
	#Points are the nearest LOOKAHEAD_WPS points to the current position
	def do_update(self):
		if self.tl_detector_running == False:
			return

		if(self.base_way != None) and (self.current_pos != None):

			if ((self.next_tl_state == TrafficLight.YELLOW) or (self.next_tl_state == TrafficLight.RED)):
				if (self.distance_to_light <= MIN_DISTANCE_TO_LIGHT):
					self.stopping = True
					#rospy.logwarn("Decelerate: Distance to light: %d", self.distance_to_light)

				else:
					self.stopping = False
			else:
				self.stopping = False

			self.final_way = Lane()
			closest_idx = self.check_closet_point(self.base_way.waypoints, self.current_pos)
			closest_idx = closest_idx + 1  #Skip forward.
			base_way_len = len(self.base_way.waypoints)
			
			if (self.stopping == True):
				host_wp = copy.deepcopy(self.base_way.waypoints[closest_idx])
				v = host_wp.twist.twist.linear.x
				a = -150*v*v/(2*self.distance_to_light)

			for i in range(closest_idx, (closest_idx + LOOKAHEAD_WPS)):
				idx = i % base_way_len
				wp = copy.deepcopy(self.base_way.waypoints[idx])
				if (self.stopping == True):
					wp.twist.twist.linear.x = v + a * 0.02
					v = wp.twist.twist.linear.x 


				self.final_way.waypoints.append(wp)

			# List of base way points - ahead, they are loaded from base waypints, so
			# are all set to the target velocity. The PID controller will manage accel.
			# We only have to modify the speeds for cases where the light is red/yellow.
			#if ((self.next_tl_state == 0) or (self.next_tl_state == 1)):
			#	if (self.distance_to_light <= MIN_DISTANCE_TO_LIGHT):
			#		self.final_way.waypoints = self.decelerate(self.final_way.waypoints)
			#		rospy.logwarn("Decelerate: Distance to light: %d", self.distance_to_light)

			self.final_waypoints_pub.publish(self.final_way)


if __name__ == '__main__':
	try:
		WaypointUpdater()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
