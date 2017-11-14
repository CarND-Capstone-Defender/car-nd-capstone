#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        
        # TODO: Add other member variables you need below
        #current position & base waypoint msgs variables
        self.current_pos = None
        self.base_way = None
        
        #rate is same as DBW node rospy rate
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            self.do_update()
            rate.sleep()
        
    def pose_cb(self, msg):
        #save msg of current position to it's variable
        self.current_pos = msg

    def waypoints_cb(self, waypoints):
        #save msg of base waypoints to it's variable
        self.base_way = waypoints
            
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
               
    #This function is used to select number of base waypoints to be added to the final waypoint msg to be published
    #Points are the nearest LOOKAHEAD_WPS points to the current position
    def do_update(self):
        if(self.base_way != None) and (self.current_pos != None):
            self.final_way = Lane()
            closest_idx = self.check_closet_point(self.base_way.waypoints, self.current_pos)
            base_way_len = len(self.base_way.waypoints)
            for i in range(closest_idx, (closest_idx + LOOKAHEAD_WPS)):
                idx = i % base_way_len
                self.final_way.waypoints.append(self.base_way.waypoints[idx])
            
            self.final_waypoints_pub.publish(self.final_way)
            

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
