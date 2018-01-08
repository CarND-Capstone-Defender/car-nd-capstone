#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightDetection , TrafficLight
from geometry_msgs.msg import TwistStamped
import tf
import yaml

import math
import copy

MIN_DISTANCE_TO_LIGHT  = 140
LOOKAHEAD_WPS = 25 # Number of waypoints we will publish. You can change this number
MIN_STOPPING_SPEED = 1.0
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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.initialized = False

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity',TwistStamped, self.current_vel_callback)

        rospy.Subscriber('/traffic_waypoint', TrafficLightDetection, self.traffic_cb)
        # rospy.Subscriber('/traffic_waypoint_injection', TrafficLightDetection, self.traffic_cb)

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600. # m/s
        self.deceleration_limit = rospy.get_param('/dbw_node/decel_limit')
        rospy.logdebug("Speed limit = %s" , self.speed_limit)

        self.mode = self.getMode()

        if self.mode == "SIM":
            self.waypoints_increasing = True
            self.STOPLINE_BRAKING_BUFFER = 1.0 # TODO: this has to be tuned eventually..
            self.SMOOTH_APPROACH_TO_STOPLINE_FACTOR = 0.25 # this is just a heuristic factor.....feel free to play around
        else:
            self.waypoints_increasing = True #TODO: Antonia: verify whether this is really the case!
            self.STOPLINE_BRAKING_BUFFER = 1.0 # TODO: this has to be tuned eventually..
            self.SMOOTH_APPROACH_TO_STOPLINE_FACTOR = 0.25 # this is just a heuristic factor.....feel free to play around

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_pos = None
        #self.base_way = None
        self.tl_position = None
        self.tl_waypoint_index = -1
        self.distance_to_light = 0

        self.next_tl_state = TrafficLight.UNKNOWN

        self.tl_detector_running = False

        self.latest_current_velocity_msg = None

        self.cycleCounter = 0


        #rate is same as DBW node rospy rate
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            self.do_update()
            rate.sleep()

    def getMode(self):
        speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600. # m/s

        # CARLA track has speed limit 10
        if (speed_limit > 10):
            # assume we are in simulator mode
            rospy.logdebug("Speed limit < 10 - assuming simulator mode")
            mode = "SIM"
        else:
            # assume we are in Carla mode - but is it simulator churchlot or real drive!?
            is_site_configured = rospy.has_param("/grasshopper_calibration_yaml")

            if is_site_configured == False:
                rospy.logdebug("No grasshopper found - assuming simulator churchlot mode")
                mode = "SIM_CHURCHLOT"
            else:
                rospy.logdebug("Grasshopper found - assuming real-drive mode")
                mode = "CARLA"

        rospy.loginfo("Identified mode %s" , mode)
        return mode

    def pose_cb(self, msg):
        #save msg of current position to it's variable
        self.current_pos = msg

        # Every time we get a pose message, calculate distance to light.
        if self.tl_position != None:
            tl = self.tl_position
            c = self.current_pos.pose.position
            if(self.tl_waypoint_index != -1):
                self.distance_to_light = math.sqrt(pow(tl.x - c.x,2) + pow(tl.y - c.y,2))
            else:
                self.distance_to_light = MIN_DISTANCE_TO_LIGHT

    def waypoints_cb(self, waypoints):
        #save msg of base waypoints to it's variable
        self.base_way = waypoints
        self.base_way_length = len(self.base_way.waypoints)
        rospy.logdebug("Base Waypoint Length %s", self.base_way_length)
        self.initialized = True


    def traffic_cb(self, msg):
        if self.initialized == False:
            rospy.logwarn("DEBUG: Waypoints not initialized")
            return
        elif self.base_way == None:
            rospy.logwarn("DEBUG: self.base_way == None")
            return
        elif self.base_way.waypoints == None:
            rospy.logwarn("DEBUG: self.base_way.waypoints == None")
            return
        elif len(self.base_way.waypoints) == 0:
            rospy.logwarn("DEBUG: len(self.base_way.waypoints) == 0")
            return

        self.tl_detector_running = True
        self.next_tl_state = msg.state
        #rospy.logwarn("Setting Traffic State to %s",self.next_tl_state)

        self.tl_waypoint_index = msg.waypoint
        self.tl_position = self.base_way.waypoints[msg.waypoint].pose.pose.position

        tl = self.tl_position
        c = self.current_pos.pose.position

        if(self.tl_waypoint_index != -1):
            self.distance_to_light = math.sqrt(pow(tl.x - c.x,2) + pow(tl.y - c.y,2))
        else:
            self.distance_to_light = MIN_DISTANCE_TO_LIGHT

        #dist = self.distance(self.base_way.waypoints , self.get_closest_point() ,self.tl_waypoint_index)
        #rospy.loginfo("Car Pos = %s", self.get_closest_point() )
        #rospy.logwarn("TL Pos = %s", self.tl_waypoint_index )
        rospy.loginfo("Distance to next TL = %s", self.distance_to_light )
        rospy.loginfo("State of the next TL = %s", self.next_tl_state )
        #rospy.logwarn("DIST to next TL = %s", dist)
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

    def get_Euler_Angle(self,pose):
        """Returns the roll, pitch yaw angles from a Quaternion """
        return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                        pose.orientation.y,
                                                        pose.orientation.z,
                                                        pose.orientation.w])

    def is_waypoint_front(self,pose, waypoint):
        """Take a waypoint and a pose , do a coordinate system transformation
        setting the origin at the position of the pose object and as x-axis
        the orientation of the z-axis of the pose
        Args:
            pose (object) : A pose object
            waypoints (object) : A waypoint object
        Returns:
            bool : True if the waypoint is behind the car False if in front
        """
        _, _, pose_yaw = self.get_Euler_Angle(pose)
        pose_X = pose.position.x
        pose_Y = pose.position.y

        waypoint_X = waypoint.pose.pose.position.x
        waypoint_Y = waypoint.pose.pose.position.y

        shift_x = waypoint_X - pose_X
        shift_y = waypoint_Y - pose_Y

        if (shift_x * math.cos(0 - pose_yaw) - shift_y * math.sin(0 - pose_yaw)) > 0:
            return True
        else:
            return False


    #This function is used to loop over all the points and find the nearest one to the current point and return it's index
    def get_closest_point(self):
        min_delta = 10000000
        min_idx = 0
        for i in range(len(self.base_way.waypoints)):
            p = self.base_way.waypoints[i].pose.pose.position
            c = self.current_pos.pose.position
            delta = math.sqrt((p.x - c.x)**2 + (p.y - c.y)**2  + (p.z - c.z)**2)
            if(delta < min_delta):
                min_delta = delta
                min_idx = i

        if self.mode == "SIM_CHURCHLOT" or self.mode == "CARLA":
            return min_idx

        # filtering of waypoints is only required for simulator mode
        # because the waypoints are pretty tightly packed and you
        # could easily take a waypoint behind the vehicle.....
        is_in_front = self.is_waypoint_front(self.current_pos.pose, self.base_way.waypoints[min_idx])

        if self.waypoints_increasing == True:
            while is_in_front == False:
                #rospy.logwarn("Waypoint NOT in front!!!")
                min_idx = (min_idx + 1) % self.base_way_length
                is_in_front = self.is_waypoint_front(self.current_pos.pose, self.base_way.waypoints[min_idx])
        else:
            while is_in_front == True:
                #rospy.logwarn("Waypoint NOT in front!!!")
                min_idx = (min_idx + 1) % self.base_way_length
                #rospy.logwarn("New min_idx = %s" , min_idx)
                is_in_front = self.is_waypoint_front(self.current_pos.pose, self.base_way.waypoints[min_idx])

        return min_idx


    #This function is used to select number of base waypoints to be added to the final waypoint msg to be published
    #Points are the nearest LOOKAHEAD_WPS points to the current position
    def do_update(self):
        if self.tl_detector_running == False:
            return

        self.cycleCounter += 1
        if(self.base_way != None) and (self.current_pos != None):

            #########################################################
            # 1. Find out whether we should stop or not
            #########################################################
            if self.next_tl_state == TrafficLight.RED:
                if (self.distance_to_light < MIN_DISTANCE_TO_LIGHT):
                    self.stopping = True
                    #rospy.logwarn("Decelerate: Distance to light: %d", self.distance_to_light)

                else:
                    self.stopping = False
            elif self.next_tl_state == TrafficLight.YELLOW:
                if (self.distance_to_light < MIN_DISTANCE_TO_LIGHT):
                    #Is it possible to pass the traffic light in time!?
                    # http://www.kfz-tech.de/Biblio/Formelsammlung/Bremsweg.htm
                    current_vel = self.latest_current_velocity_msg.twist.linear.x
                    brakingDistance = abs(current_vel*current_vel / (2 * (abs(self.deceleration_limit) - 0.5)))

                    if brakingDistance > self.distance_to_light:
                        #rospy.logwarn("DON'T STOP AT YELLOW: Braking distance: %s > Distance to light: %s", brakingDistance , self.distance_to_light)
                        self.stopping = False
                    else:
                        self.stopping = True
                        #rospy.logwarn("STOP AT YELLOW: Braking distance: %s < Distance to light: %s", brakingDistance , self.distance_to_light)
                else:
                    self.stopping = False

            else:
                self.stopping = False

            rospy.loginfo("Stopping from Waypoint updater = %s", self.stopping )

            #########################################################
            # 2. Calculate the next waypoints
            #########################################################
            self.final_way = Lane()
            closest_idx = self.get_closest_point()

            if self.waypoints_increasing == True:
                sign = +1
            else:
                sign = -1

            #rospy.logwarn("Closest Waypoint %s", closest_idx)
            #rospy.logwarn("Range start %s", closest_idx)
            #rospy.logwarn("Range end %s", closest_idx + sign * LOOKAHEAD_WPS)
            #rospy.logwarn("sign %s", sign)
            for i in range(closest_idx, (closest_idx + sign * LOOKAHEAD_WPS), sign):
                idx = i % self.base_way_length
                #rospy.logwarn("new waypoint %s", idx)
                wp = copy.deepcopy(self.base_way.waypoints[idx])

                #########################################################
                # 3. Set the appropriate speed for each waypoint
                #########################################################
                if (self.stopping == True and self.distance_to_light > 0.2):
                    if self.mode == "CARLA":
                        current_vel = 2  #TODO: Antonia self.latest_current_velocity_msg.twist.linear.x is not available in CARLA !?
                        #current_vel = self.latest_current_velocity_msg.twist.linear.x
                    else:
                        current_vel = self.latest_current_velocity_msg.twist.linear.x

                    a  = current_vel*current_vel/(2*self.distance_to_light)  # http://www.kfz-tech.de/Biblio/Formelsammlung/Bremsweg.htm
                    #rospy.logwarn("a = %s" , a)
                    new_vel = (current_vel - a) 

                    # ensure a smooth approach until the stopline
                    if self.distance_to_light < self.STOPLINE_BRAKING_BUFFER:
                        wp.twist.twist.linear.x = 0.0
                    else:
                        wp.twist.twist.linear.x = max(new_vel,
                                                  abs(self.SMOOTH_APPROACH_TO_STOPLINE_FACTOR * (self.distance_to_light - self.STOPLINE_BRAKING_BUFFER)))

                    # For stoping cases, set a min speed, else we creep to red light.
                    if wp.twist.twist.linear.x < MIN_STOPPING_SPEED :
                        wp.twist.twist.linear.x = 0.0

                        #rospy.logwarn("Setting target-speed = %s" , wp.twist.twist.linear.x)
                else:
                    wp.twist.twist.linear.x = self.speed_limit  

                wp.twist.twist.linear.x = min(wp.twist.twist.linear.x,self.speed_limit) # don't exceed max speed
                wp.twist.twist.linear.x = max(wp.twist.twist.linear.x,0.0) # don't provide negative speed

                #if (self.cycleCounter % 50 == 0):
                #    rospy.logwarn("Setting target-speed = %s" , wp.twist.twist.linear.x)
 

                self.final_way.waypoints.append(wp)

            #rospy.logwarn("Publish Waypoints: %s", self.final_way.waypoints[0])
            self.final_waypoints_pub.publish(self.final_way)


    def current_vel_callback(self, cb_msg):
        self.latest_current_velocity_msg = cb_msg
        #rospy.logwarn("Last velocity = %s " ,  self.latest_current_velocity_msg.twist.linear.x)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
