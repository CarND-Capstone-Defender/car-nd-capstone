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


MIN_DISTANCE_TO_LIGHT  = 100
LOOKAHEAD_WPS = 25 # Number of waypoints we will publish. You can change this number

#MPH_TO_MPS = 0.44704 # 0.44704 is factor from miles per hour --> meter per second
#MAX_SPEED_SIM = 25 * MPH_TO_MPS # maximum speed in simulator is 25 miles per hour
#MAX_SPEED_CARLA = 10 * MPH_TO_MPS # maximum speed on Carla is 10 miles per hour

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity',TwistStamped, self.current_vel_callback)


        config_string = rospy.get_param("/traffic_light_config")
        config = yaml.load(config_string)
        stop_line_positions = config['stop_line_positions']
        # CARLA track has only 1 traffic light
        if (len(stop_line_positions) > 2):
            # assume we are in simulator mode
            self.mode = "SIM" 
            self.waypoints_increasing = True
        else:
            # assume we are in Carla mode
            self.mode = "CARLA" 
            self.waypoints_increasing = False

        rospy.Subscriber('/traffic_waypoint', TrafficLightDetection, self.traffic_cb)
        # rospy.Subscriber('/traffic_waypoint_injection', TrafficLightDetection, self.traffic_cb)

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600. # m/s
        self.deceleration_limit = rospy.get_param('/dbw_node/decel_limit') 
        rospy.logdebug("Speed limit = %s" , self.speed_limit)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        #current position & base waypoint msgs variables
        self.current_pos = None
        self.base_way = None
        self.tl_position = None
        self.tl_waypoint_index = -1
        self.distance_to_light = 0

        self.next_tl_state = TrafficLight.UNKNOWN

        self.tl_detector_running = False

        self.latest_current_velocity_msg = None

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
            if(self.tl_waypoint_index != -1):
                self.distance_to_light = math.sqrt(pow(tl.x - c.x,2) + pow(tl.y - c.y,2))
            else:
                self.distance_to_light = 300

    def waypoints_cb(self, waypoints):
        #save msg of base waypoints to it's variable
        self.base_way = waypoints
        self.base_way_length = len(self.base_way.waypoints)
        rospy.logdebug("Base Waypoint Length %s", self.base_way_length)
        

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
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
            self.distance_to_light = MIN_DISTANCE_TO_LIGHT + 1
            
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

        if(self.base_way != None) and (self.current_pos != None):

            #########################################################
            # 1. Find out whether we should stop or not
            #########################################################
            if ((self.next_tl_state == TrafficLight.YELLOW) or (self.next_tl_state == TrafficLight.RED)):
                if (self.distance_to_light <= MIN_DISTANCE_TO_LIGHT):
                    self.stopping = True
                    #rospy.logwarn("Decelerate: Distance to light: %d", self.distance_to_light)

                else:
                    self.stopping = False
            else:
                self.stopping = False



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
            dist = self.distance(self.base_way.waypoints , self.get_closest_point(), self.tl_waypoint_index) - 2

            for i in range(closest_idx, (closest_idx + sign * LOOKAHEAD_WPS), sign):
                idx = i % self.base_way_length
                #rospy.logwarn("new waypoint %s", idx)
                wp = copy.deepcopy(self.base_way.waypoints[idx])

                #########################################################
                # 3. Set the appropriate speed for each waypoint
                #########################################################
                if (self.stopping == True and dist > 0.2):
                    
                    current_vel = self.latest_current_velocity_msg.twist.linear.x
                    #new_vel = math.sqrt(current_vel - 1.1  # http://www.kfz-tech.de/Biblio/Formelsammlung/Bremsweg.htm
                    a  = current_vel*current_vel/(2*dist)  # http://www.kfz-tech.de/Biblio/Formelsammlung/Bremsweg.htm
                    #a = a/20 # 50ms is update rate  #Antonia: this is too weak for some reasons.....
                    #rospy.logwarn("a = %s" , a)
                    new_vel = (current_vel - a)
                    new_vel = min(new_vel,self.speed_limit) # don't exceed max speed
                    new_vel = max(new_vel,0) # don't provide negative speed
                    #rospy.logwarn("Distance to next TL = %s , target-speed = %s", dist ,new_vel )

                    if dist < 1.5 or new_vel < 0.01:
                        wp.twist.twist.linear.x = 0
                    else:
                        wp.twist.twist.linear.x = new_vel
                    #v = wp.twist.twist.linear.x
                else:
                    wp.twist.twist.linear.x = self.speed_limit

                #rospy.logwarn("Setting target-speed = %s" , wp.twist.twist.linear.x)

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
