#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight, TrafficLightDetection
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

# specifies the number of confirmed occurrences of same color
# until a traffic light detection is considered as reliable
STATE_COUNT_THRESHOLD = 2


class TLDetector(object):
    def __init__(self):
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        # specifies a rate which images shall be used in order to
        # deal with long detection time in the neural network
        # example:
        #   1 = every frame is taken
        #   2 = only every second frame is taken
        #   3 = only every third frame is taken
        #   .....
        self.FRAME_RATE = 3


        #rospy.init_node('tl_detector' , log_level=rospy.DEBUG)
        rospy.init_node('tl_detector' , log_level=rospy.INFO)

        self.fullInitialized = False
        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        self.last_vehicle_position = -1
        self.waypoints_increasing = True
        self.total_frame_counter = 0
        self.vehicle_position = -1
        self.state_count = 0

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.light_wp = -1
        self.last_wp = -1
        self.red_counter = 0

        # List of positions that correspond to the line to stop in front of for a given intersection
        self.trafficLightWaypoint = []

        self.mode = self.getMode()

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', TrafficLightDetection , queue_size=1)
        #self.upcoming_traffic_light_injection = rospy.Publisher('/traffic_waypoint_injection', TrafficLightDetection, queue_size=1)

        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        rospy.spin()

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
        if self.fullInitialized == False:
            return

        self.pose = msg
        self.vehicle_position = self.get_closest_waypoint(self.pose.pose.position.x ,self.pose.pose.position.y)
        #rospy.logwarn('vehicle_position = %s' , self.vehicle_position)

        waypoint_distance = abs(self.vehicle_position - self.last_vehicle_position)

        if self.vehicle_position == self.last_vehicle_position:
            #rospy.logdebug('Waypoint increasing = %s' , self.waypoints_increasing)
            return

        if self.vehicle_position > self.last_vehicle_position or (self.vehicle_position < 10 and self.last_vehicle_position > len(self.waypoints)-10):
            self.waypoints_increasing = True
        elif self.vehicle_position < self.last_vehicle_position :
            self.waypoints_increasing = False
            rospy.logwarn('Waypoint increasing = %s' , self.waypoints_increasing)
            rospy.logwarn('Current Waypoint = %s , Last Waypoint = %s' , self.vehicle_position , self.last_vehicle_position)

        self.last_vehicle_position = self.vehicle_position

        rospy.logdebug('Waypoint increasing = %s' , self.waypoints_increasing)

        #ANTONIA: Use these 5 lines in order to test the churchlot in the simulator
        #         otherwise you won't get the tl_detector and thus the waypoint updater
        #         because in this churchlot mode the simulator doesn't provide images...
        if self.mode == "SIM_CHURCHLOT":
            if self.last_vehicle_position > 49 or self.last_vehicle_position < 3:
                self.publishWaypoint(58,TrafficLight.RED)
            else:
                self.publishWaypoint(58,TrafficLight.GREEN)


    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

        # calculate the corresponding waypoints from
        # the traffic light stop line positions
        # NOTE: moved this calculation here because we know that
        #       self.waypoints has been provided (could be asynchronuous !)
        stop_line_positions = self.config['stop_line_positions']

        if len(stop_line_positions) == 0:
            rospy.logerror('Error: No stop_line_positions available!!')

        for i in range(len(stop_line_positions)):
            nextWaypoint = self.get_closest_waypoint(stop_line_positions[i][0],stop_line_positions[i][1])
            rospy.loginfo('stopline_waypoint[%s] = %s' , i  , nextWaypoint)
            self.trafficLightWaypoint.append(nextWaypoint)

        self.fullInitialized = True
        rospy.loginfo('TL Detector fully initialized')


    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.fullInitialized == False:
            return

        self.total_frame_counter += 1

        if (self.total_frame_counter % self.FRAME_RATE != 0):
            rospy.logdebug('dropping frame #%s as dropping rate is %s' , self.total_frame_counter , self.FRAME_RATE)
            return

        self.has_image = True
        self.camera_image = msg
        self.light_wp, self.state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.last_state != self.state:
            # reset the state counter
            if self.last_state == TrafficLight.YELLOW:
                self.last_state = self.state
                # as we treat yellow similar to red don't reset the red counter here!!
                # effect is that a found red color has no delay of THRSTATE_COUNT_THRESHOLD occurrences
            else:
                self.red_counter = 0
                self.last_wp = -1
                self.last_state = self.state

        elif self.state == TrafficLight.RED:
            self.red_counter += 1  #increase the red occurrence counter
            rospy.logdebug('RED occurred now  %s times' , self.red_counter)
            if self.red_counter >= STATE_COUNT_THRESHOLD:
                # threshold reached!
                if self.waypoints_increasing == True:
                    if self.light_wp <= self.last_wp+3 or self.last_wp == -1:
                        # don't report red in case we already crossed the traffic light stop line !!
                        self.last_wp = self.light_wp
                        rospy.logdebug('WP increasing - finally publishing RED light at waypoint %s' , self.light_wp)
                        self.publishWaypoint(self.light_wp,self.state)
                        # self.upcoming_red_light_pub.publish(Int32(self.light_wp))
                else:
                    ######################
                    # UDACITY CARLA HACK Start
                    # Following special treatment is a hack in order to deal
                    # with the high inaccuracy in calculation of nearest waypoint
                    # within the area of waypoint 50...60...0...3
                    ######################
                    #if self.mode == "CARLA" and (self.vehicle_position > 51 or self.vehicle_position <3):
                       # self.last_wp = self.light_wp
                       # #rospy.loginfo('Finally publishing RED light in %s waypoints' , 0)
                       # rospy.logdebug('CARLA HACK: Finally publishing RED light at waypoint %s' , self.light_wp)
                       # #self.upcoming_red_light_pub.publish(Int32(0))
                       # self.publishWaypoint(self.light_wp,self.state)
                       # return
                    ######################
                    ## HACK End
                    ######################

                    self.last_wp = self.light_wp
                    rospy.loginfo('WP decreasing - finally publishing RED light at waypoint %s' , self.light_wp)
                    self.publishWaypoint(self.light_wp,self.state)


        elif self.state == TrafficLight.GREEN:
            # reset the red counter --> go!
            #self.FRAME_RATE = 4
            self.red_counter = 0
            self.last_wp = -1
            self.last_state = self.state
            rospy.logdebug('Go GREEN - finally publishing RED light at waypoint %s' , -1)
            self.publishWaypoint(-1,self.state)
        elif self.state == TrafficLight.YELLOW:
            # treat yellow as "almost" red..... --> be cautious
            #self.FRAME_RATE = 4
            self.red_counter += 1
            self.last_wp = -1
            rospy.logdebug('YELLOW occurred now  %s times' , self.red_counter)
            self.last_state = self.state
            rospy.logdebug('Go YELLOW - finally publishing YELLOW light at waypoint %s' , -1)
            self.publishWaypoint(self.light_wp, self.state)
        else:
            # seems to be unknown detection..... --> go!
            # reset the red counter
            #self.FRAME_RATE = 4
            self.red_counter = 0
            self.last_wp = -1
            self.last_state = self.state
            rospy.logdebug('Go UNKNOWN - finally publishing RED light at waypoint %s' , -1)
            self.publishWaypoint(-1,self.state)

    def get_closest_waypoint(self, posx , posy):
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

    def get_light_state(self):
        """Determines the current color of the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            rospy.logdebug('get_light_state() was called without an images being received...')
            return False

        #Just call the classifier...
        return self.light_classifier.get_classification(self.camera_image)

    def getWaypointsUntilNextStopline(self):
        # 1) First of all get the current vehicle position
        car_position = self.get_closest_waypoint(self.pose.pose.position.x ,self.pose.pose.position.y)
        #rospy.logdebug('Car position: %s' , car_position)

        # 2) now calculate the number of waypoints until the next traffic light stopline
        waypointUntilNextStopline = -1

        for i in range(len(self.trafficLightWaypoint)):
            if car_position < self.trafficLightWaypoint[i] and waypointUntilNextStopline == -1:
                waypointUntilNextStopline = self.trafficLightWaypoint[i] - car_position

        # if no traffic stopline has been found car_position must be beyond the last traffic stopline
        # --> "estimate" the distance as the distance to first stopline and wait until
        #     car_position > 0 again
        if waypointUntilNextStopline == -1:
            #rospy.logdebug('END OF TRACK....')
            waypointUntilNextStopline = (len(self.waypoints) - car_position) + self.trafficLightWaypoint[0]


        if self.waypoints_increasing == False:
            waypointUntilNextStopline = len(self.waypoints) - waypointUntilNextStopline



        return waypointUntilNextStopline

    def getClosestTrafficLightWaypoint(self):
        # 1) First of all get the current vehicle position
        car_position = self.get_closest_waypoint(self.pose.pose.position.x ,self.pose.pose.position.y)
        #rospy.logdebug('Car position: %s' , car_position)

        # start with the first traffic light waypoint

        if self.waypoints_increasing == True:
            index = 0
            # loop through all traffic light waypoints and get the closest one
            for i in range(len(self.trafficLightWaypoint)):
                if car_position < self.trafficLightWaypoint[i]:
                    index =  i
                    break
        else:
            index = len(self.trafficLightWaypoint)-1
            # loop through all traffic light waypoints and get the closest one
            for i in range(len(self.trafficLightWaypoint)):
                if car_position > self.trafficLightWaypoint[len(self.trafficLightWaypoint) - i - 1]:
                    index =  len(self.trafficLightWaypoint) - i - 1
                    break

        return self.trafficLightWaypoint[index]

    def publishWaypoint(self, wp , status):
        detectorMsg = TrafficLightDetection()
        detectorMsg.waypoint = wp

        if status == TrafficLight.RED:
            detectorMsg.state = TrafficLight.RED
        elif status == TrafficLight.YELLOW:
            detectorMsg.state = TrafficLight.YELLOW
        elif status == TrafficLight.GREEN:
            detectorMsg.state = TrafficLight.GREEN
        else:
            detectorMsg.state = TrafficLight.UNKNOWN

        #if detectorMsg.state == TrafficLight.GREEN:
            #rospy.loginfo('Publishing waypoint %s with color GREEN ' , detectorMsg.waypoint)
        #elif detectorMsg.state == TrafficLight.RED:
            #rospy.loginfo('Publishing waypoint %s with color RED ' , detectorMsg.waypoint)
        #elif detectorMsg.state == TrafficLight.YELLOW:
            #rospy.loginfo('Publishing waypoint %s with color YELLOW ' , detectorMsg.waypoint)
        #elif detectorMsg.state == TrafficLight.UNKNOWN:
            #rospy.loginfo('Publishing waypoint %s with color UNKNOWN ' , detectorMsg.waypoint)

        self.upcoming_red_light_pub.publish(detectorMsg)


    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        #TODO find the closest visible traffic light (if one exists)

        if(self.pose):
            # 1. Get the waypoints to go before the next stopline
            waypointsToGo = self.getWaypointsUntilNextStopline()

            # 2. in case we are too far away from a traffic light we can also skip TL detection....
            if waypointsToGo > 200:
                #rospy.loginfo('Shut off TL detection....')
                return -1, TrafficLight.UNKNOWN

            # 3. Detect the color of the traffic light (if yet visible)
            state = self.get_light_state()

            nextTLWaypoint = self.getClosestTrafficLightWaypoint()


            # Just debug....
            rospy.logdebug('%s waypoints to next TL stopline' , waypointsToGo)

            if state == TrafficLight.UNKNOWN:
                return -1, TrafficLight.UNKNOWN
            else:
                return nextTLWaypoint, state
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
