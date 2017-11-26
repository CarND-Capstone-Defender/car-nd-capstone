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

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector' , log_level=rospy.DEBUG)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []


        # List of positions that correspond to the line to stop in front of for a given intersection
        self.trafficLightWaypoint = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        #self.upcoming_traffic_light_injection = rospy.Publisher('/traffic_waypoint_injection', TrafficLightDetection, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        self.waypoints = lane.waypoints

        stop_line_positions = self.config['stop_line_positions']
        #print('stop_line_positions = ' , stop_line_positions) 
        for i in range(len(stop_line_positions)):
            nextWaypoint = self.get_closest_waypoint(stop_line_positions[i][0],stop_line_positions[i][1])
            self.trafficLightWaypoint.append(nextWaypoint)
            #print('Waypoint[' , i , '] = ' , nextWaypoint)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

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
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) 

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position.x ,self.pose.pose.position.y)

            #print('Car position: ', car_position)

            waypointUntilNextStopline = -1
            for i in range(len(self.trafficLightWaypoint)):
                if car_position < self.trafficLightWaypoint[i] and waypointUntilNextStopline == -1:
                    waypointUntilNextStopline = self.trafficLightWaypoint[i] - car_position

            # if no traffic stopline has been found car_position must be beyond the last traffic stopline
            # --> "estimate" the distance as the distance to first stopline and wait until
            #     car_position > 0 again      
            if waypointUntilNextStopline == -1:
                waypointUntilNextStopline = self.trafficLightWaypoint[0]

            #TODO find the closest visible traffic light (if one exists)
            light_wp = 1
            state = self.get_light_state()

            if state == TrafficLight.UNKNOWN:
                print(waypointUntilNextStopline , ' waypoints until next traffic light stopline with color UNKNOWN')
            elif state == TrafficLight.RED:
                print(waypointUntilNextStopline , ' waypoints until next traffic light stopline with color RED')
            elif state == TrafficLight.YELLOW:
                print(waypointUntilNextStopline , ' waypoints until next traffic light stopline with color YELLOW')
            elif state == TrafficLight.GREEN:
                print(waypointUntilNextStopline , ' waypoints until next traffic light stopline with color GREEN')

            if state == TrafficLight.UNKNOWN:
                return -1, TrafficLight.UNKNOWN
            else:
                return waypointUntilNextStopline, state
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
