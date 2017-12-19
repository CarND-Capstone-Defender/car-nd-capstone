#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import TwistController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_capabilites = {
            'vehicle_mass' : rospy.get_param('~vehicle_mass', 1736.35) ,
            'fuel_capacity' : rospy.get_param('~fuel_capacity', 13.5),
            'brake_deadband' : rospy.get_param('~brake_deadband', .1),
            'decel_limit' : rospy.get_param('~decel_limit', -5),
            'accel_limit' : rospy.get_param('~accel_limit', 1.),
            'wheel_radius' : rospy.get_param('~wheel_radius', 0.2413),
            'wheel_base' : rospy.get_param('~wheel_base', 2.8498),
            'steer_ratio' : rospy.get_param('~steer_ratio', 14.8),
            'max_lat_accel' : rospy.get_param('~max_lat_accel', 3.),
            'max_steer_angle' : rospy.get_param('~max_steer_angle', 8.) }

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # Set the initial stage prior to recieving any messages.
        self.dbw_enabled = False
        self.latest_current_velocity_msg = None
        self.latest_twist_msg=None

        self.controller = TwistController(**vehicle_capabilites)

        # Messages arrive async, each callback updates this
        # class with private copy of last update seen.

        #Topic to receive target linear and angular velocities
        rospy.Subscriber('/twist_cmd',TwistStamped,self.twist_cmd_callback)
        # Subscribe to current velocity topic.
        rospy.Subscriber('/current_velocity',TwistStamped, self.current_vel_callback)

        #Messge that indicates if dbw is enabled or not.
        rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_enabled_callback)

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Get predicted throttle, brake, and steering using `twist_controller`
            if (self.latest_twist_msg != None and self.latest_current_velocity_msg != None):
                proposed_linear_vel = self.latest_twist_msg.twist.linear.x
                proposed_angular_vel = self.latest_twist_msg.twist.angular.z
                current_linear_vel = self.latest_current_velocity_msg.twist.linear.x
                #if current_linear_vel < 0.001 :
                #    current_linear_vel = 0.

                throttle, brake, steer = self.controller.control(proposed_linear_vel,
                                                                proposed_angular_vel,
                                                                current_linear_vel,
                                                                self.dbw_enabled)
            else:
                throttle =  brake = steer = 0.

            if ( self.dbw_enabled is True ):
                self.publish(throttle, brake, steer)

            rate.sleep()

    def publish(self, throttle, brake, steer):

        #rospy.logwarn(" Throttle:%f : brake:%f Steer:%f ",throttle,brake,steer)

        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


    def dbw_enabled_callback(self, cb_msg):
        # Extract True/False message.
        self.dbw_enabled = cb_msg.data

    def current_vel_callback(self, cb_msg):
        self.latest_current_velocity_msg = cb_msg

    def twist_cmd_callback(self,cb_msg):
        self.latest_twist_msg = cb_msg





if __name__ == '__main__':
    DBWNode()
