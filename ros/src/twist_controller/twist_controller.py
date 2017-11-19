import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


# Tuned for simulator, unknown if representitve for Carla.
VEL_PID_KP = 1.0
VEL_PID_KI = 0.001
VEL_PID_KD = 0.0000001
# Trise = 2.2T , say 5sec 10-90% todo: include accel limit. https://en.wikipedia.org/wiki/RC_time_constant
THROTTLE_LP_TAO =  5.0 / 2.2
THROTTLE_LP_TS = 0.02 # Assuming 50 hz operation 1.0 / 50.0
MAX_THROTTLE = 1.0 * 0.9
THROTTLE_PID_LIMIT_MAX = 1.0 #float('inf')
THROTTLE_PID_LIMIT_MIN = -1.0 #float('-inf')



class TwistController(object):
    def __init__(self, **veh_caps):

        # Capture key vehicle capabilites

        rospy.logwarn(str(veh_caps))
        self.vehicle_mass =     veh_caps['vehicle_mass']
        self.fuel_capacity =    veh_caps['fuel_capacity']
        self.brake_deadband =   veh_caps['brake_deadband']
        self.decel_limit =      veh_caps['decel_limit']
        self.accel_limit =      veh_caps['accel_limit']
        self.wheel_radius =     veh_caps['wheel_radius']
        self.wheel_base =       veh_caps['wheel_base']
        self.steer_ratio =      veh_caps['steer_ratio']
        self.max_lat_accel =    veh_caps['max_lat_accel']
        self.max_steer_angle =  veh_caps['max_steer_angle']

        self.min_speed = 0.0

        # 1st implementation of steering is to use provided Yaw Controller.
        self.yaw = YawController( self.wheel_base,
                                  self.steer_ratio,
                                  self.min_speed,
                                  self.max_lat_accel,
                                  self.max_steer_angle)

        self.throttle_pid = PID(VEL_PID_KP,VEL_PID_KI,VEL_PID_KD,THROTTLE_PID_LIMIT_MIN,THROTTLE_PID_LIMIT_MAX)
        self.throttle_lp = LowPassFilter(THROTTLE_LP_TAO,THROTTLE_LP_TS)
        self.lastTime = 0.0
        self.first_call = True
        self.pref_vel = 0.0

    def control(self,
                proposed_linear_vel,
                proposed_angular_vel,
                current_linear_vel,
                dbw_enabled):

        throttle = 0.0
        brake = 0.0
        steer = 0.0

        # Testing - hardcode proposed velicity
        proposed_linear_vel = 10.0 * ONE_MPH # 0.621371 # Why is ONE_MPH 0.44704 above?

        if self.first_call:
            # It would be best to get timestampe from currnet vel message but not populated.
            # We assume that current velicity is now, and no latency in transport to this node.

            while True: # Time may not valid at startup, should wait for time to be setup.
                 self.lastTime = rospy.get_time()
                 if self.lastTime != 0.0:
                     break

            self.prev_vel = current_linear_vel
            self.prev_throttle = 0.0
            self.throttle_pid.reset()
            self.first_call = False

        else:

            # Calculate time since last message for PID controller.
            now = rospy.get_time()
            dt = now - self.lastTime
            self.lastTime = now

            delta_vel = proposed_linear_vel - current_linear_vel


            if (dbw_enabled):
                #  For Steerig - use provided yaw controller
                steer = self.yaw.get_steering( proposed_linear_vel, proposed_angular_vel, current_linear_vel)
                error = self.throttle_pid.step(delta_vel,dt)

                #if (error > 0.0 ):
                    # Acceleration condition.
                #    throttle = self.throttle_lp.filt(error)
                #else:
                #    throttle = 0.0
                throttle = self.throttle_lp.filt(error)
                if throttle <= 0.0:
                    throttle = 0.0

                rospy.logwarn("Error:%f : Throttle:%f : DeltaVel:%f Target vel:%f : Actual Vel:%f",error,throttle,delta_vel,proposed_linear_vel,current_linear_vel)

            else:
                self.throttle_pid.reset()
                throttle = brake = steer = 0.0

        return throttle,brake,steer
