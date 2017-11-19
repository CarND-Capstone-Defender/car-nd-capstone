import rospy
from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Tuned for simulator, unknown if representitve for Carla.
VEL_PID_KP = 1.0
VEL_PID_KI = 1.0
VEL_PID_KD = 1.0




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

        self.throttle = PID(VEL_PID_KP,VEL_PID_KI,VEL_PID_KD,0.0,1.0)

    def control(self,
                proposed_linear_vel,
                proposed_angular_vel,
                current_linear_vel,
                dbw_enabled):

        throttle = 1.0
        brake = 0.0
        steer = 0.0


        # TODO: For Steerig - use provided yaw controller.

        if (dbw_enabled):
            steer = self.yaw.get_steering( proposed_linear_vel, proposed_angular_vel, current_linear_vel)

        return throttle,brake,steer
