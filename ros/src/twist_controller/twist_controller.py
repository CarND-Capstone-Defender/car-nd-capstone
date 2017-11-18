import rospy



GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class TwistController(object):
    def __init__(self, **kwargs):

        # TODO: Implement

        rospy.logwarn(str(kwargs))
        self.vehicle_mass =     kwargs['vehicle_mass']
        self.fuel_capacity =    kwargs['fuel_capacity']
        self.brake_deadband =   kwargs['brake_deadband']
        self.decel_limit =      kwargs['decel_limit']
        self.accel_limit =      kwargs['accel_limit']
        self.wheel_radius =     kwargs['wheel_radius']
        self.wheel_base =       kwargs['wheel_base']
        self.steer_ratio =      kwargs['steer_ratio']
        self.max_lat_accel =    kwargs['max_lat_accel']
        self.max_steer_angle =  kwargs['max_steer_angle']



    def control(self,
                proposed_linear_vel,
                proposed_angular_vel,
                current_linear_vel,
                dbw_enabled):

        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.
