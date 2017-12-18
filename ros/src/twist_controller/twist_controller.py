import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


YAW_LP_TAO = 0.2
YAW_LP_TS = 0.1

# Tuned for simulator, unknown if representitve for Carla.
VEL_PID_KP = 0.8
VEL_PID_KI = 0.0001
VEL_PID_KD = 0.0000001
# Trise = 2.2T , say 5sec 10-90% todo: include accel limit. https://en.wikipedia.org/wiki/RC_time_constant
THROTTLE_LP_TAO =  2.0 / 2.2
THROTTLE_LP_TS = 0.02 # Assuming 50 hz operation 1.0 / 50.0

BRAKE_LP_TAO = 0.1/2.2
BRAKE_LP_TS = 0.02 # Assuming 50 hz operation 1.0 / 50.0


MAX_THROTTLE = 1.0 * 0.9
THROTTLE_PID_LIMIT_MAX = 1.0 #float('inf')
THROTTLE_PID_LIMIT_MIN = -1.0 #float('-inf')

CURRENT_FUEL_LEVEL = 100  # Percent full

MAX_brake_FORCE_RATIO = 1



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

        self.current_vehicle_mass = self.vehicle_mass + (( CURRENT_FUEL_LEVEL / 100) * (self.fuel_capacity * GAS_DENSITY))
        # @Kostas Oreopoulos - carnd slack commennt
        self.max_brake_force = abs(MAX_brake_FORCE_RATIO * self.vehicle_mass * self.decel_limit * self.wheel_radius)


        # 1st implementation of steering is to use provided Yaw Controller.
        self.yaw = YawController( self.wheel_base,
                                  self.steer_ratio,
                                  self.min_speed,
                                  self.max_lat_accel,
                                  self.max_steer_angle)

        self.yaw_lp = LowPassFilter(YAW_LP_TAO,YAW_LP_TS)



        self.throttle_pid = PID(VEL_PID_KP,VEL_PID_KI,VEL_PID_KD,THROTTLE_PID_LIMIT_MIN,THROTTLE_PID_LIMIT_MAX)
        self.throttle_lp = LowPassFilter(THROTTLE_LP_TAO,THROTTLE_LP_TS)
        self.brake_lp = LowPassFilter(BRAKE_LP_TAO,BRAKE_LP_TS)

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
        #proposed_linear_vel = 20.0 * ONE_MPH # 0.621371 # Why is ONE_MPH 0.44704 above?

        if self.first_call:
            # It would be best to get timestampe from currnet vel message but not populated.
            # We assume that current velicity is now, and no latency in transport to this node.

            while True: # Time may not valid at startup, should wait for time to be setup.
                 self.lastTime = rospy.get_time()
                 if self.lastTime != 0.0:
                     break

            self.prev_vel = current_linear_vel
            self.prev_throttle = 0.0

            self.yaw_lp.filt(.0)

            self.throttle_pid.reset()
            self.throttle_lp.filt(0.)
            self.brake_lp.filt(0.)

            self.first_call = False

        else:

            # Calculate time since last message for PID controller.
            now = rospy.get_time()
            dt = now - self.lastTime
            self.lastTime = now

            #  At breaking, we are generating negative proposed velocity, possible error in waypoint gen while breaking.
            if (proposed_linear_vel < 0.):
                #rospy.logwarn("Proposed liner vel:%f set to zero:", proposed_linear_vel)
                proposed_linear_vel = 0

            delta_vel = proposed_linear_vel - current_linear_vel


            if (dbw_enabled):
                #  For Steerig - use provided yaw controller
                steer_err = self.yaw.get_steering( proposed_linear_vel, proposed_angular_vel, current_linear_vel)
                steer = self.yaw_lp.filt(steer_err)
                #  rospy.logwarn("Yaw:%f : Yaw_lp:%f : P-lin-vel:%f Pre-ang-Vel:%f current-lin-vel:%f",steer_err,steer,proposed_linear_vel,proposed_angular_vel,current_linear_vel)

                #rospy.logwarn("Proposed_linear_vel:%f current_velocity:%f", proposed_linear_vel, current_linear_vel)


                error = self.throttle_pid.step(delta_vel,dt)

                if (error > 0.0 ):
                    # Acceleration condition. , at proposed vel = 0, we get some small accel in current vel, focce accel to zero for that case.
                    if proposed_linear_vel == 0. :
                       throttle = 0.
                    else:
                       throttle = self.throttle_lp.filt(error)
                    self.brake_lp.set(0.) #  While we are accelerating set the brake lp to zero.
                else:
                    throttle = 0.0
                    # use linear map from pid to brake force range. error 0 to -1/
                    brake_force = abs(error) * self.max_brake_force
                    brake = self.brake_lp.filt(brake_force)
                    #rospy.logwarn("BRAKING: BF:%f Brake:%f Error:%f abs_error:%f max_bf:%f proposed_linear_vel:%f",
                    #brake_force, brake, error,abs(error), self.max_brake_force ,proposed_linear_vel)


                    #Need to apply final presure as error gets too small, we creep along.
                    if (brake <= self.brake_deadband * 5):
                        brake = self.brake_deadband * 5

                    self.throttle_lp.set(0.) #  While we are braking set the accelerator lp to zero.

                #rospy.logwarn("PID Error:%f : Throttle:%f : brake:%f DeltaVel:%f Target vel:%f : Actual Vel:%f",error,throttle,brake,delta_vel,proposed_linear_vel,current_linear_vel)

            else:
                self.yaw_lp.filt(.0)
                self.throttle_pid.reset()
                self.throttle_lp.set(0.)
                self.brake_lp.set(0.)
                throttle = brake = steer = 0.0

        return throttle,brake,steer
