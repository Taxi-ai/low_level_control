import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_SPEED = 0.1
KP = 0.3
KI = 0.1
KD = 0.0
MN = 0.0 # minimum throttle
MX = 0.2 # maximum throttle
TAU = 0.5 # 1/(2pi*tau) = cutoff freq
TS = 0.02 # sample time
STOP_SPEED = 0.1
BRAKE = 400
MN_THROTTLE = 0.1


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit,
                 wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):

        self.yaw_controller = YawController(wheel_base, steer_ratio, MIN_SPEED, max_lat_accel, max_steer_angle)

        kp = KP
        ki = KI
        kd = KD
        mn = MN
        mx = MX
        self.throttle_controller = PID(kp, ki, kd, mn, mx)
        #filter noisy velocity
        tau = TAU
        ts = TS
        self.vel_lpf = LowPassFilter(tau, ts)

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        #comfort params
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_vel = None
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled:
            ## needs to reset controller because of PID control
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        current_vel = self.vel_lpf.filt(current_vel)

        rospy.loginfo("Angular vel: {}".format(angular_vel))
        rospy.loginfo("Target vel: {}".format(linear_vel))
        rospy.loginfo("Current vel: {}".format(current_vel))

        steer = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)
        rospy.loginfo("Steer from yaw controller: {}".format(steer))

        vel_error = linear_vel - current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time - self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        rospy.loginfo("Throttle from PID: {}".format(throttle))
        brake = 0

        if linear_vel == 0. and current_vel < STOP_SPEED:
            rospy.logwarn("Target vel is 0, need to stop!")
            throttle = 0
            brake = BRAKE
        elif throttle < MN_THROTTLE and vel_error < 0:
            rospy.logwarn("Moving faster than target vel, need to slow down!")
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steer