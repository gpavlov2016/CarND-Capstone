import rospy

from yaw_controller import YawController
from lowpass import LowPassFilter
from pid import PID
import time

GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MAX_SPEED = 40.0


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.throttle_pid = PID(kwargs['throttle_gains'])
        self.steering_pid = PID(kwargs['steering_gains'])
        self.brake_pid = PID(kwargs['brake_gains'])

        self.last_t = None
        self.filter = LowPassFilter(0.2,0.1)

    '''
    Params:
    target_v - desired linear velocity
    target_w - desired angular velocity
    current_v - current linear velocity
    dbw_enabled - drive by wire enabled (ignore error in this case)
    '''

    def control(self, target_v, target_w, current_v, dbw_enabled):
        # Get throttle value from controller
        if self.last_t is None or not dbw_enabled:
            self.last_t = time.time()
            return 0.0, 0.0, 0.0

        dt = time.time() - self.last_t
        error_v = min(target_v.x, MAX_SPEED*ONE_MPH) - current_v.x
        throttle = self.throttle_pid.step(error_v, dt)
        if error_v < 0:
            # brake = -10.0*error_v   # Proportional braking
            brake = self.brake_pid.step(-error_v, dt)
            brake = max(brake, 1.0)
            throttle = 0.0
        else:
            throttle = max(0.0, min(1.0, throttle))
            brake = 0.0

        error_yaw = target_w.z

        steer = self.steering_pid.step(error_yaw, dt)
        steer = self.filter.filt(steer)
        # steer = 0.0

        rospy.loginfo(("throttle: {} brake: {} current_v: {} " + \
            "target_v: {} error_v: {} steer: {} error_yaw: {}").format(

            throttle, brake, current_v.x, target_v.x, error_v, steer, error_yaw))
        self.last_t = time.time()
        return throttle, brake, steer
