import math
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller):
        self.yaw_controller = yaw_controller

    def control(self, current_twist, proposed_twist):
        linear_velocity = proposed_twist.twist.linear.x
        angular_velocity = proposed_twist.twist.angular.z
        current_velocity = current_twist.twist.linear.x
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        throttle = 0.75
        brake = 0.0
        return throttle, brake, steer
