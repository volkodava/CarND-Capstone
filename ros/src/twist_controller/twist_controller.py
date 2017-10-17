import math
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, speed_controller, max_acc, max_dec, vehicle_mass, wheel_radius):
        self.yaw_controller = yaw_controller
        self.speed_controller = speed_controller
        self.max_acc = max_acc
        self.max_dec = max_dec
        self.vehicle_mass = vehicle_mass
        self.wheel_radius = wheel_radius

    def control(self, current_twist, proposed_twist, dt):
        linear_velocity = proposed_twist.twist.linear.x
        angular_velocity = proposed_twist.twist.angular.z
        current_velocity = current_twist.twist.linear.x
        steer = self.yaw_controller.get_steering(linear_velocity, angular_velocity, current_velocity)
        desired_acc = (linear_velocity - current_velocity)/dt
        final_acc = self.speed_controller.step(desired_acc, dt)
        if final_acc >= 0:
            throttle = final_acc/self.max_acc
            brake = 0.0
        else:
            throttle = 0
            brake = final_acc/self.max_dec
        return throttle, brake, steer
