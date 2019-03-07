#!/usr/bin/python
from math import pi

import rospy

from roboy_navigation.async_pid import AsyncPID
from roboy_navigation.steering_helper import TargetAngleListener, \
    AngleSensorListener, MyoMuscleController, rad_to_deg

RELAXED_EFFORT = 10
MAX_EFFORT = 1000
MAX_STEERING_ANGLE = 10.0 / 180 * pi


class SteeringController:

    def __init__(self):
        self.angle_sensor_listener = AngleSensorListener()
        self.target_angle_listener = TargetAngleListener()
        self.muscle_controller = MyoMuscleController(
            fpga_id=4, left_motor_id=12, right_motor_id=10
        )
        self.async_pid = AsyncPID(
            target_val_provider=self.get_target_angle,
            actual_val_provider=self.get_actual_angle,
            control_callback=self.set_muscle_effort,
            sample_rate=100,
            Kp=50000.0,
            Kd=10000.0
        )

    def start(self):
        rospy.init_node('steering_controller')
        #TODO: refactor
        rospy.set_param('~wheel_base', 1.6)
        self.target_angle_listener.start()
        self.angle_sensor_listener.start()
        self.muscle_controller.start()
        self.async_pid.start()

    def get_target_angle(self):
        angle = self.target_angle_listener.get_latest_target_angle()
        return self.clip_bounds(angle)

    def get_actual_angle(self):
        return self.angle_sensor_listener.get_latest_smooth_angle()

    def set_muscle_effort(self, effort):
        effort_left, effort_right = (RELAXED_EFFORT, -effort) if effort < 0 \
            else (effort, RELAXED_EFFORT)
        effort_left = max(effort_left, RELAXED_EFFORT)
        effort_right = max(effort_right, RELAXED_EFFORT)
        effort_left = min(effort_left, MAX_EFFORT)
        effort_right = min(effort_right, MAX_EFFORT)
        print(effort)
        self.muscle_controller.send_command(effort_left, effort_right)

    def clip_bounds(self, angle):
        if -MAX_STEERING_ANGLE <= angle <= MAX_STEERING_ANGLE:
            return angle
        rospy.logwarn('steering_helper.py: Requested target angle=%.2f is not '
                      'in the admissible bounds (%.2f, %.2f)',
                      rad_to_deg(angle),
                      rad_to_deg(-MAX_STEERING_ANGLE),
                      rad_to_deg(MAX_STEERING_ANGLE)
                      )
        return min(max(angle, -MAX_STEERING_ANGLE), MAX_STEERING_ANGLE)


if __name__ == '__main__':
    SteeringController().start()
