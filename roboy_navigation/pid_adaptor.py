#!/usr/bin/python

import rospy

from math import atan

from roboy_middleware_msgs.msg import MotorAngle
from geometry_msgs.msg import Twist


# Taken from http://docs.ros.org/kinetic/api/teb_local_planner/html/cmd__vel__to__ackermann__drive_8py_source.html
def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)


class PIDAdaptor:
    """
    Usage:
    adaptor = PIDAdaptor()
    adaptor.start()
    actual_angle = adaptor.get_latest_actual_angle()
    target_angle = adaptor.get_latest_target_angle()

    Parameters:

    wheel_base - double, distance between the wheels of the model.
    """
    def __init__(self):
        self.target_angle = 0
        self.actual_angle = 0

    def start(self):
        self.name = rospy.get_name()
        self.wheel_base = rospy.get_param('~wheel_base')
        self.listen_to_angle_sensor()
        self.listen_to_navigation_controller()
        rospy.loginfo('PIDAdaptor started')

    def listen_to_angle_sensor(self):

        def angle_receiver(raw_angle):
            # TODO(melkonyan): double-check the correct angle field.
            if len(raw_angle.raw_angles) != 1:
                rospy.logerr('Invalid motor_angle command received')
            angle = float(raw_angle.raw_angles[0]) / 4096 * 360
            self.target_angle = angle

        rospy.Subscriber('/roboy/middleware/StearingAngle', MotorAngle,
                         angle_receiver)

    def listen_to_navigation_controller(self):

        def navigation_commands_receiver(twist):
            angular_velocity = twist.angular.z
            linear_velocity = twist.linear.x
            self.target_angle = convert_trans_rot_vel_to_steering_angle(
                linear_velocity, angular_velocity, self.wheel_base
            )

        rospy.Subscriber('/cmd_vel', Twist, navigation_commands_receiver)

    def get_latest_target_angle(self):
        return self.target_angle

    def get_latest_actual_angle(self):
        return self.actual_angle


if __name__ == '__main__':
    rospy.init_node('pid_adaptor')
    adaptor = PIDAdaptor()
    adaptor.start()
    rospy.spin()