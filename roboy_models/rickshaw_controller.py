#!/usr/bin/python
"""
A controller that listens to cmd_vel topics and forwards commands to the
rickshaw model running in gazebo. Model's definition can be found in
roboy_models/rickshaw/model.urdf
"""
import rospy

from math import pi, atan
from simple_pid import PID

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray

from rickshaw_angle_sensor import RickshawSteeringSensor

EPS = 1e-04
WHEEL_RADIUS = 0.25
WHEEL_BASE = 1.6


def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)


class RickshawControlGazebo:

    def __init__(self):
        self.steering_sensor = RickshawSteeringSensor()

    def start(self):
        rospy.init_node('rickshaw_control_gazebo', log_level=rospy.DEBUG)
        self.steering_sensor.start()
        wheels_pub = rospy.Publisher(
            '/rickshaw_wheels_controller/command', Float64MultiArray,
            queue_size=10)
        front_part_pub = rospy.Publisher(
            '/rickshaw_front_part_controller/command', Float64,
            queue_size=10)
        #self.angle_sensor.start()
        def handle_velocity_command(twist):
            vx, vy = twist.linear.x, twist.linear.y
            if vy > EPS:
                rospy.logerr('Non-holomonic velocity found vy=%.2f', vy)
            phi = twist.angular.z
            steering_vel = self.get_steering_velocity(vx, phi)
            rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
            rospy.logdebug('Try setting steering angle to %.2f', phi)
            front_part_pub.publish(steering_vel)
            wheel_p = 2.0 * WHEEL_RADIUS * pi
            wheel_rot_vel = vx / wheel_p
            rospy.logdebug('Try setting wheels speed to %.2f', wheel_rot_vel)
            lin_vel = Float64MultiArray(data=[wheel_rot_vel, wheel_rot_vel,
                                              wheel_rot_vel])
            wheels_pub.publish(lin_vel)

        rospy.Subscriber('cmd_vel', Twist, handle_velocity_command, queue_size=1)
        rospy.spin()

    def get_steering_velocity(self, lin_vel, angular_vel):
        target_angle = convert_trans_rot_vel_to_steering_angle(
            lin_vel, angular_vel, WHEEL_BASE
        )
        pid = PID(1, 0.1, 0.05, setpoint=target_angle)
        steering_vel = pid(self.steering_sensor.get_latest_sensor_value())
        return steering_vel


if __name__ == '__main__':
    controller = RickshawControlGazebo()
    controller.start()
