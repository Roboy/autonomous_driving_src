#!/usr/bin/python
"""
A controller that listens to cmd_vel topics and forwards commands to the
rickshaw model running in gazebo. Model's definition can be found in
roboy_models/rickshaw/model.urdf
"""
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from math import pi

EPS = 1e-04
WHEEL_RADIUS = 0.2

class RickshawControlGazebo:

    def __init__(self):
        pass


    def start(self):
        rospy.init_node('rickshaw_control_gazebo', log_level=rospy.DEBUG)
        rear_wheel_pub = rospy.Publisher(
            '/rickshaw_rear_wheel_controller/command', Float64,
            queue_size=10)
        front_part_pub = rospy.Publisher(
            '/rickshaw_front_part_controller/command', Float64,
            queue_size=10)

        def handle_velocity(twist):
            vx, vy = twist.linear.x, twist.linear.y
            if vy > EPS:
                rospy.logerr('Non-holomonic velocity found vy=%.2f', vy)
            phi = twist.angular.z
            rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
            front_part_pub.publish(phi)
            wheel_p = 2.0 * WHEEL_RADIUS * pi
            wheel_rot_vel = vx / wheel_p
            rear_wheel_pub.publish(wheel_rot_vel)

        rospy.Subscriber('cmd_vel', Twist, handle_velocity, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    controller = RickshawControlGazebo()
    controller.start()
