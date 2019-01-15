#!/usr/bin/python
"""
A controller that listens to cmd_vel topics and forwards commands to the
tricycle model running in gazebo. Model's definition can be found in
navigation/gazebo/tricycle.urdf
"""
import rospy
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.msg import ModelState
import random

#TODO: replace with parameter
MODEL_NAME = 'tricycle'

EPS = 1e-04

class TricycleControlGazebo:

    def __init__(self):
        pass


    def start(self):
        set_robot_pos = rospy.ServiceProxy('/gazebo/set_model_state', ModelState)
        rospy.init_node('tricycle_control_gazebo', log_level=rospy.DEBUG)
        rear_wheels_pub = rospy.Publisher(
            '/tricycle_rear_wheels_controller/command', Float64MultiArray,
            queue_size=10)
        front_wheel_pub = rospy.Publisher(
            '/tricycle_front_wheel_controller/command', Float64,
            queue_size=10)

        def handle_initialpose(pose):
            rospy.loginfo('Resetting poisition of the %s', MODEL_NAME)
            pos = pose.pose.pose
            robot_state = ModelState()
            robot_state.model_name = MODEL_NAME
            robot_state.pose = pos
            success, status_msg = set_robot_pos(robot_state)
            if not success:
                rospy.logerror('Position change failed. Error:\n %s', status_msg)


        def handle_velocity(twist):
            vx, vy = twist.linear.x, twist.linear.y
            if vy > EPS:
                rospy.logerror('Non-holomonic velocity found vy=%.2f', vy)
            phi = twist.angular.z
            rospy.logdebug('Command(vel=%.2f, steering=%.2f) received', vx, phi)
            front_wheel_pub.publish(phi)
            lin_vel = Float64MultiArray(data=[vx, vx])
            rear_wheels_pub.publish(lin_vel)

        rospy.Subscriber('cmd_vel', Twist, handle_velocity, queue_size=1)
        #rospy.Subscriber('initialpose', PoseWithCovarianceStamped,
        #                 handle_initialpose, queue_size=1)
        rospy.spin()

if __name__ == '__main__':
    controller = TricycleControlGazebo()
    controller.start()
