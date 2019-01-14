#!/usr/bin/python
"""
Custom simulation of a tricycle. It's a ROS node that listens to the cmd_vel
topic, updates tricycle position by integrating commanded velocity over time
and publishes current position to the odom and tf topics.
"""
from math import sqrt, sin, cos, pi, tan
from collections import namedtuple
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Vector3, Point as Point3D, Quaternion, Twist, \
    PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from navigation.msg import Controls
import time

Position = namedtuple('Position', ['x', 'y', 'th', 'phi'])
Velocity = namedtuple('Velocity', ['x', 'y', 'th'])

WORLD_FRAME = '/map'
TRICYCLE_FRAME = 'tricycle_frame'

last_update = time.time()


class TricycleSim:
    def __init__(self, init_pos, init_vel=Velocity(0, 0, 0), base_len=2.0):
        self.position = init_pos
        self.velocity = init_vel
        self.base_len = base_len

    def update_pos(self, controls, dt):
        vel, steering = controls
        x, y, th, _ = self.position
        vel_x = vel * cos(th)
        vel_y = vel * sin(th)
        vel_th = vel / self.base_len * tan(steering)
        self.velocity = Velocity(vel_x, vel_y, vel_th)
        self.position = Position(x + vel_x * dt, y + vel_y * dt,
                                 th + vel_th * dt, steering)

    def update_pos_no_constraints(self, controls, dt):
        vel, steering = controls
        x, y, th, _ = self.position
        vel_x = vel * cos(th)
        vel_y = vel * sin(th)
        vel_th = steering
        self.velocity = Velocity(vel_x, vel_y, vel_th)
        self.position = Position(x + vel_x * dt, y + vel_y * dt,
                                 th + vel_th * dt, steering)


def engine():
    rospy.init_node('tricycle_sim', log_level=rospy.DEBUG)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
    tf_broadcaster = tf.TransformBroadcaster()
    # tricycle = TricycleSim(init_pos=Position(190, 10, 0, 0), base_len=1.5)
    tricycle = TricycleSim(init_pos=Position(4, 4, 0, 0), base_len=1.5)

    def publish_odometry(position, velocity):
        curr_time = rospy.Time.now()
        odom_quat = quaternion_from_euler(0.0, 0.0, position.th)
        # Publish to tf
        tf_broadcaster.sendTransform(
            (position.x, position.y, 0.0),
            rotation=odom_quat,
            time=curr_time,
            child=TRICYCLE_FRAME,
            parent=WORLD_FRAME)
        # publish to 'odom' topic
        odom_msg = Odometry()
        odom_msg.header.stamp = curr_time
        odom_msg.header.frame_id = WORLD_FRAME
        odom_msg.child_frame_id = TRICYCLE_FRAME
        odom_msg.pose.pose.position = Point3D(position.x, position.y, 0.0)
        odom_msg.pose.pose.orientation = Quaternion(odom_quat[0], odom_quat[1],
                                                    odom_quat[2], odom_quat[3])
        odom_msg.twist.twist.linear = Vector3(velocity.x, velocity.y, 0)
        odom_msg.twist.twist.angular = Vector3(0, 0, velocity.th)
        odom_pub.publish(odom_msg)

    def handle_velocity(twist):
        vx, vy = twist.linear.x, twist.linear.y
        phi = twist.angular.z
        # TODO(melkonyan): create a custom controller
        controls = Controls(sqrt(vx ** 2 + vy ** 2), phi)
        rospy.logdebug('Command(vel=%.2f, steering=%.2f) received',
                       controls.vel, controls.steering)
        curr_time = time.time()
        global last_update
        dt = min(curr_time - last_update, 1)
        last_update = curr_time
        tricycle.update_pos((controls.vel, controls.steering), dt)

    def handle_initialpose(pose):
        pos = pose.pose.pose.position
        orientation = pose.pose.pose.orientation
        theta = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])[-1]
        rospy.loginfo('Changing position from %s to %s', tricycle.position, pos)
        tricycle.velocity = Velocity(0.0, 0.0, 0.0)
        tricycle.position = Position(pos.x, pos.y, theta, theta)

    publish_odometry(tricycle.position,
                     tricycle.velocity)  # reset bike to default pos
    # rospy.Subscriber('robike/controls', Controls, handle_controls, queue_size=1)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped,
                     handle_initialpose, queue_size=1)
    rospy.Subscriber('cmd_vel', Twist, handle_velocity, queue_size=1)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        publish_odometry(tricycle.position, tricycle.velocity)
        rate.sleep()


if __name__ == '__main__':
    engine()
