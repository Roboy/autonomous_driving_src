#!/usr/bin/python
from math import sqrt, sin, cos, pi, tan
import numpy as np
from collections import namedtuple

import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Point as Point3D, Quaternion, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

Position = namedtuple('Position', ['x', 'y', 'th', 'phi'])
Velocity = namedtuple('Velocity', ['x', 'y', 'th'])
Point = namedtuple('Point', ['x', 'y'])
Line = namedtuple('Line', ['p1', 'p2'])

GREEN = ColorRGBA(0.0, 1.0, 0.0, 1.0)

WORLD_FRAME='/map'
TRICYCLE_FRAME = 'tricycle_frame'

class TricycleViz:

    def __init__(self, base_len=3.0, rear_axis_len=2.0, steering_axis_len=1.8,
                 wheels=1.0):
        rospy.init_node('tricycle_viz')
        rospy.loginfo('Tricycle(base=%.2f,rear_axis=%.2f,steering_axis=%.2f,wheels=%.2f)',
                      base_len, rear_axis_len, steering_axis_len, wheels)
        self.base_len = base_len
        base_axis = Line(Point(0.0, 0.0), Point(base_len, 0.0))
        rear_axis = Line(Point(0.0, -rear_axis_len / 2), Point(0.0, rear_axis_len / 2))
        steering_axis = Line(Point(base_len, -steering_axis_len / 2), Point(base_len, steering_axis_len / 2))
        left_wheel_axis = Line(Point(-wheels / 2, -rear_axis_len / 2), Point(wheels / 2, -rear_axis_len / 2))
        right_wheel_axis = Line(Point(-wheels / 2, rear_axis_len / 2), Point(wheels / 2, rear_axis_len / 2))
        self.front_wheel_axis = Line(Point(-wheels / 2, 0), Point(wheels / 2, 0))
        self.tricycle_frame = [base_axis, rear_axis, steering_axis, left_wheel_axis, right_wheel_axis]

    def get_figure(self, position):
        x, y, th, phi = position
        steering_rot = np.array([[cos(phi), sin(phi)], [-sin(phi), cos(phi)]])
        steering_disp = np.array([self.base_len, 0])
        rot = np.array([[cos(th), sin(th)], [-sin(th), cos(th)]])
        disp = np.array([x, y])

        def transform(p, rot=rot, disp=disp):
            p_b = np.transpose(np.array([p.x, p.y]))
            p_s = np.matmul(rot, p_b) + np.transpose(disp)
            return Point(p_s[0], p_s[1])

        steering_axis = (Line(transform(self.front_wheel_axis.p1, rot=steering_rot, disp=steering_disp),
                              transform(self.front_wheel_axis.p2, rot=steering_rot, disp=steering_disp)))
        tricycle_frame = self.tricycle_frame + [steering_axis]
        return [Line(transform(line.p1), transform(line.p2)) for line in tricycle_frame]

    def run(self):
        rviz_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        marker = Marker()
        marker.header.frame_id = TRICYCLE_FRAME
        marker.ns = 'tricycle'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        #marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.color = GREEN
        marker.scale = Vector3(0.1, 0.1, 0.1)

        def update_position(odom_msg):
            if odom_msg.child_frame_id != TRICYCLE_FRAME:
                return
            pos = odom_msg.pose.pose.position
            quat = odom_msg.pose.pose.orientation
            angle = Vector3(*euler_from_quaternion((quat.x, quat.y, quat.z, quat.w)))
            rospy.logdebug('Position=(%.2f, %.2f) Orientation=(%.2f)', pos.x, pos.y, angle.z * 180 / pi)
            tricycle_pos = Position(pos.x, pos.y, angle.z, angle.z)
            marker.points = []
            for (p1, p2) in self.tricycle_frame:
                marker.points.append(Point3D(p1.x, p1.y, 0.0))
                marker.points.append(Point3D(p2.x, p2.y, 0.0))
            rviz_pub.publish(marker)

        rospy.Subscriber('/odom', Odometry, update_position, queue_size=1)
        rospy.spin()


if __name__ == '__main__':
    base = rospy.get_param('base', 1.5)
    rear_axis = rospy.get_param('rear_axis', 0.8)
    steering_axis = rospy.get_param('steering_axis', 0.6)
    wheels = rospy.get_param('wheels', 0.6)
    TricycleViz(base_len=base,
                rear_axis_len=rear_axis,
                steering_axis_len=steering_axis,
                wheels=wheels).run()