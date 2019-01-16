#!/usr/bin/python
"""
A node that handles initial_position command that can be issued from RVIZ and
publishes model position to odom and tf topics.
"""

import rospy
import tf

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# TODO: replace with parameter
MODEL_NAME = 'tricycle'
WORLD_FRAME = '/map'
TRICYCLE_FRAME = 'base_link'


class RVIZBridge:

    def __init__(self):
        pass

    def start(self):
        rospy.init_node('gazebo_rviz_bridge',
                        log_level=rospy.DEBUG)
        self.handle_initial_position()
        self.publish_model_position()
        rospy.spin()

    def handle_initial_position(self):
        set_robot_pos = rospy.Publisher('/gazebo/set_model_state',
                                        ModelState, queue_size=1)

        def handle_initialpose(pose):
            rospy.loginfo('Resetting poisition of the %s', MODEL_NAME)
            pos = pose.pose.pose
            robot_state = ModelState()
            robot_state.model_name = MODEL_NAME
            robot_state.pose = pos
            success, status_msg = set_robot_pos.publish(robot_state)
            if not success:
                rospy.logerr('Position change failed. Error:\n %s',
                             status_msg)

        rospy.Subscriber('initialpose', PoseWithCovarianceStamped,
                         handle_initialpose, queue_size=1)

    def publish_model_position(self):
        odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        tf_broadcaster = tf.TransformBroadcaster()

        def publish_odom(position, velocity):
            curr_time = rospy.Time.now()
            # Publish to tf
            lin_pos = position.position
            quat = position.orientation
            tf_broadcaster.sendTransform(
                translation=(lin_pos.x, lin_pos.y, lin_pos.z),
                rotation=(quat.x, quat.y, quat.z, quat.w),
                time=curr_time,
                child=TRICYCLE_FRAME,
                parent=WORLD_FRAME)
            # publish to 'odom' topic
            odom_msg = Odometry()
            odom_msg.header.stamp = curr_time
            odom_msg.header.frame_id = WORLD_FRAME
            odom_msg.child_frame_id = TRICYCLE_FRAME
            odom_msg.pose.pose = position
            odom_msg.twist.twist = velocity
            odom_pub.publish(odom_msg)

        def publish_model_state(model_states):
            for model_name, pose, twist in zip(model_states.name,
                                               model_states.pose,
                                               model_states.twist):
                if model_name == MODEL_NAME:
                    publish_odom(pose, twist)
                    return
            rospy.logerr('Could not find position and velocity of %s',
                         MODEL_NAME)

        rospy.Subscriber('/gazebo/model_states/', ModelStates,
                         publish_model_state, queue_size=10)


if __name__ == '__main__':
    RVIZBridge().start()
