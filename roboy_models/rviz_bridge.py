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

DEFAULT_MODEL_NAME = 'tricycle'
DEFAULT_WORLD_FRAME = '/map'
DEFAULT_MODEL_FRAME = 'base_link'

START_DELAY = 3 # sec

class RVIZBridge:

    def __init__(self):
        pass

    def start(self):
        rospy.init_node('gazebo_rviz_bridge',
                        log_level=rospy.DEBUG)
        self.get_params()
        rospy.sleep(START_DELAY)
        self.handle_initial_position()
        self.publish_model_position()
        rospy.spin()

    def get_params(self):
        self.name = rospy.get_name()
        self.model_name = rospy.get_param('%s/model_name' % self.name)
        self.model_frame = rospy.get_param('%s/model_frame' % self.name,
                                           default=DEFAULT_MODEL_FRAME)
        self.world_frame = rospy.get_param('%s/world_frame' % self.name,
                                           default=DEFAULT_WORLD_FRAME)

    def handle_initial_position(self):
        set_robot_pos = rospy.Publisher('/gazebo/set_model_state',
                                        ModelState, queue_size=1)

        def handle_initialpose(pose):
            rospy.loginfo('rviz_bridge.py: Resetting poisition of the %s',
                          self.model_name)
            pos = pose.pose.pose
            robot_state = ModelState()
            robot_state.model_name = self.model_name
            robot_state.pose = pos
            set_robot_pos.publish(robot_state)

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
                child=self.model_frame,
                parent=self.world_frame)
            # publish to 'odom' topic
            odom_msg = Odometry()
            odom_msg.header.stamp = curr_time
            odom_msg.header.frame_id = self.world_frame
            odom_msg.child_frame_id = self.model_frame
            odom_msg.pose.pose = position
            odom_msg.twist.twist = velocity
            odom_pub.publish(odom_msg)

        def publish_model_state(model_states):
            model_states = zip(model_states.name,
                               model_states.pose,
                               model_states.twist)
            for model_name, pose, twist in model_states:
                if model_name == self.model_name:
                    publish_odom(pose, twist)
                    return
            rospy.logerr('rviz_bridge.py: Could not find position and '
                         'velocity of %s',
                         self.model_name)

        rospy.Subscriber('/gazebo/model_states/', ModelStates,
                         publish_model_state, queue_size=10)


if __name__ == '__main__':
    RVIZBridge().start()
