#!/usr/bin/python
"""
1. "pick_up_requested" (/dr_to_ad): will be sent after Raf texts a 'Pick-me-up' message on the telegram channel
    -> Bike drives to hard-coded pickup point
    (will probably take 2-3 seconds until it starts to drive because planning takes some time after nav goal was set)
2. "arrived_at_pick_up_point" (/ad_to_dr): after bike stops at pickup point -> Raf gets on the bike
3. "start_driving" (/dr_to_ad): after Raf tells Roboy to drive to dropoff point -> Bike starts driving
4. "arrived_at_drop_off_point": after bike stops at pickup point -> roboy says 'We have arrived' and demo continues

Backup: if something goes wrong we manually set navigation goals and send messages
- navigation goal can be set in RVIZ
- manual sending of messages
    1. rostopic pub -1 /dr_to_ad std_msgs/String pick_up_requested
    2. rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_pick_up_point
    3. rostopic pub -1 /dr_to_ad std_msgs/String start_driving
    4. rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_drop_off_point

"""


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf

MIN_DISTANCE = 0.5  # distance in meters to navigation goal so that the message is sent
NAV_GOAL1 = [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]  # hard coded positions of the pickup and dropoff point #TODO
NAV_GOAL2 = [[2.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0]]  # set position (3D euler) and rotation (4D quaternion)
PRINT_DISTANCE = True  # print distance for debugging
global distance_i
distance_i = 0


global monitoring_tf, pub_goal  # need to be globalled because they are used in callbacks


def init():
    global pub_goal
    rospy.init_node("demo_roboy")
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pub_communication = rospy.Publisher('/ad_to_dr', String, queue_size=10)
    rospy.Subscriber("/dr_to_ad", String, callback_DR_messages)
    tf_listener = tf.TransformListener()
    return tf_listener, pub_goal, pub_communication


def callback_DR_messages(msg):
    if msg.data == 'pick_up_requested':
        move_to_pickup()
    elif msg.data == 'start_driving':
        move_to_dropoff()
    else:
        print('\n\n\n-------------------------------------------------------------------------------------------\n  '
              'UNKNOWN MESSAGE SENT: %s'
              '-------------------------------------------------------------------------------------------\n\n\n' % msg)


def set_nav_goal(ng):
    global pub_goal
    msg = PoseStamped()
    msg.pose = Pose(Point(ng[0][0], ng[0][1], ng[0][2]), Quaternion(ng[1][0], ng[1][1], ng[1][2], ng[1][3]))
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    pub_goal.publish(msg)


def move_to_pickup():
    global monitoring_tf
    monitoring_tf = True
    print('1. pick_up_requested received: setting nav goal and start to drive to pickup')
    set_nav_goal(NAV_GOAL1)


def move_to_dropoff():
    global monitoring_tf
    monitoring_tf = True
    print('3. start_driving received: setting nav goal and start to drive to dropoff')
    set_nav_goal(NAV_GOAL2)


def check_if_goal_is_reached(navgoal, translation):
    global distance_i
    x1 = navgoal[0]
    y1 = navgoal[1]
    x2 = translation[0]
    y2 = translation[1]
    d = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
    if PRINT_DISTANCE:
        distance_i += 1
        if distance_i == 15:
            print d
            distance_i = 0
    if d < MIN_DISTANCE:
        return True
    else:
        return False


if __name__ == '__main__':
    monitoring_tf = False  # set to true once first msg was received; before that we don't need to monitor tf messages
    arrived_at_pickup = False
    nav_goal = NAV_GOAL1[0] #already set first nav goal
    tf_listener, pub_goal, pub_communication = init()
    print("Started: waiting for message pick_up_requested")
    try:
        rate = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            if monitoring_tf:  # only need to monitor if we are driving at the moment
                # get tf message of bike
                try:
                    (trans, rot) = tf_listener.lookupTransform('/map', '/odom', rospy.Time(0))
                except (tf.LookupException):
                    print('tf exception: lookup (probably odom not published) -> continuing')
                    continue
                except (tf.ConnectivityException, tf.ExtrapolationException):
                    print('tf exception:other -> continuing')
                    continue
                # send messages when nav goal is reached
                if check_if_goal_is_reached(nav_goal, trans):
                    # stage 1 arrive at pickup
                    if not arrived_at_pickup:
                        arrived_at_pickup = True
                        pub_communication.publish('arrived_at_pick_up_point')
                        print('2. arrived_at_pick_up_point sent: waiting for instructions to drive off again')
                        monitoring_tf = False
                        nav_goal = NAV_GOAL2[0]
                    # stage 2 arrive at dropoff
                    else:
                        pub_communication.publish('arrived_at_pick_up_point')
                        print('4. arrived_at_drop_off_point sent: AD demo has finished')
                        print('You can quit this script now')
                        monitoring_tf = False
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
