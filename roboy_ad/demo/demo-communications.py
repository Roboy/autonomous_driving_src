#!/usr/bin/python
"""
THIS IS HOW THE DEMO WORKS:

1. "pick_up_requested" (/dr_to_ad): will be sent after Raf texts a 'Pick-me-up' message on the telegram channel
    -> Bike drives to hard-coded pickup point
    (will probably take 2-3 seconds until it starts to drive because planning takes some time after nav goal was set)
2. "arrived_at_pick_up_point" (/ad_to_dr): after bike stops at pickup point -> Raf gets on the bike
3. "start_driving" (/dr_to_ad): after Raf tells Roboy to drive to dropoff point -> Bike starts driving
4. "arrived_at_drop_off_point": after bike stops at pickup point -> roboy says 'We have arrived' and demo continues

Backup: if something goes wrong we manually set navigation goals and send messages
- navigation goal can be set in RVIZ
- manual sending of messages -> see demo-backup.py
    1. rostopic pub -1 /dr_to_ad std_msgs/String pick_up_requested
    2. rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_pick_up_point
    3. rostopic pub -1 /dr_to_ad std_msgs/String start_driving
    4. rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_drop_off_point

"""


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf
import time

MIN_DISTANCE = 0.5  # distance in meters to navigation goal so that the message is sent
RATE = 30.0
NAV_GOAL1 = [[47.2464027405, -50.3722496033, 0.0], [0.0, 0.0, -0.49633316537, 0.868132126438]]  # hard coded positions of the pickup and dropoff point
NAV_GOAL2 = [[66.0144729614, -76.4052886963, 0.0], [0.0, 0.0, -0.507478034911, 0.861664693534]]  # set position (3D euler) and rotation (4D quaternion)
PRINT_DISTANCE = True  # print distance for debugging
PRINT_DEBUG = False
PRINT_DISTANCE_N = 15  # only print distance every n iteration

SKIP_START = False


class Demo:
    def __init__(self):
        print("DEMO:\n"
              "1. 'pick_up_requested' receive from /dr_to_ad\n"
              "[NAV GOAL SET FOR PICKUP POINT]\n"
              "[WAIT UNTIL BIKE REACHES NAV GOAL]\n"
              "2. 'arrived_at_pick_up_point' send to /ad_to_dr\n"
              "[RAF GETS ON BIKE]\n"
              "[RAF TELLS ROBOY TO DRIVE TO DROPOFF POINT]\n"
              "3. 'start_driving' receive from /dr_to_ad\n"
              "[NAV GOAL SET FOR DROPOFF POINT]\n"
              "[WAIT UNTIL BIKE REACHES NAV GOAL]\n"
              "4. 'arrived_at_drop_off_point' send to /ad_to_dr\n"
              "[DEMO AD FINISHED. DIALOG REVOLUTION CONTINUES]\n")
        rospy.init_node("demo_roboy")
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.pub_communication = rospy.Publisher('/ad_to_dr', String, queue_size=10)
        rospy.Subscriber("/dr_to_ad", String, self.callback_DR_messages)
        self.tf_listener = tf.TransformListener()
        # variables
        self.monitoring_tf = False  # set to true once first msg was received; before that we don't need to monitor tf messages
        self.arrived_at_pickup = False
        self.nav_goal = NAV_GOAL1[0]  # already set first nav goal
        self.printdistance_i = 0
        if SKIP_START:
            self.monitoring_tf = True
            time.sleep(1)  # make sure first tf messages are received


    def callback_DR_messages(self, msg):
        if msg.data == 'pick_up_requested':
            self.move_to_pickup()
        elif msg.data == 'start_driving':
            self.move_to_dropoff()
        else:
            print('\n\n\n-------------------------------------------------------------------------------------------\n  '
                  'UNKNOWN MESSAGE RECEIVED: %s'
                  '-------------------------------------------------------------------------------------------\n\n\n' % msg)

    def set_nav_goal(self, ng):
        msg = PoseStamped()
        msg.pose = Pose(Point(ng[0][0], ng[0][1], ng[0][2]), Quaternion(ng[1][0], ng[1][1], ng[1][2], ng[1][3]))
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        self.pub_goal.publish(msg)

    def move_to_pickup(self):
        self.monitoring_tf = True
        print('1. pick_up_requested received: setting nav goal and start to drive to pickup')
        self.set_nav_goal(NAV_GOAL1)

    def move_to_dropoff(self):
        self.monitoring_tf = True
        print('3. start_driving received: setting nav goal and start to drive to dropoff')
        self.set_nav_goal(NAV_GOAL2)

    def check_if_goal_is_reached(self, translation):
        x1 = self.nav_goal[0]
        y1 = self.nav_goal[1]
        x2 = translation[0]
        y2 = translation[1]
        d = ((x1 - x2)**2 + (y1 - y2)**2)**0.5
        if PRINT_DISTANCE:
            self.printdistance_i += 1
            if self.printdistance_i == 15:
                print('Distance to nav goal: %f' % d)
                print('Translation map->base_link: %f, %f, %f' % (translation[0], translation[1], translation[2]))
                self.printdistance_i = 0
        if d < MIN_DISTANCE:
            print('\n\n\n-------------------------------------------------------------------------------------------\n'
                  'GOAL REACHED \n'
                   '-------------------------------------------------------------------------------------------\n\n\n')
            return True
        else:
            return False

    def run(self):
        try:
            rate = rospy.Rate(RATE)
            while not rospy.is_shutdown():
                if self.monitoring_tf:  # only need to monitor if we are driving at the moment
                    # get tf message of bike
                    try:
                        (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    except (tf.LookupException):
                        if PRINT_DEBUG:
                            print('tf exception: lookup (probably odom not published) -> continuing')
                        continue
                    except (tf.ConnectivityException, tf.ExtrapolationException):
                        print('tf exception:other -> continuing')
                        continue
                    # send messages when nav goal is reached
                    if self.check_if_goal_is_reached(trans):
                        # stage 1 arrive at pickup
                        if not self.arrived_at_pickup:
                            self.arrived_at_pickup = True
                            self.pub_communication.publish('arrived_at_pick_up_point')
                            print('2. arrived_at_pick_up_point sent: waiting for instructions to drive off again')
                            self.monitoring_tf = False
                            self.nav_goal = NAV_GOAL2[0]
                        # stage 2 arrive at dropoff
                        else:
                            self.pub_communication.publish('arrived_at_pick_up_point')
                            print('4. arrived_at_drop_off_point sent: AD demo has finished')
                            print('You can quit this script now')
                            self.monitoring_tf = False
                rate.sleep()
        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    d = Demo()
    print("Started: waiting for message pick_up_requested")
    d.run()
