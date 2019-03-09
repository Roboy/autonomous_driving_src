#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys, termios, tty, os, time


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


button_delay = 0.05



class Demo:
    def __init__(self):
        rospy.init_node("demo_backup")
        self.pub_ad = rospy.Publisher('/ad_to_dr', String, queue_size=10)
        self.pub_dr = rospy.Publisher('/dr_to_ad', String, queue_size=10)
        self.continue_sending = False

    def run(self):
        pass
        #rospy.spin()


if __name__ == '__main__':
    d = Demo()
    print('Press 1-4 to send the following messages or press s to stop')
    print('1. pick_up_requested -> /dr_to_ad')
    print('2. arrived_at_pick_up_point -> /ad_to_dr')
    print('3. start_driving -> /dr_to_ad')
    print('4. arrived_at_drop_off_point -> /ad_to_dr\n')
    print('If the nav goals are failed to set, press:')
    print('a. set navgoal of pickup point')
    print('b. set navgoal of dropoff point\n')
    print('---------------------------------')
    d.run()

    while True:
        char = getch()

        if (char == "s"):
            print("Stop!")
            exit(0)

        elif (char == "1"):
            print("1. pick_up_requested -> /dr_to_ad")
            d.pub_dr.publish("pick_up_requested")
            time.sleep(button_delay)


        elif (char == "2"):
            print("2. arrived_at_pick_up_point -> /ad_to_dr")
            d.pub_ad.publish("arrived_at_pick_up_point")
            time.sleep(button_delay)

        elif (char == "3"):
            print("3. start_driving -> /dr_to_ad")
            d.pub_dr.publish("start_driving")
            time.sleep(button_delay)

        elif (char == "4"):
            print("4. arrived_at_drop_off_point -> /dr_to_dr")
            d.pub_ad.publish("arrived_at_drop_off_point")

        elif (char == "a"):
            print("Pickup navigation goal published")
            folder = os.path.dirname(os.path.realpath(__file__))
            os.system(folder + '/publish_navgoal1.sh')

        elif (char == "b"):
            print("Dropoff navigation goal published")
            folder = os.path.dirname(os.path.realpath(__file__))
            os.system(folder + '/publish_navgoal2.sh')

        elif (char == "c"):
            if d.continue_sending:
                print('Set continue_sending to FALSE')
                d.continue_sending = False
            else:
                print('Set continue_sending to TRUE')
                d.continue_sending= True


        else:
            print("unknown button pressed")

