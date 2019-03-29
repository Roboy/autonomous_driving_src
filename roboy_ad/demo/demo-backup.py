#!/usr/bin/python3
'''
This script enables sending the relevant messages to team dialog revolution for the demo. This should work as a backup in case the normal script (demo-communications.py) does not work. Then you can send the messages with the press of a button

EDIT: A second version of the script was made because DR works with ROS bridge because they are on ROS 2 and apparently it does not support receiving messages that were sent via the 'rostopic pub' CLI command
'''


 
# adapted from https://github.com/recantha/EduKit3-RC-Keyboard/blob/master/rc_keyboard.py
 
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
 
button_delay = 0.2

print('Press 1-4 to send the following messages or press s to stop')
print('1. pick_up_requested -> /dr_to_ad')
print('2. arrived_at_pick_up_point -> /ad_to_dr')
print('3. start_driving -> /dr_to_ad')
print('4. arrived_at_drop_off_point -> /ad_to_dr\n')
print('If the nav goals are failed to set, press:')
print('a. set navgoal of pickup point')
print('b. set navgoal of dropoff point\n')
print('---------------------------------')
 
while True:
    char = getch()
 
    if (char == "s"):
        print("Stop!")
        exit(0)
 
    elif (char == "1"):
        print("1. rostopic pub -1 /dr_to_ad std_msgs/String pick_up_requested")
        os.system('rostopic pub -1 /dr_to_ad std_msgs/String pick_up_requested')
        time.sleep(button_delay)


    elif (char == "2"):
        print("2: rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_pick_up_point")
        os.system('rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_pick_up_point')
        time.sleep(button_delay)

    elif (char == "3"):
        print("3: rostopic pub -1 /dr_to_ad std_msgs/String start_driving")
        os.system('rostopic pub -1 /dr_to_ad std_msgs/String start_driving')
        time.sleep(button_delay)

    elif (char == "4"):
        print("4: rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_drop_off_point")
        os.system('rostopic pub -1 /ad_to_dr std_msgs/String arrived_at_drop_off_point')
        time.sleep(button_delay)

    elif (char == "a"):
        print("Pickup navigation goal published")
        folder = os.path.dirname(os.path.realpath(__file__))
        os.system(folder + '/publish_navgoal1.sh')

    elif (char == "b"):
        print("Dropoff navigation goal published")
        folder = os.path.dirname(os.path.realpath(__file__))
        os.system(folder + '/publish_navgoal2.sh')

    else:
        print("unknown button pressed")
