#!/usr/bin/python3
 
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
        os.system('./publish_navgoal1.sh')

    elif (char == "b"):
	print("Dropoff navigation goal published")
        os.system('./publish_navgoal2.sh')	

    else:
        print("unknown button pressed")
