#!/usr/bin/python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

import threading, Queue

from steering_helper import MyoMuscleController, AngleSensorListener


class SteeringTesterThread(threading.Thread):

    def __init__(self, timeout=0.5):
        super(SteeringTesterThread, self).__init__()
        self.stop_request = threading.Event()
        self.timeout = timeout
        self.muscle_controller = MyoMuscleController(4, 7, 8)

    def start(self):
        self.cmd_listener = rospy.Subscriber(
            '/cmd_vel', Twist,
            callback=self.process_command,
            queue_size=10)
        self.muscle_controller.start()

    def process_command(self, twist):
        z = twist.angular.z
        if z < -1:
            self.muscle_controller.send_command(100, 10)
        elif z > 1:
            self.muscle_controller.send_command(10, 100)


class AnglePublisher(threading.Thread):

    def __init__(self, rate=20):
        super(AnglePublisher, self).__init__()
        self.stop_request = threading.Event()
        self.rate = rate
        self.angle_sensor_listener = AngleSensorListener()
        self.angle_sensor_listener.start()
        self.publisher = rospy.Publisher('/test_angle', Float64, queue_size=10)

    def run(self):
        rate = rospy.Rate(self.rate)
        while not self.stop_request.is_set():
            actual_angle = self.angle_sensor_listener.get_latest_actual_angle()
            self.publisher.publish(actual_angle)
            rate.sleep()

    def stop(self, timeout=None):
        self.stop_request.set()
        super(AnglePublisher, self).join(timeout)


if __name__ == '__main__':
    rospy.init_node('test_steering')
    cmd_q = Queue.Queue()
    angle_publisher = AnglePublisher()
    angle_publisher.run()
    tester = SteeringTesterThread()
    tester.start()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
    angle_publisher.stop()
