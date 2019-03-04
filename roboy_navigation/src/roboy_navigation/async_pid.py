"""
An asynchronous PID controller.
"""
import rospy

from simple_pid import PID
from threading import Thread

from roboy_navigation.srv import *


class AsyncPID:

    def __init__(self, target_val_provider, actual_val_provider,
                 control_callback, sample_rate, Kp=100, Kd=0.1, Ki=0.05):
        """

        :param target_val_provider: callable, returns the target value for the
                controller.
        :param actual_val_provider: callable, returns the actual value of the
                controller variable.
        :param control_callback: callable, accepts control signal.
        :param sample_rate: int, rate at which to run controller, in hertz.
        """
        self.setpoint_provider = target_val_provider
        self.input_provider = actual_val_provider
        self.control_callback = control_callback
        self.sample_rate = sample_rate
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki

    def start(self):
        rospy.Service('pid_config', PIDConfig, self.update_pid_config)
        Thread(target=self._run).run()

    def _run(self):
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown():
            setpoint = self.setpoint_provider()
            pid = PID(self.Kp, self.Ki, self.Kd, setpoint=setpoint)
            control = pid(self.input_provider())
            self.control_callback(control)
            rate.sleep()

    def update_pid_config(self, cfg):
        self.Kp = cfg.Kp
        self.Kd = cfg.Kd
        self.Ki = cfg.Ki
        rospy.loginfo('PID reconfigured (%.2f, %.2f %.2f)',
                      self.Kp, self.Kd, self.Ki)
        return PIDConfigResponse(True)