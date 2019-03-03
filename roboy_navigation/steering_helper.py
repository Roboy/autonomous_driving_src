"""
    Usage:
    target_angle_listener = TargetAngleListener()
    target_angle_listener.start()

    actual_angle_listener = AngleSensorListener()
    actual_angle_listener.start()

    actual_angle = actual_angle_listener.get_latest_actual_angle()
    target_angle = target_angle_listener.get_latest_target_angle()

    Parameters:
    wheel_base - double, distance between the wheels of the model.

"""
import rospy
from math import atan

from roboy_middleware_msgs.msg import MotorCommand, MotorAngle
from roboy_middleware_msgs.srv import ControlMode
from geometry_msgs.msg import Twist


MOTOR_CONTROL_POSITION = 0
MOTOR_CONTROL_VELOCITY = 1
MOTOR_CONTROL_DISPLACEMENT = 2

INIT_DISPLACEMENT = 10


# Taken from http://docs.ros.org/kinetic/api/teb_local_planner/html/cmd__vel__to__ackermann__drive_8py_source.html
def convert_trans_rot_vel_to_steering_angle(lin_vel, ang_vel, wheelbase):
    if ang_vel == 0 or lin_vel == 0:
        return 0

    radius = lin_vel / ang_vel
    return atan(wheelbase / radius)


class TargetAngleListener:

    def __init__(self):
        self.target_angle = 0
        self.wheel_base = rospy.get_param('~wheel_base')

    def start(self):
        self.listen_to_navigation_controller()

    def listen_to_navigation_controller(self):

        def navigation_commands_receiver(twist):
            angular_velocity = twist.angular.z
            linear_velocity = twist.linear.x
            self.target_angle = convert_trans_rot_vel_to_steering_angle(
                linear_velocity, angular_velocity, self.wheel_base
            )

        rospy.Subscriber('/cmd_vel', Twist, navigation_commands_receiver)

    def get_latest_target_angle(self):
        return self.target_angle


class AngleSensorListener:

    def __init__(self):
        self.actual_angle = 0

    def start(self):
        self.listen_to_angle_sensor()

    def listen_to_angle_sensor(self):

        def angle_receiver(raw_angle):
            # TODO(melkonyan): double-check the correct angle field.
            if len(raw_angle.raw_angles) != 1:
                rospy.logerr('Invalid motor_angle command received')
            angle = float(raw_angle.raw_angles[0]) / 4096 * 360
            self.target_angle = angle

        rospy.Subscriber('/roboy/middleware/StearingAngle', MotorAngle,
                         angle_receiver)

    def get_latest_actual_angle(self):
        return self.actual_angle


class MyoMuscleController:

    def __init__(self, fpga_id, left_motor_id, right_motor_id):
        self.fpga_id = fpga_id
        self.left_motor_id = left_motor_id
        self.right_motor_id = right_motor_id

    def start(self):
        self.publisher = rospy.Publisher('/roboy/middleware/MotorCommand',
                                         MotorCommand,
                                         queue_size=1)
        set_control_mode = rospy.ServiceProxy(
            '/roboy/shoulder_right/middleware/ControlMode',
            ControlMode)
        set_control_mode(MOTOR_CONTROL_DISPLACEMENT, INIT_DISPLACEMENT)

    def send_command(self, effort_left, effort_right):
        command = MotorCommand()
        command.id = self.fpga_id
        command.motors = [self.left_motor_id, self.right_motor_id]
        command.setPoints = [effort_left, effort_right]
        self.publisher.publish(command)