#!/usr/bin/env python3
import rospy
import time
import board
import busio
from sensor_msgs.msg import Joy
from enum import Enum
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

class XboxButtons(Enum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    BACK = 6
    START = 7
    POWER = 8
    STICK_LEFT = 9
    STICK_RIGHT = 10

class XboxAxes(Enum):
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4
    RT = 5
    LT = 2
    DPAD_X = 6
    DPAD_Y = 7

class ArmTeleop:
    def __init__(self):
        rospy.init_node('arm_teleop', anonymous=True)

        # Initialize PCA9685 board for PWM control
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x41)
        self.pca.frequency = 50  # 50Hz for servos
        self.step_size = 5 # Incremental step size for each motor

        # Define motor control channels (adjust as necessary)
        self.servos = {
            "base"     : servo.Servo(self.pca.channels[0]),  # Base rotation
            "shoulder" : servo.Servo(self.pca.channels[1]),  # Shoulder joint
            "elbow"    : servo.Servo(self.pca.channels[2]),  # Elbow joint
            "wrist1"   : servo.Servo(self.pca.channels[3]),  # Wrist joint (pitch)
            "wrist2"   : servo.Servo(self.pca.channels[4]),  # Second wrist joint (roll)
            "gripper"  : servo.Servo(self.pca.channels[5]),  # Gripper open/close
        }

        # Initialize joystick states
        self.joy_axes = [0] * 8
        self.joy_buttons = [0] * 11

        # Subscribe to the joystick topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

    def set_servo(self, servo_name, value):
        """ Convert joystick input (-1 to 1) into gradual servo movement """
        servo = self.servos[servo_name]

        if servo.angle is None:  # Ensure the servo has an initial position
            servo.angle = 90  # Start at neutral position

        # Calculate new angle based on joystick input
        new_angle = servo.angle + (self.step_size * value)

        # Clamp the angle to valid servo range (0-180)
        new_angle = max(0, min(180, new_angle))

        # Set new angle
        if abs(new_angle - servo.angle) > 0.1:  # Avoid redundant updates
            servo.angle = new_angle
            rospy.loginfo(f"Setting {servo_name} angle to {servo.angle}")

    def joy_callback(self, msg):
        """ Process joystick input and move servos accordingly """
        self.joy_axes = msg.axes
        self.joy_buttons = msg.buttons

        # Control base rotation with left stick X-axis
        self.set_servo("base", self.joy_axes[XboxAxes.LEFT_STICK_X.value])

        # Shoulder up/down with left stick Y-axis
        self.set_servo("shoulder", self.joy_axes[XboxAxes.LEFT_STICK_Y.value])

        # Elbow movement with right stick Y-axis
        self.set_servo("elbow", self.joy_axes[XboxAxes.RIGHT_STICK_Y.value])

        # Wrist rotation with right stick X-axis
        self.set_servo("wrist1", self.joy_axes[XboxAxes.RIGHT_STICK_X.value])

        # Wrist rotation with right stick X-axis
        self.set_servo("wrist2", self.joy_axes[XboxAxes.RIGHT_STICK_X.value])

        # Gripper open/close with RT/LT triggers
        gripper_value = self.joy_axes[XboxAxes.RT.value] - self.joy_axes[XboxAxes.LT.value]
        self.set_servo("gripper", gripper_value)


if __name__ == "__main__":
    arm_teleop = ArmTeleop()
    rospy.spin()
