#!/usr/bin/env python3
import rospy
import time
import board
import busio
from sensor_msgs.msg import Joy
from enum import Enum
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
    RIGHT_STICK_X = 2
    RIGHT_STICK_Y = 3
    RT = 4
    LT = 5
    DPAD_X = 6
    DPAD_Y = 7

class ArmTeleop:
    def __init__(self):
        rospy.init_node('arm_teleop', anonymous=True)

        # Initialize PCA9685 board for PWM control
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c, address=0x41)
        self.pca.frequency = 50  # 50Hz for servos

        # Define motor control channels (adjust as necessary)
        self.servos = {
            "base"     : 0,  # Base rotation
            "shoulder" : 1,  # Shoulder joint
            "elbow"    : 2,  # Elbow joint
            "wris1"    : 3,  # Wrist joint (pitch)
            "wrist2"   : 4,  # Second wrist joint (roll)
            "gripper"  : 5,  # Gripper open/close
        }

        # Initialize joystick states
        self.joy_axes = [0] * 8
        self.joy_buttons = [0] * 11

        # Subscribe to the joystick topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

    def set_servo(self, servo_name, value):
        """ Convert joystick value (-1 to 1) to PWM signal (0 to 4095) """
        channel = self.servos[servo_name]
        pwm_value = int((value + 1) / 2 * 1024)  # Map -1 to 1 range into 0-4095 # Adjusting speed for now
        self.pca.channels[channel].duty_cycle = pwm_value
        rospy.loginfo(f"Setting {servo_name} to {pwm_value}")

    def joy_callback(self, msg):
        """ Process joystick input and move servos accordingly """
        # debug statement until it's cleaned up
        rospy.loginfo(msg)
        self.joy_axes = msg.axes
        self.joy_buttons = msg.buttons

        # Control base rotation with left stick X-axis
        self.set_servo("base", self.joy_axes[XboxAxes.LEFT_STICK_X.value])

        # Shoulder up/down with left stick Y-axis
        self.set_servo("shoulder", self.joy_axes[XboxAxes.LEFT_STICK_Y.value])

        # Elbow movement with right stick Y-axis
        self.set_servo("elbow", self.joy_axes[XboxAxes.RIGHT_STICK_Y.value])

        # Wrist rotation with right stick X-axis
        self.set_servo("wrist2", self.joy_axes[XboxAxes.RIGHT_STICK_X.value])

        # TODO: Add support for wrist1

        # Gripper open/close with RT/LT triggers
        gripper_value = self.joy_axes[XboxAxes.RT.value] - self.joy_axes[XboxAxes.LT.value]
        self.set_servo("gripper", gripper_value)

        # Example: Using buttons for quick movements
        # if self.joy_buttons[XboxButtons.A.value]:
        #     self.set_servo("aux", 1)  # Example action
        # elif self.joy_buttons[XboxButtons.B.value]:
        #     self.set_servo("aux", -1)

if __name__ == "__main__":
    arm_teleop = ArmTeleop()
    rospy.spin()
