#!/usr/bin/env python3
import time
import board
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# Initialize PCA9685 board
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c, address=0x41)
pca.frequency = 50  # Set PWM frequency for servos

# Create servo objects for each channel
servos = [servo.Servo(pca.channels[i]) for i in range(16)]  # PCA9685 has 16 channels

def test_servos():
    for i, s in enumerate(servos):
        try:
            print(f"Testing Channel {i}...")
            s.angle = 90  # Move to a mid-range position
            time.sleep(1)  # Wait for movement
            s.angle = 0  # Move back
            time.sleep(1)
            s.angle = 180  # Move to the other extreme
            time.sleep(1)
        except ValueError:
            print(f"Channel {i} is inactive or not connected to a servo.")
        print("\n")

if __name__ == "__main__":
    test_servos()
    pca.deinit()  # Cleanup
