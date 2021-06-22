from adafruit_servokit import ServoKit
import time
kit = ServoKit (channels=16)
kit.servo[0].angle = 140
time.sleep(5)
kit.servo[0].angle = 0
