#!/usr/bin/env python

import Jetson.GPIO as GPIO
import time

output_pin2 = 33

def main():
    for i in range(5):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(output_pin2, GPIO.OUT, initial=GPIO.HIGH)
        p2 = GPIO.PWM(output_pin2, 50)
        p2.start(12)
        time.sleep(0.75)
        p2.start(2)
        time.sleep(0.75)

        p2.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
