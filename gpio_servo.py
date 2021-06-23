#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

output_pin1 = 32
output_pin2 = 33

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(output_pin1, GPIO.OUT, initial=GPIO.HIGH)
    p1 = GPIO.PWM(output_pin1, 50)
    GPIO.setup(output_pin2, GPIO.OUT, initial=GPIO.HIGH)
    p2 = GPIO.PWM(output_pin2, 50)

    print("PWM running. Press CTRL+C to exit.")
    try:
        while True:
            p1.start(2.5)
            print("p1 start at 2.5%")
            time.sleep(0.5)
            p2.start(2.5)
            print("p2 start at 2.5%")
            time.sleep(0.5)
            p1.start(7.25)
            print("p1 start at 7.25%")
            time.sleep(0.25)
            p2.start(7.5)
            print("p2 start at 7.25%")
            time.sleep(0.5)
            p1.start(12)
            print("p1 start at 12%")
            time.sleep(0.5)
            p2.start(12)
            print("p2 start at 12%")
            time.sleep(0.5)
            p1.start(7.25)
            print("p1 start at 7.25%")
            time.sleep(0.5)
            p2.start(7.25)
            print("p2 start at 7.25%")
            time.sleep(0.5)
    finally:
        p1.stop()
        p2.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()