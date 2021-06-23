#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

output_pin2 = 33

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(output_pin2, GPIO.OUT, initial=GPIO.HIGH)
    p2 = GPIO.PWM(output_pin2, 50)

    print("PWM running. Press CTRL+C to exit.")
    p2.start(2.5)
    print("p2 start at 2.5%")
    time.sleep(0.25)
       #     p1.start(12)
          #  print("p1 start at 12%")
           # time.sleep(0.5)
            #p2.start(12)
            #print("p2 start at 12%")
            #time.sleep(0.5)


    p1.stop()
    p2.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    main()