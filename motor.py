import RPi.GPIO as GPIO
import time
output_pin = 18
GPIO.setmode(GPIO.BCM)  
GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
curr_value = GPIO.HIGH
GPIO.output(output_pin, curr_value)
curr_value = GPIO.HIGH
time.sleep(25)
GPIO.output(output_pin, GPIO.LOW)
GPIO.cleanup()
