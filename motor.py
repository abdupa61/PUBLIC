import RPi.GPIO as GPIO
import time
output_pin = 18 
def main():
    GPIO.setmode(GPIO.BCM)  
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
    curr_value = GPIO.HIGH
    try:
        while True:
            GPIO.output(output_pin, curr_value)
            curr_value = GPIO.HIGH
            time.sleep(5)
    finally:
        GPIO.cleanup()
if __name__ == '__main__':
    main()
