import RPi.GPIO as GPIO
import time

# Pin Definitions
input_pin = 18  # BCM pin 18, BOARD pin 12

def main():
    prev_value = None

    start_time = time.time()
    last_time = time.time()

    # Pin Setup:
    GPIO.setmode(GPIO.BCM)  # BCM pin-numbering scheme from Raspberry Pi
    GPIO.setup(input_pin, GPIO.IN)  # set pin as an input pin
    try:
        while True:
            value = GPIO.input(input_pin)
            if value != prev_value:
                if value== GPIO.HIGH:
                    cur_time = time.time()
                    print("Detected at time", cur_time-start_time, "| Last detected", cur_time-last_time, "ago")
                    last_time = cur_time
                prev_value = value
            time.sleep(0.05)
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
