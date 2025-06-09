import RPi.GPIO as GPIO
import time

# GPIO pin setup
RED_PIN = 21
YELLOW_PIN = 20
GREEN_PIN = 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(RED_PIN, GPIO.OUT)
GPIO.setup(YELLOW_PIN, GPIO.OUT)
GPIO.setup(GREEN_PIN, GPIO.OUT)

def turn_off_all():
    GPIO.output([RED_PIN, YELLOW_PIN, GREEN_PIN], GPIO.LOW)

def blink_led(pin, interval=0.5, times=5):
    for _ in range(times):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(interval)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(interval)

def blink_all_leds(interval=0.5, times=3):
    for _ in range(times):
        GPIO.output([RED_PIN, YELLOW_PIN, GREEN_PIN], GPIO.HIGH)
        time.sleep(interval)
        GPIO.output([RED_PIN, YELLOW_PIN, GREEN_PIN], GPIO.LOW)
        time.sleep(interval)

try:
    print("LED Test Program (Press 'q' to quit)")
    print("1: RED ON | 2: GREEN ON | 3: YELLOW BLINK | 4: ALL BLINK")

    while True:
        key = input("Enter key (1-4 or q): ")

        turn_off_all()

        if key == '1':
            print("Red LED ON")
            GPIO.output(RED_PIN, GPIO.HIGH)

        elif key == '2':
            print("Green LED ON")
            GPIO.output(GREEN_PIN, GPIO.HIGH)

        elif key == '3':
            print("Yellow LED BLINK")
            blink_led(YELLOW_PIN)

        elif key == '4':
            print("All LEDs BLINK")
            blink_all_leds()

        elif key.lower() == 'q':
            print("Exiting program.")
            break

        else:
            print("Invalid input. Try again.")

finally:
    GPIO.cleanup()
    print("GPIO cleanup completed.")
