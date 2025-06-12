try:
    import RPi.GPIO as GPIO
    IS_RPI = True
except (RuntimeError, ModuleNotFoundError):
    GPIO = None
    IS_RPI = False

import time as TIME

RED, YELLOW, GREEN = 21, 20, 16

if IS_RPI:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RED, GPIO.OUT)
    GPIO.setup(YELLOW, GPIO.OUT)
    GPIO.setup(GREEN, GPIO.OUT)
else:
    print("WARNING: Not running on Raspberry Pi. GPIO functions disabled.")

def INIT():
    if IS_RPI:
        GPIO.output([RED, YELLOW, GREEN], GPIO.LOW)

def GO():
    if IS_RPI:
        GPIO.output(GREEN, GPIO.HIGH)
        GPIO.output([RED, YELLOW], GPIO.LOW)

def STOP():
    if IS_RPI:
        GPIO.output(RED, GPIO.HIGH)
        GPIO.output([GREEN, YELLOW], GPIO.LOW)

def TURN():
    if IS_RPI:
        for _ in range(10):
            GPIO.output(YELLOW, GPIO.HIGH)
            TIME.sleep(0.5)
            GPIO.output(YELLOW, GPIO.LOW)
            TIME.sleep(0.5)

def PARK():
    if IS_RPI:
        for _ in range(10):
            GPIO.output([RED, YELLOW, GREEN], GPIO.HIGH)
            TIME.sleep(0.5)
            GPIO.output([RED, YELLOW, GREEN], GPIO.LOW)
            TIME.sleep(0.5)
