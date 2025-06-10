import RPi.GPIO, time as GPIO, TIME

RED, YELLOW, GREEN = 21, 20, 16

GPIO.setmode(GPIO.BCM)
GPIO.setup(RED, GPIO.OUT)
GPIO.setup(YELLOW, GPIO.OUT)
GPIO.setup(GREEN, GPIO.OUT)

def INIT():
    GPIO.output([RED, YELLOW, GREEN], GPIO.LOW)

def GO():
    GPIO.output(GREEN, GPIO.HIGH)

def STOP():
    GPIO.output(RED, GPIO.HIGH)

def TURN():
    for _ in range(10):
        GPIO.output(YELLOW, GPIO.HIGH)
        TIME.sleep(0.5)
        GPIO.output(YELLOW, GPIO.LOW)
        TIME.sleep(0.5)

def PARK():
    for _ in range(10):
        GPIO.output([RED, YELLOW, GREEN], GPIO.HIGH)
        TIME.sleep(0.5)
        GPIO.output([RED, YELLOW, GREEN], GPIO.LOW)
        TIME.sleep(0.5)

    GPIO.cleanup()
