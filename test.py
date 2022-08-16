'''
                              |-------|
                              |       |
                              |   M1  |
                              |       |
                              |-------|
                                /  \
                               /    \
                              /      \
                             /        \
                            /          \
                           /            \
                          /              \
                         /                \
                        /                  \
                       /                    \
                      /                      \
                     /                        \
          |-------| /                          \  |-------|
          |       |/                            \ |       |
          |   M2  |-------------------------------|   M3  |
          |       |                               |       |
          |-------|                               |-------|
'''

# PS4 Controller Library
from pyPS4Controller.controller import Controller

# Raspi GPIO Library
import RPi.GPIO as GPIO
from time import sleep

# Pin Variables
loco_dir1_pin = 14
loco_dir2_pin = 15
loco_dir3_pin = 18
loco_pwm1_pin = 23
loco_pwm2_pin = 24
loco_pwm3_pin = 25

# Joystick Variables
global left_x 
global left_y
global right_x 

# Initializing Variables
left_x = 0 
left_y = 0
right_x = 0 

# Motor Direction and PWM Variables
global M1_pwm
global M2_pwm
global M3_pwm
global M1_dir
global M2_dir
global M3_dir

# Initializing Variables
M1_pwm = 0
M2_pwm = 0
M3_pwm = 0
M1_dir = 0
M2_dir = 0
M3_dir = 0

# Setup GPIO pins
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(loco_dir1_pin, GPIO.OUT)
GPIO.setup(loco_dir2_pin, GPIO.OUT)
GPIO.setup(loco_dir3_pin, GPIO.OUT)
GPIO.setup(loco_pwm1_pin, GPIO.OUT)
GPIO.setup(loco_pwm2_pin, GPIO.OUT)
GPIO.setup(loco_pwm3_pin, GPIO.OUT)

# Setup PWM
global pwm1
global pwm2
global pwm3

pwm1 = GPIO.PWM(loco_pwm1_pin, 5000)
pwm2 = GPIO.PWM(loco_pwm2_pin, 5000)
pwm3 = GPIO.PWM(loco_pwm3_pin, 5000)

# Start PWM
pwm1.start(0)
pwm2.start(0)
pwm3.start(0)

# Checking PS4 Connection
def connect():
    print("PS4 Connected Successfully...!!!")

# Checking PS4 Disconnection
def disconnect():
    print("PS4 Disconnected")

# Debug
global printValues
def printValues():
    print("left_x: ", left_x)
    print("left_y: ", left_y)
    print("right_x: ", right_x)
    print("M1_pwm: ", M1_pwm)
    print("M1_dir: ", M1_dir)
    print("M2_pwm: ", M2_pwm)
    print("M2_dir: ", M2_dir)
    print("M3_pwm: ", M3_pwm)
    print("M3_dir: ", M3_dir)

# Setup PS4 Controller
class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    # Constrain function
    global constrain
    def constrain(val, min_val, max_val):
        if val < min_val: 
            val = min_val
        elif val > max_val: 
            val = max_val
        return val

    # Variable Assignment
    global var_assign
    def var_assign():
        global left_x 
        global left_y
        global right_x

        global M1_pwm
        global M2_pwm
        global M3_pwm

    # Calculating Function
    global calculateLocValues
    def calculateLocValues(left_x, left_y, right_x):

        left_x = 3*left_x
        left_y = 3*left_y
        right_x = 3*right_x

        M1_pwm = (0.667*left_x + 0.333*right_x)*0.3921
        M2_pwm = (-0.333*left_x + 0.577*left_y + 0.333*right_x)*0.3921
        M3_pwm = (-0.333*left_x - 0.577*left_y + 0.333*right_x)*0.3921

        '''print("M1_pwm: ", M1_pwm)
        print("M2_pwm: ", M2_pwm)
        print("M3_pwm: ", M3_pwm)'''

        # Direction Deduction from mapped PWM values
        # DIRECTION = 0 for Forward, 1 for Reverse

        if M1_pwm > 10:
            M1_dir = 0
        elif M1_pwm < -10:
            M1_dir = 1
            M1_pwm = abs(M1_pwm)
        else:
            M1_pwm = 0
            M1_dir = 0

        if M2_pwm > 10:
            M2_dir = 0
        elif M2_pwm < -10:
            M2_dir = 1
            M2_pwm = abs(M2_pwm)
        else:
            M2_pwm = 0
            M2_dir = 0


        if M3_pwm > 10:
            M3_dir = 0
        elif M3_pwm < -10:
            M3_dir = 1
            M3_pwm = abs(M3_pwm)
        else:
            M3_pwm = 0
            M3_dir = 0


        M1_pwm = constrain(M1_pwm, 0, 100)
        M2_pwm = constrain(M2_pwm, 0, 100)
        M3_pwm = constrain(M3_pwm, 0, 100)

        print("M1_pwm: ", M1_pwm)
        print("M1_dir: ", M1_dir)
        GPIO.output(loco_dir1_pin, M1_dir)
        pwm1.ChangeDutyCycle(M1_pwm)
        #print("M1_pwm: ", M1_pwm)

        print("M2_pwm: ", M2_pwm)
        print("M2_dir: ", M2_dir)
        GPIO.output(loco_dir2_pin, M2_dir)
        pwm2.ChangeDutyCycle(M2_pwm)
        #print("M2_pwm: ", M2_pwm)

        print("M3_pwm: ", M3_pwm)
        print("M3_dir: ", M3_dir)
        GPIO.output(loco_dir3_pin, M3_dir)
        pwm3.ChangeDutyCycle(M3_pwm)
        #print("M3_pwm: ", M3_pwm)

    '''# Motors Driving Function
    global driveLocMotors
    def driveLocMotors():
        print("M1_pwm: ", M1_pwm)
        GPIO.output(loco_dir1_pin, M1_dir)
        pwm1.ChangeDutyCycle(M1_pwm)
        #print("M1_pwm: ", M1_pwm)

        print("M2_pwm: ", M2_pwm)
        GPIO.output(loco_dir2_pin, M2_dir)
        pwm2.ChangeDutyCycle(M2_pwm)
        #print("M2_pwm: ", M2_pwm)

        print("M3_pwm: ", M3_pwm)
        GPIO.output(loco_dir3_pin, M3_dir)
        pwm3.ChangeDutyCycle(M3_pwm)
        #print("M3_pwm: ", M3_pwm)'''

    def on_L3_up(self, value):
        var_assign()
        left_y = -0.00387 * value
        calculateLocValues(0, left_y, 0)
        #driveLocMotors()
        #print("on_L3_up: {}".format(left_y))
        #printValues()

    def on_L3_down(self, value):
        var_assign()
        left_y = -0.00387 * value
        calculateLocValues(0, left_y, 0)
        #driveLocMotors()
        #print("on_L3_down: {}".format(left_y))
        #printValues()

    def on_L3_left(self, value):
        var_assign
        left_x = 0.00387 * value
        calculateLocValues(left_x, 0, 0)
        #driveLocMotors()
        #print("on_L3_left: {}".format(left_x))
        #printValues()

    def on_L3_right(self, value):
        var_assign()
        left_x = 0.00387 * value
        calculateLocValues(left_x, 0, 0)
        #driveLocMotors()
        #print("on_L3_right: {}".format(left_x))
        #printValues()

    def on_R3_up(self, value):
        pass

    def on_R3_down(self, value):
        pass

    def on_R3_left(self, value):
        var_assign()
        right_x = 0.00387 * value
        calculateLocValues(0, 0, right_x)
        #driveLocMotors()
        #print("on_R3_left: {}".format(right_x))
        #printValues()

    def on_R3_right(self, value):
        var_assign()
        right_x = 0.00387 * value
        calculateLocValues(0, 0, right_x)
        #driveLocMotors()
        #print("on_R3_right: {}".format(right_x))
        #printValues()

controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(on_connect=connect, on_disconnect=disconnect)