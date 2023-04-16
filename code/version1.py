# Importing modules
import RPi.GPIO as GPIO#GPIO pin control library
import gpiozero#GPIO pin control library (simplifies procedures)
import time#clock functions
import numpy as np# array library
from picamera.array import PiRGBArray#pixel array from video in
from picamera import PiCamera#camera object
import cv2#computer vision library

#removes warning messages from GPIO pins
GPIO.setwarnings(False)

#broadcom numbering (GPIO pin)
GPIO.setmode(GPIO.BCM)

#Motors pin number init
#Ena is the enable PWM pin, allows the pulse rate to be modified (DC motor)
#In1, In2 are the two digital for the DC motor, they set the direction of the motor
#Servo is the PWM servo pin
Ena, In1, In2, Servo = 2,3,4,12

#Motors init pins. Initialized as outputs (can provide HIGH, LOW values and 1-100 for PWM pins)
GPIO.setup(Servo, GPIO.OUT)
GPIO.setup(Ena, GPIO.OUT)
GPIO.setup(In1, GPIO.OUT)
GPIO.setup(In2, GPIO.OUT)

#start PWM on the DC motor, set to an OFF, OFF state, rendering it not moving
pwm = GPIO.PWM(Ena, 75)
GPIO.output(In1, GPIO.LOW)
GPIO.output(In2, GPIO.LOW)
pwm.start(75)


#start servo, keeps the servo applying force to stay in certain direction
servo_pin = GPIO.PWM(12, 100)
servo_pin.start(12)

#init DC motor encoder (quadrature)
#encoder class takes care of reading, but the encoder works as specified below
#Square waves A,B are outputted. By comparing where the signals are HIGH, LOW on each square wave, a counterclockwise or a clockwise "tick" can be recorded
#By counting the ticks, we can determine the position, by checking the pattern, we can determine the direction
#This project soley uses the position of the encoder in order to find whether or not the launch angle is reached
encoder = gpiozero.RotaryEncoder(17, 18, max_steps=0)

#camera init (standard RPI camera module)
camera = PiCamera()

#camera dimensions
camera.resolution = (640, 480)
#framerate to capture from camera (as a top speed, the PI 3B+ cannot achieve this)
camera.framerate = 32

#the number of degrees of view the camera has. This comes into play when analyzing the launch angle
camera_degrees = 54

#create rawCapture object, used to take frames from camera
camera_width = 640
rawCapture = PiRGBArray(camera, size=(camera_width, 480))

#load HOG detection (human faces source)
#this was used over the HAAR database (more common) due the nature of data being analyzed. HOG is typically used for soley faces, while HAAR is used for entire bodies/other parts
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

#store coordinates of people (bounding boxes)
boxes = []

#pulses per revolution - datasheet said 120, but was calculated to be 110. Getting consistent, accurate results from this
pulses_per_revolution = 110
# time until allowed to launch, allowing motor to accelerate
accel_time = 0.5
# sleep each iteration of main loop, allows data to refresh
loop_time = 0.005  

# stores current angle, cummulative
angle_current = 0
#time at previous loop
t_prev = 0
#current time
t_curr = 0
t_start = time.perf_counter()#time since program started, significantly more accurate than time.clock

#stores motor's stopping point
releaseAngle = 0

#function to capture continuous frames from the camera and check for people
def cameraCapture():
    global releaseAngle
    
    #for each frame, checks for people
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        image = frame.array
        
        #converts the frame to grayscale, needed to use HAAR facial detection, HOR can work without
        #gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        #HOG outputs 
        boxes, weights = hog.detectMultiScale(image, winStride=(4,4))# winstride is the area searched at a time, lower = more accuracy
        
        #contains bounding boxes for faces
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])
        
        #draws a box around each face - comment out to increase speed
        for (xA, yA, xB, yB) in boxes:
            cv2.rectangle(image, (xA, yA), (xB, yB),(0, 255, 0), 2)
        cv2.imshow("Frame", image);
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        
        #prints bounding box x coordinate data if human found
        if (len(boxes) > 0):
            print("X coordinate: " + str(boxes[0]))
            break
        
    #calculates the release angle for the candy to be ejected from 
    x_point = boxes[0][0]
    releaseAngle =  ((x_point/camera_width*camera_degrees)-camera_degrees/2 )
    
    #makes release angle positive if not
    if releaseAngle < 0:
        releaseAngle = 360+ releaseAngle
    #prints the release angle
    print("Release angle: " + str(releaseAngle))
    
#opens servo
def openServo():
    servo_pin.ChangeDutyCycle(5)
    
#closes servo
def closeServo():
    servo_pin.ChangeDutyCycle(12)#pwm on servos changes the angle

#starts DCmotor
def startDC(pinA, pinB):
    GPIO.output(pinA, GPIO.LOW)
    GPIO.output(pinB, GPIO.HIGH)
    
#stops DC motor
def stopDC(pinA, pinB):
    GPIO.output(pinA, GPIO.LOW)
    GPIO.output(pinB, GPIO.LOW)

closeServo()
#starts camera capture. Once the camera is done obtaining the bounding box of a single person, the program continues
cameraCapture()

#current time. Used to calculate if the acclerate time is reached
timei = time.perf_counter() - t_start

#keeps track whether or not the skittle was released
not_shot = True

#starts DC motor 
startDC(In1, In2)

#continously updates encoder values and checks if the motor should brake and the servo should eject
while not_shot:
    #time DC motor has been running for 
    t_curr = time.perf_counter() - t_start
    
    #calculates angle the motor is at, cummulative
    
    angle_current = 360 / pulses_per_revolution * encoder.steps *-1
    
    print("Angle: " + str(angle_current))#comment out in headless, uses up space

    #if the candy should be released, this is true, 2 is the accuracy range. Unprofessional methods prove this to work
    if (abs(angle_current%360-releaseAngle) <=2 and t_curr - timei > accel_time):
        #release candy
        openServo()
         
        #stop DC motor
        stopDC(In1, In2)
        
        #candy released, will cause loop to end
        not_shot = False
    # delay 
    t_prev = t_curr
    time.sleep(loop_time)
    
#ensures motors are off, sometimes an exception causes the other one to not render
GPIO.output(In1, GPIO.LOW)
GPIO.output(In2, GPIO.LOW)
#delay until motors stopped
time.sleep(3)


