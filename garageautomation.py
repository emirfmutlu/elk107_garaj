#import and set GPIO library
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)

#set GPIO warnings off
GPIO.setwarnings(False)


#import delay function
import time

#import libraries which required on plate recognition
import cv2
import imutils
import numpy as np
import pytesseract
from PIL import Image

#define an integer variable to keep present pahase of automation
global automation_phase
automation_phase = 1
# phase=1 => arac disarda
# phase=2 => dis sensor araci gordu, plaka tanima çalisiyor
# phase=3 => kapi acildi, aracin iceri girmesi bekleniyor
# phase=4 => ic sensor araci gordu
# phase=5 => arac ic sensor hizasından çikti, kapi kapandi
# phase=6 => ic sensor araci tekrar gordu, kapı acildi
# phase=7 => dis sensor aracı gordu
# phase=8 / phase=1 => arac dis sensorun hizasından cikti, kapı kapandı

#define door servo motor signal pins
SERVO_LEFT_GPIO = 23
SERVO_RIGHT_GPIO = 24

#set motor pins
GPIO.setup(SERVO_LEFT_GPIO, GPIO.OUT)
GPIO.setup(SERVO_RIGHT_GPIO, GPIO.OUT)

#define servo angles
servo_left_close = 125
servo_left_open = 5
servo_right_close = 35
servo_right_open = 155

#converts the motor shaft angle (degrees) to pwm pulse percent
pulse_percent = lambda shaft_angle: 2.5 + shaft_angle / 18

#set gpio ports to generate pwm signal (50Hz) for the servo motors
servo_left = GPIO.PWM(SERVO_LEFT_GPIO, 50)
servo_right = GPIO.PWM(SERVO_RIGHT_GPIO, 50)

## NOTE: In our tests, servos were shaking too much.
## We measured width of the PWM signal by a logic analyzer.
## We figured out that PWM pulses are not stable;
## pulse widths were changing a lot between sequantial pulses.
## Since internal friction of our servo motors is capable to carry the door,
## program stops the PWM signal to prevent shaking.

#define servo sleep time delay in seconds
servo_sleep_delay = 1

#stop pwm signals of servo motors
def servo_stop_all():
    time.sleep(servo_sleep_delay)
    servo_left.ChangeDutyCycle(0)
    servo_right.ChangeDutyCycle(0)

#start pwm signal generation with initial values
servo_left.start(pulse_percent(servo_left_close))
servo_right.start(pulse_percent(servo_right_close))
servo_stop_all()

#control function of door servo motors
def door_open():
    servo_left.ChangeDutyCycle(pulse_percent(servo_left_open))
    servo_right.ChangeDutyCycle(pulse_percent(servo_right_open))
    servo_stop_all()
def door_close():
    servo_left.ChangeDutyCycle(pulse_percent(servo_left_close))
    servo_right.ChangeDutyCycle(pulse_percent(servo_right_close))
    servo_stop_all()

#define buzzer pin
BUZZER_GPIO = 26

#set buzzer pin
GPIO.setup(BUZZER_GPIO, GPIO.OUT)

#buzzer function
def buzzer_seconds(buzzer_duration):
    GPIO.output(BUZZER_GPIO, GPIO.HIGH)
    time.sleep(buzzer_duration)
    GPIO.output(BUZZER_GPIO, GPIO.LOW)

#plate recognition
def recognite_plate():
    ## NOTE: In this part of the code, we benefit from an article on circuitdigest.com
    ## https://circuitdigest.com/microcontroller-projects/license-plate-recognition-using-raspberry-pi-and-opencv

    #take photograph of the plate
    cam = cv2.VideoCapture(0)
    ret, img = cam.read()
    cam.release()
    
    img = cv2.resize(img, (620,480) )

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert to grey scale
    gray = cv2.bilateralFilter(gray, 11, 17, 17) #Blur to reduce noise
    edged = cv2.Canny(gray, 30, 200) #Perform Edge detection

    # find contours in the edged image, keep only the largest
    # ones, and initialize our screen contour
    cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    screenCnt = None

    # loop over our contours
    for c in cnts:
        # approximate the contour
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.018 * peri, True)
     
        # if our approximated contour has four points, then
        # we can assume that we have found our screen
        if len(approx) == 4:
            screenCnt = approx
            break

    if screenCnt is None:
        detected = 0
        print ("No contour detected")
    else:
        detected = 1

    if detected == 1:
        cv2.drawContours(img, [screenCnt], -1, (0, 255, 0), 3)

    # Masking the part other than the number plate
    mask = np.zeros(gray.shape,np.uint8)
    new_image = cv2.drawContours(mask,[screenCnt],0,255,-1,)
    new_image = cv2.bitwise_and(img,img,mask=mask)

    # Now crop
    (x, y) = np.where(mask == 255)
    (topx, topy) = (np.min(x), np.min(y))
    (bottomx, bottomy) = (np.max(x), np.max(y))
    Cropped = gray[topx:bottomx+1, topy:bottomy+1]

    #Read the number plate
    text = pytesseract.image_to_string(Cropped, config='--psm 8')
    print(text)

#define buzzer duration for phase transitions
buzzer_phase = 0.25

#execute actions
def action_exe(event):
    global automation_phase
    if automation_phase == 1 and event == 1:
        buzzer_seconds(buzzer_phase)
        print("vehicle detected, waiting for plate recognition module")
        automation_phase == 2
        ## call plate recognition
        ##
        print("plate approved, door opened")
        door_open()
        automation_phase=3
        ##
        
    elif automation_phase == 3 and event == 3:
        buzzer_seconds(buzzer_phase)
        print("vehicle started to enter")
        automation_phase = 4
    elif automation_phase == 4 and event == 4:
        buzzer_seconds(buzzer_phase)
        print("vehicle is fully entered to the garage, door closed")
        door_close()
        automation_phase = 5
    elif automation_phase == 5 and event == 3:
        buzzer_seconds(buzzer_phase)
        print("vehicle wants to get out, door opened")
        door_open()
        automation_phase = 6
    elif automation_phase == 6 and event == 1:
        buzzer_seconds(buzzer_phase)
        print("vehicle detected by outside sensor")
        automation_phase = 7
    elif automation_phase == 7 and event == 2:
        buzzer_seconds(buzzer_phase)
        print("vehicle fully leaved the garage, door closed")
        door_close()
        automation_phase = 1

#define sensor pins
OUTSIDE_SENSOR_GPIO = 5
INSIDE_SENSOR_GPIO = 6

#set sensor pins
GPIO.setup(OUTSIDE_SENSOR_GPIO, GPIO.IN)
GPIO.setup(INSIDE_SENSOR_GPIO, GPIO.IN)

#sensor output type is NPN
sensor_type = 1

#interrupt functions for sensor events
def outside_sensor_interrupt(channel):
    #outside sensor status changed
    if GPIO.input(OUTSIDE_SENSOR_GPIO) == sensor_type:
        #object detected / event-1
        action_exe(1)
    else:
        #object leaved the sensor range / event-2
        action_exe(2)
def inside_sensor_interrupt(channel):
    #inside sensor status changed
    if GPIO.input(INSIDE_SENSOR_GPIO) == sensor_type:
        #object detected / event-3
        action_exe(3)
    else:
        #object leaved the sensor range / event-4
        action_exe(4)

#set interrupts
GPIO.add_event_detect(OUTSIDE_SENSOR_GPIO, GPIO.BOTH, callback=outside_sensor_interrupt)
GPIO.add_event_detect(INSIDE_SENSOR_GPIO, GPIO.BOTH, callback=inside_sensor_interrupt)

print("program started")
recognite_plate()

