#This py detects ball and returns distance. 
#In addition, it sends value of 
#1) boolean whether or not detected target ball 
#2) distance (meter) 
#3) rotational velocity (0-1 is magnitude) , (0,1, and 2 determines direction) 
#to roborio through analog output.
#====================================
# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import argparse
import cv2
import imutils
import time
import math
from lcd_display import lcd
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

# GPIO 25 is boolean output
# GPIO 8 is ball distance output
# GPIO 7 is rotational velocity output

GPIO.setup(25, GPIO.OUT)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(7, GPIO.OUT)

tbA = GPIO.PWM(25, 1000)
tbdA = GPIO.PWM(8, 1000)
tbrA = GPIO.PWM(7, 1000)

tbA.start(0)
tbdA.start(0)
tbrA.start(0) 

my_lcd = lcd()

text = 'Ball detected!'
global tb
tb = 0
#display = Blynk(auth_token, pin = "V0")

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
    help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
    help="max buffer size")
args = vars(ap.parse_args())
# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
Lower = (15, 115, 115)
Upper = (40, 255, 255)
# yellow: (15, 115, 115)/(40, 255, 255)
#orange: (10, 100, 20)/ (25, 255, 255)
#red: (10,255,255)/(180,255,255)
pts = deque(maxlen=args["buffer"])
# if a video path was not supplied, grab the reference
# to the webcam
if not args.get("video", False):
    vs = VideoStream(src=0).start()

# allow the camera or video file to warm up
time.sleep(2.0)
# this function converts radius to distance(meters)
# r = -0.976
# r^2 = 95,3%
def distance_radius_converter(rad):
    a = 52.75193624
    b = -0.9431246065
    distance = a* math.pow(rad,b)
    return distance
#this function determines the ball's direction(based on middle)
#and automatially adjust robot(provide rotation value)
# r = 0.984
# r^2 = 96%
def determine_direction(x):
    standrad_x = 340
    error = x - standrad_x
    a = 0.001905329
    b = 0.0364120782
    
    rotational_speed = a*error + b
    #when ball is exactly in middle
    if(x == standrad_x):
        rotational_speed = 0
        
    return rotational_speed
#function that display nesscary
#identites on the screen when a ball is detected

def draw_references(x,y,radius):
    cv2.circle(frame, (x, y), radius,
                (0, 0, 0), 3)
    cv2.putText(frame, text, (x + radius, y-43), cv2.FONT_HERSHEY_TRIPLEX,
  0.8, (255, 41, 30), 1, cv2.LINE_AA)
    cv2.putText(frame, "x: " + str(x) + ", y: " + str(y), (x + radius, y-24), cv2.FONT_HERSHEY_TRIPLEX,
  0.8, (255, 41, 30), 1, cv2.LINE_AA)
    cv2.putText(frame, " radius = " + str(radius), (x + radius, y-5), cv2.FONT_HERSHEY_TRIPLEX,
  0.8, (255, 41, 30), 1, cv2.LINE_AA)
    cv2.circle(frame, center, 5, (255, 0, 255), -1)

# display radius, rotational velocity, x val, and distance on lcd

def display_content(distance,radius,x,RS):
    distance = int(distance *100) / 100
    radius = int(radius*100)/100
    RS = int(RS*100)/100
    my_lcd.display_string("D:" + str(distance)+ "m,R:" + str(radius), 1)
    my_lcd.display_string("x:" + str(x) + ",RS:" + str(RS)  , 2)

# this function outputs analog signal to robot
# tb = 0 means no ball
# tb = 1 means ball on left side
# tb = 2 means ball on right side

def send_vals(rotation, distance):
    
    tbA.ChangeDutyCycle(tb)
    tbdA.ChangeDutyCycle(distance)
    tbrA.ChangeDutyCycle(abs(rotation))

# keep looping
while True:
    # grab the current frame
    frame = vs.read()
    # handle the frame from VideoCapture or VideoStream
    frame = frame[1] if args.get("video", False) else frame
    # if we are viewing a video and we did not grab a frame,
    # then we have reached the end of the video
    if frame is None:
        break
    # resize the frame, blur it, and convert it to the HSV
    # color space
    frame = imutils.resize(frame, width=800)
    frame = imutils.resize(frame, height=500)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # construct a mask for the color "yellow", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, Lower, Upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None
    tb = 0
    # only proceed if at least one contour was found
    if len(cnts) > 1:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            x = int(x)
            y = int(y)
            
            #draw_references(x,y,int(radius))
            #print("distance = " + str(distance_radius_converter(radius)) + "  m")
            display_content(distance_radius_converter(radius),radius,x,determine_direction(x))
            #print("(x,y): (" + str(x) + ", " + str(y) +")")
            #print("x: " + str(int(x)) + ", y: "
            # + str(int(y)) + ", radius: " + str(int(radius)))
            
            
            #tbr,tbd,tb sent to roborio
            tbr = determine_direction(x)
            tbd = distance_radius_converter(radius)
            if(tbr<0):
                tb = 1
            else:
                tb = 2
            send_vals(tbr,tbd)
            
    # update the points queue
    pts.appendleft(center)
    # loop over the set of tracked points
    for i in range(1, len(pts)):
        # if either of the tracked points are None, ignore
        # them
        if pts[i - 1] is None or pts[i] is None:
            continue
        # otherwise, compute the thickness of the line and
        # draw the connecting lines
        
    # show the frame to our sc
    #cv2.imshow("track ball", frame)
    key = cv2.waitKey(1) & 0xFF
    # if the escape key is pressed, stop the loop
    if key == 27:
        break
    time.sleep(0.01)
# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
    vs.stop()
# otherwise, release the camera
else:
    vs.release()
# close all windows
cv2.destroyAllWindows()