#!/usr/bin/env python

'''
face detection using haar cascades with Pi camera.
Also provide changing the camera angle with a servo powered tilt/pan mount.
Use input from xbox joystick to control movement.

USAGE:
	facedetect.py [--cascade <cascade_fn>] [--nested-cascade <cascade_fn>] [<video_source>]
'''

# Python 2/3 compatibility
from __future__ import print_function
from __future__ import division

import numpy as np
import cv2
import time
import signal
#from xbox360controller import Xbox360Controller

# Import the PCA9685 module.
import Adafruit_PCA9685

# local modules
from video import create_capture
from common import clock, draw_str

#joy = Xbox360Controller(0, axis_threshold=0.2)

#joy.leftX()

class Point:
	""" Point class to manipulates x,y coords """
	
	def __init__(self, lx=0, ly=0):
		self.x = lx
		self.y = ly
		
	def rectToPoint(self, rect):
		print (rect)
		self.x = rect[0]+((rect[2] -rect[0])/2)
		self.y = rect[1]+((rect[3] - rect[1])/2)
	
	def distFromToCenter(self, rect):
		return
		


pwm=None

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600 # Max pulse length out of 4096
servo_range = servo_max-servo_min

MIN_FACE_POINTS=30


#pwm.set_pwm_freq(60)

oldFacePoints = []
facePoint = Point()

xServo = servo_min+((servo_max - servo_min)/2)
yServo = xServo

servoPoint = Point(servo_min+((servo_max - servo_min)/2), servo_min+((servo_max - servo_min)/2))
servoPrevPoint = Point(servo_min+((servo_max - servo_min)/2), servo_min+((servo_max - servo_min)/2))

servoSpeed = Point(3,3)

xServoID = 0
yServoID = 1

curRect=-1
rectStart=clock()
dt = 0
t=0

def rectsToPoint(rects, point):
	if len(rects) >= 1:
		curRect = 0
		point.rectToPoint(rects[0])
		"""if len(rects) == 1:
		curRect = 0
		point.rectToPoint(rects[0])
	elif len(rects) > curRect:
		rectDt = (rectStart - clock()) * 1000
		if rectDt > 10
			curRect=int(round(random.random*(len(rects)-1)))
		point.rectToPoint(rects[point])"""
	else:
		curRect=-1
	
def draw_point(img, point):
	cv2.circle(img, (int(point.x), int(point.y)), 5, (0,255,0), thickness=-1)
	
def initServos():
	pwm = Adafruit_PCA9685.PCA9685(address=0x41)
	pwm.set_pwm_freq(60)
	
def clamp(val, min, max):
	if val < min:
		val = min
	if val > max:
		val = max
	return val

"""	
def updateServoController():
	joyX = joy.axis_l.x()
	joyX*=servoSpeed.x*dt
	servoPoint.x += joyX
	
	joyY = joy.axis_l.y()
	joyY*=servoSpeed.y*dt
	servoPoint.y += joyY
	
	servoPoint.x = clamp(servoPoint.x, servo_min, servo_max)
	servoPoint.y = clamp(servoPoint.y, servo_min, servo_max)
	
	
def updateServo():
	if servoPrevPoint.x != servoPoint.x:
		pwm.set_pwm(xServoID, 0, servoPoint.x)
	if servoPrevPoint.y != servoPoint.y:
		pwm.set_pwm(yServoID, 0, servoPoint.y)
	servoPrevPoint.x = servoPoint.x
	servoPrevPoint.y = servoPoint.y
"""	

def detect(img, cascade):
	rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30),
									 flags=cv2.CASCADE_SCALE_IMAGE)
	if len(rects) == 0:
		return []
	rects[:,2:] += rects[:,:2]
	return rects

def draw_rects(img, rects, color):
	for x1, y1, x2, y2 in rects:
		cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

if __name__ == '__main__':
	import sys, getopt
	print(__doc__)

	args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade=', 'nested-cascade='])
	try:
		video_src = video_src[0]
	except:
		video_src = 0
	initServos()
	args = dict(args)
	cascade_fn = args.get('--cascade', "data/haarcascades/haarcascade_frontalface_alt.xml")
	nested_fn  = args.get('--nested-cascade', "data/haarcascades/haarcascade_eye.xml")

	cascade = cv2.CascadeClassifier(cascade_fn)
	nested = cv2.CascadeClassifier(nested_fn)

	cam = create_capture(video_src, fallback='synth:bg=data/lena.jpg:noise=0.05')
	
	isRunning=True

	while isRunning:
		t = clock()
		ret, img = cam.read()
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		gray = cv2.equalizeHist(gray)
		
		rects = detect(gray, cascade)
		vis = img.copy()
		draw_rects(vis, rects, (0, 255, 0))
		rectsToPoint(rects, facePoint)
		draw_point(vis, facePoint)
		if not nested.empty():
			for x1, y1, x2, y2 in rects:
				roi = gray[y1:y2, x1:x2]
				vis_roi = vis[y1:y2, x1:x2]
				subrects = detect(roi.copy(), nested)
				draw_rects(vis_roi, subrects, (255, 0, 0))

		draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
		cv2.imshow('facedetect', vis)
		
		"""
		updateServoController()
		updateServo()
		"""
		
		if cv2.waitKey(5) == 27:
			isRunning=False
		"""
		if joy.button_mode():
			isRunning = False
		"""
		dt = clock() - t
	cv2.destroyAllWindows()
