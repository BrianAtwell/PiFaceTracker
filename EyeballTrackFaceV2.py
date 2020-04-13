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
from picamera.array import PiRGBArray
from picamera import PiCamera
from xbox360controller import Xbox360Controller

from pidcontroller import PIDController

# Import the PCA9685 module.
import Adafruit_PCA9685

# local modules
from video import create_capture
from common import clock, draw_str

pidControllerXY = (PIDController(), PIDController())

isJoyPadConnected=True

try:
	joy = Xbox360Controller(0, axis_threshold=0.2)
except:
	isJoyPadConnected=False
	pass
print('is Joy Pad connected %d' % (isJoyPadConnected))
	
"""
Proportional: 0.05
Integral: 0.15
Derivative: 0.042
"""
"""
DefaultProportional= 0.05
DefaultIntegral=0.15
DefaultDerivative=0.042
"""
DefaultProportional=0.05
DefaultIntegral=0.03
DefaultDerivative=0.0011

#joy.leftX()

class Point:
	""" Point class to manipulates x,y coords """
	
	def __init__(self, lx=0, ly=0):
		self.x = lx
		self.y = ly
		
	def rectToPoint(self, rect):
		self.x = rect[0]+((rect[2] -rect[0])/2)
		self.y = rect[1]+((rect[3] - rect[1])/2)
	
	def distFromToCenter(self, rect):
		return
		


pwm=None

isRunning=True
isPIDMode=True
showCameraStream=False

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600 # Max pulse length out of 4096
servo_range = servo_max-servo_min

MIN_FACE_POINTS=30

CameraResolution = (640, 480)
pidOutput=Point(375, 375)

centerScreen = Point(CameraResolution[0]/2, CameraResolution[1]/2)


#pwm.set_pwm_freq(60)

oldFacePoints = []
facePoint = Point()

xServo = servo_min+((servo_max - servo_min)/2)
yServo = xServo

servoPoint = Point(int(servo_min+((servo_max - servo_min)/2)), int(servo_min+((servo_max - servo_min)/2)))
lastFacePoint=0
lastPIDUpdate=0
servoPrevPoint = Point(int(servo_min+((servo_max - servo_min)/2)), int(servo_min+((servo_max - servo_min)/2)))

servoSpeed = Point(10,10)

xServoID = int(1)
yServoID = int(0)



curRect=-1
rectStart=clock()
dt = 0
t=0

def exit_signal(signum, frame):
	global isRunning
	isRunning=False
	
signal.signal(signal.SIGINT, exit_signal)
signal.signal(signal.SIGTERM, exit_signal)

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
	pwm.set_pwm(xServoID, 0, servoPoint.x)
	pwm.set_pwm(yServoID, 0, servoPoint.y)
	return pwm
	
def clamp(val, min, max):
	if val < min:
		val = min
	if val > max:
		val = max
	return val
	
def updateServoController(facePoint):
	global isPIDMode
	global pidOutput
	global lastPIDUpdate
	global isJoyPadConnected

	faceDeltaTime = clock() - lastFacePoint
	PIDUpdateDeltaTime=clock()-lastPIDUpdate
	isChanged =False
	if isJoyPadConnected:
		if joy.button_b.is_pressed:
			isPIDMode=not isPIDMode
	pidOutput.x=servoPoint.x
	pidOutput.y=servoPoint.y
	if faceDeltaTime < 0.5 and isPIDMode: #and PIDUpdateDeltaTime > 1.0:
		pidOutput.x = pidControllerXY[0].update(facePoint.x)
		pidOutput.y = pidControllerXY[1].update(CameraResolution[1]-facePoint.y)
		clamp(pidOutput.x, servo_min, servo_max)
		clamp(pidOutput.y, servo_min, servo_max)
		if int(servoPoint.x) != int(pidOutput.x) or int(servoPoint.y) != int(pidOutput.y):
			isChanged=True
		servoPoint.x = pidOutput.x
		servoPoint.y = pidOutput.y
		lastPIDUpdate=clock()
	
	if isJoyPadConnected:
		joyX = joy.axis_l.x
		if joyX > 0.3 or joyX < -0.3:
			joyX*=servoSpeed.x*dt
			servoPoint.x += joyX
			servoPoint.x = clamp(servoPoint.x, servo_min, servo_max)
			isChanged=True
		
		joyY = joy.axis_l.y
		if joyY > 0.3 or joyY < -0.3:
			joyY*=servoSpeed.y*dt
			servoPoint.y += joyY
			servoPoint.y = clamp(servoPoint.y, servo_min, servo_max)
			isChanged=True
	"""
	if isChanged:
		print("axisx: {0} axisy: {1}",joy.axis_l.x, joy.axis_l.y)
		print("joyx: {0} joyy: {1}",joyX, joyY)
		print("sevoX: {0} servoY: {1}",servoPoint.x, servoPoint.y)
	"""
	
	
def updateServo():
	if servoPrevPoint.x != servoPoint.x:
		pwm.set_pwm(xServoID, 0, int(servoPoint.x))
	if servoPrevPoint.y != servoPoint.y:
		pwm.set_pwm(yServoID, 0, int(servoPoint.y))
	servoPrevPoint.x = servoPoint.x
	servoPrevPoint.y = servoPoint.y
	
def shutdownServo():
	pwm.set_pwm(xServoID, 0, int(servo_min+(servo_range/2)))
	pwm.set_pwm(yServoID, 0, int(servo_min+(servo_range/2)))
	pwm.set_pwm(xServoID, 0, 0)
	pwm.set_pwm(yServoID, 0, 0)
	

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
	pwm=initServos()
	args = dict(args)
	cascade_fn = args.get('--cascade', "data/haarcascades/haarcascade_frontalface_alt.xml")
	nested_fn  = args.get('--nested-cascade', "data/haarcascades/haarcascade_eye.xml")

	cascade = cv2.CascadeClassifier(cascade_fn)
	nested = cv2.CascadeClassifier(nested_fn)
	
	camera = PiCamera()
	camera.resolution = CameraResolution
	camera.framerate = 32
	rawCapture = PiRGBArray(camera, size=CameraResolution)
	lastPIDUpdate=clock()
	
	pidControllerXY[1].setProportional(DefaultProportional)
	pidControllerXY[1].setIntegral(DefaultIntegral)
	pidControllerXY[1].setDerivative(DefaultDerivative)
	pidControllerXY[1].reset(startPoint=centerScreen.y)
	pidControllerXY[1].initializeIntegral(startPoint=centerScreen.y)
	pidControllerXY[1].setSetPoint(centerScreen.y)
	pidControllerXY[1].applyInt=12417.0
	
	pidControllerXY[0].setProportional(DefaultProportional)
	pidControllerXY[0].setIntegral(DefaultIntegral)
	pidControllerXY[0].setDerivative(DefaultDerivative)
	pidControllerXY[0].reset(startPoint=centerScreen.x)
	pidControllerXY[0].initializeIntegral(startPoint=centerScreen.x)
	pidControllerXY[0].setSetPoint(centerScreen.x)
	pidControllerXY[0].applyInt=12417.0

	#cam = create_capture(video_src, fallback='synth:bg=data/lena.jpg:noise=0.05')

	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		t = clock()
		#ret, img = cam.read()
		img = frame.array
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		gray = cv2.equalizeHist(gray)
		
		rects = detect(gray, cascade)
		vis = img.copy()
		if len(rects) >= 1:
			rectsToPoint(rects, facePoint)
			if showCameraStream:
				draw_rects(vis, rects, (0, 255, 0))
				draw_point(vis, facePoint)
			if not nested.empty():
				for x1, y1, x2, y2 in rects:
					roi = gray[y1:y2, x1:x2]
					vis_roi = vis[y1:y2, x1:x2]
					subrects = detect(roi.copy(), nested)
					draw_rects(vis_roi, subrects, (255, 0, 0))
			lastFacePoint=clock()
		
		if showCameraStream:
			draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
			draw_str(vis, (20, 35), 'X: %.1f Y: %.1f' % (facePoint.x, facePoint.y))
			if isPIDMode:
				draw_str(vis, (20, 50), 'PIDMode pidOutput X: %.1f Y: %.1f' % (pidOutput.x, pidOutput.y))
				draw_str(vis, (20, 65), 'AppDeriv: %.4f AppInt: %.4f' % (pidControllerXY[1].applyDeriv, pidControllerXY[1].applyInt))
			cv2.imshow('facedetect', vis)
		
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
		
		updateServoController(facePoint)
		updateServo()
		
		c = cv2.waitKey(5)
		#Exit on Escape key
		if c == 27:
			print("key {0}".format(c))
			isRunning=False
		#Exit if back button is pressed
		if isJoyPadConnected:
			if joy.button_mode.is_pressed:
				isRunning = False
		if not isRunning:
			break
		dt = clock() - t
	shutdownServo()
	del pwm
	del rawCapture
	del camera
	cv2.destroyAllWindows()
	exit()
