# PiFaceTracker
Track a person face with OpenCV and a PID controller

## Description
This project was originally setup to use raspberry pi camera with a pan/tilt and some servos to track a person's face. A half of a ping pong ball is glued to the camera as an eyeball for a halloween prop. The idea was originally posted in Maker magizine for a cat tracker and there are other projects with the same concept.

In this project I also enabled controll of the pan/tilt of the camera through an Xbox 360 controller.

## Purpose
I wanted to learn more about OpenCV and develop my own PID controller. I wanted to make my own PID controller so I could understand more about for work. Using a PID controller for a couple of servos is low cost and low risk compared to other scenarios.

## Hardware
Prices as of April, 13, 2020
Item | Price | Link
------------- | --------- | --------
Raspberry Pi 2 | $40 | 
Mirco SD card | $12 | 
16-Channel PWM / Servo HAT for Raspberry Pi | $17.50 | https://www.adafruit.com/product/2327
M2.5 Stand offs for HAT 2x | $1.50 | https://www.adafruit.com/product/2336
Raspberry Pi Camera | $29.95 | https://www.adafruit.com/product/3099
Raspberry Pi Camera extra long cable | $3.95 | https://www.adafruit.com/product/2143
Pan/Tilt Camera Mount | $18.95 | https://www.adafruit.com/product/1967
Xbox 360 Controller | | 

Project Cost - $123.85


## Third Party Libraries
* 16-Channel PWM / Servo HAT Download Adafruit Library https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi
* OpenCV for Raspberry PI (I don't remeber how I set it up) but here is a https://www.learnopencv.com/install-opencv-4-on-raspberry-pi/
* Xbox360 Driver for python https://github.com/linusg/xbox360controller
* PiCamera Python Library
  * Enable Pi camera through raspi-conf command and then run the following two commands
  * sudo apt-get update
  * sudo apt-get install python-picamera python3-picamera
* Also looked into this Xbox360 driver https://github.com/FRC4564/Xbox
