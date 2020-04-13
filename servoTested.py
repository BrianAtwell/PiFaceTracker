#!/usr/bin/env python

# Import the PCA9685 module.
import Adafruit_PCA9685

from curses import wrapper
from lxml import etree

pwm = Adafruit_PCA9685.PCA9685()

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600 # Max pulse length out of 4096
cur_servo_pulse = servo_min
last_servo_pulse = servo_min

pwm.set_pwm_freq(60)

c=0

def main(stdscr):
	
	stdscr.noecho()
	stdscr.cbreak()
	stdscr.keypad(True)
	
	# Clear screen
    stdscr.clear()
    
    pwm.set_pwm(0, 0, cur_servo_pulse)
    
	while c != curses.KEY_ESCAPE:
        c=stdscr.getch()
		
		if c == curses.KEY_UP:
			cur_servo_pulse+=10
		if c == curses.KEY_DOWN:
			cur_servo_pulse-=10
			
		if cur_servo_pulse != last_servo_pulse:
			if cur_servo_pulse < servo_min:
				cur_servo_pulse = servo_min
			if cur_servo_pulse > servo_max:
				cur_servo_pulse = servo_max
			pwm.set_pwm(0, 0, cur_servo_pulse)
			stdscr.addstr(0, 0, "Servo pulse"+str(cur_servo_pulse), curses.A_REVERSE)
            stdscr.refresh()
			last_servo_pulse=cur_servo_pulse
		
	pwm.set_pwm(0, 0, 0)
	
wrapper(main)