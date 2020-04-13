"""
File: pidcontroller.py
Author: Brian Atwell
Date: August 25, 2018
Descrioption: This implements a PID controller.


Copyright (c) 2018, Brian Atwell
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the Brian Atwell.
4. Neither Brian Atwell nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY Brian Atwell ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL Brian Atwell BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

class PIDController:

	def __init__(self):
		self.derivative=0.0
		self.integral=0.0
		self.proportional=0.0
		self.setPoint=0.0
		self.input=0.0
		self.output=0.0
		self.error=0.0
		self.prevError=0.0
		self.applyDeriv=0.0
		self.applyInt=0.0
		self.applyProp=0.0
		
	def setDerivative(self, newDeriv):
		self.derivative = newDeriv
	
	def setIntegral(self, newInt):
		self.integral = newInt
	
	def setProportional(self, newProp):
		self.proportional = newProp
		
	def setSetPoint(self, newPoint):
		self.setPoint = newPoint
		
	def reset(self, startPoint=0.0):
		self.input=0.0
		self.output=startPoint
		self.applyDeriv=0.0
		self.applyInt=0.0
		self.applyProp=0.0
		self.error=0.0
		self.prevError=0.0
		
	def initializeIntegral(self, startPoint=0.0):
		self.applyInt=startPoint/self.integral
	
	def update(self, input, dt=1):
		self.input=input
		self.error=self.setPoint - self.input
		self.applyInt=(self.applyInt + self.error) * dt
		self.applyDeriv=(self.error - self.prevError)/dt
		self.output = (self.proportional * self.error) + (self.integral * self.applyInt) + (self.derivative * self.applyDeriv)
		self.prevError=self.error
		
		return self.output
