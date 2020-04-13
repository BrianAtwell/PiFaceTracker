"""
File: circulararray.py
Author: Brian Atwell
Date: August 21, 2018
Descrioption: This a python implementation of a circular array using a list.
This implementation could be used as a queue or an array.


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

class  CircularArray:
	
	def __init__(self, size):
		self.array = []
		self.size = size
		self.startPos=0
		self.count=0
	
	def __getitem__(self, key):
		if key >= self.size:
			return None
		pos=(self.startPos+key)%self.size
		return self.array[pos]
			
	
	def __setitem__(self, idx, value):
		if idx > self.size:
			return
		if idx > (self.count+1) or self.count >= self.size:
			return
		if idx == (self.count+1):
			count+=1
		pos=(self.startPos+idx)%self.size
		self.array[pos] = value
		
	def __len__(self):
		return self.count
		
	def append(self, obj):
		if (self.count) >= self.size:
			return
		self.count+=1
		if len(self.array) < self.count or len(self.array) < self.size:
			self.array.append(obj)
		else:
			pos=(self.startPos+self.count-1)%self.size
			self.array[pos]=obj
	
	def clear(self):
		self.startPos=0
		self.endPos=0
	
	#def copy(self):
		
	def generator(self):
		idx = 0
		pos=self.startPos
		while idx < self.count:
			yield self.array[pos]
			pos=(pos+1)%self.size
			idx+=1
	
	#def extend(self):
	
	def index(self, value):
		pos = 0
		for obj in self.generator():
			if obj == value:
				return pos
			pos+=1
	
	
	def insert(self, idx, obj):
		curObj=obj
		nextObj=None
		pos=(self.startPos+idx)%self.size
		while idx <= self.count:
			pos=(pos+1)%self.size
			nextObj=self.array[pos]
			self.arry[pos]=curObj
			curObj = nextObj
			idx+=1
	
	def popFirst(self):
		if self.count == 0:
			return None
		retObj = self.array[self.startPos]
		self.startPos = (self.startPos+1) % self.size
		self.count -= 1
		return retObj
		
	def pop(self):
		if self.count == 0:
			return None
		pos = (self.startPos+self.count) % self.size
		retObj = self.array[pos]
		self.count-=1
		return retObj
	
	def size(self):
		return self.size
		
	def toList(self):
		retList = []
		for obj in self.generator():
			retList.append(obj)
		return retList