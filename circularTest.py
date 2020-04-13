"""
File: circularTest.py
Author: Brian Atwell
Date: August 21, 2018
Description: This tests the functionality of circularTest

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


import circulararray
import sys

def printCount(obj):
	print("count {}".format(len(obj)))

def printFullList(obj):
	x = 0
	count = len(obj.array)-1
	sys.stdout.write("Full ")
	for curObj in obj.array:
		if count == x:
			print("[{}]{}".format(x,curObj))
		else:
			sys.stdout.write("[{}]{}, ".format(x,curObj))
		x+=1
		
def printList(obj):
	x = 0
	count = len(obj)-1
	sys.stdout.write(" ")
	for curObj in obj.generator():
		if count == x:
			print("[{}]{}".format(x,curObj))
		else:
			sys.stdout.write("[{}]{}, ".format(x,curObj))
		x+=1

carray=circulararray.CircularArray(10)


carray.append(1)
printList(carray)

carray.append(2)
carray.append(3)
carray.append(4)
carray.append(5)
carray.append(6)

printList(carray)
printFullList(carray)
printCount(carray)

carray.popFirst()
carray.popFirst()
carray.popFirst()
carray.popFirst()

printList(carray)
printFullList(carray)
printCount(carray)

carray.append(7)
carray.append(8)
carray.append(9)
carray.append(10)
carray.append(11)
carray.append(12)
carray.append(13)
#carray.append(14)
printList(carray)
printFullList(carray)
printCount(carray)

carray.append(14)
carray.popFirst()
carray.append(15)
printList(carray)
printFullList(carray)
printCount(carray)

carray.popFirst()
carray.append(15)

carray.popFirst()
carray.append(16)
printList(carray)
printFullList(carray)

carray.popFirst()
carray.append(17)
printList(carray)
printFullList(carray)

carray.popFirst()
carray.append(18)
printList(carray)
printFullList(carray)

carray.popFirst()
carray.append(19)
printList(carray)
printFullList(carray)

carray.popFirst()
carray.append(20)

printList(carray)
printFullList(carray)
printCount(carray)
