#---------Python 3.8.8
#Author : Venkatesh Sai
#Ts     : 12:05p.m / 04-04-2022
#© Copyrighted (2021) : Venkatesh Sai , Zuppa Geo Navigation Technologies Pvt. Ltd . Github Repo : f434a20243df049cc079d2ab2e5cf9b24a0170df
import numpy as np
import struct
class InputByteStream:
	buffer=[]
	pointer=0;
	def _init_(self):
		self.buffer=[]
		self.pointer=0;

	def assignDataBuffer(self,data):
		self.buffer=data;
		self.pointer=0;

	def clearBuffer(self):
		self.buffer=[];
		self.pointer=0;

	def getIntegerValue(self,len):
		ret=0;
		for i in range(len):
			ret|=(self.buffer[self.pointer]<<(i*8))
			self.pointer+=1
		return ret

	def getFloatValue(self):
		ret=0
		arr=bytearray()
		for i in range(4):
			val=self.buffer[self.pointer]
			self.pointer+=1
			arr.extend(np.byte(val))
		ret=struct.unpack('f',arr)
		return ret

	def bool(self):
		if(self.getIntegerValue(1)>0):
			return True
		else:
			return False

	def int8_t(self):
		return np.byte(self.getIntegerValue(1))

	def uint8_t(self):
		return np.ubyte(self.getIntegerValue(1))

	def int16_t(self):
		return np.short(self.getIntegerValue(2))

	def uint16_t(self):
		return np.ushort(self.getIntegerValue(2))

	def int32_t(self):
		return np.intc(self.getIntegerValue(4))

	def uint32_t(self):
		return np.uintc(self.getIntegerValue(4))

	def int64_t(self):
		return self.getIntegerValue(8)

	def uint64_t(self):
		return np.ulonglong(self.getIntegerValue(8))

	def float(self):
		return self.getFloatValue()[0]

class OutputByteStream:
	buffer=[]
	pointer=0;
	def _init_(self):
		self.buffer=[]
		self.pointer=0;

	def clearBuffer(self):
		self.buffer=[];
		self.pointer=0;

	def setIntegerValue(self,val,len):
		for i in range(len):
			self.buffer.append(chr(np.ubyte((val & (0xFF << (i*8)))>>(i*8))))
			self.pointer+=1
		return 1

	def setFloatValue(self,val):
		ret=0
		arr = bytearray(struct.pack("f", val))  
		for i in range(4):
			self.buffer.append(chr(arr[i]))
			self.pointer+=1
			ret=1
		return ret

	def bool(self,val):
		if(val>0):
			self.setIntegerValue(1,1)
		else:
			self.setIntegerValue(0,1)
		return 1

	def int8_t(self,val):
		return self.setIntegerValue(val,1);

	def uint8_t(self,val):
		return self.setIntegerValue(val,1);

	def int16_t(self,val):
		return self.setIntegerValue(val,2);

	def uint16_t(self,val):
		return self.setIntegerValue(val,2);

	def int32_t(self,val):
		return self.setIntegerValue(val,4);

	def uint32_t(self,val):
		return self.setIntegerValue(val,4);

	def int64_t(self,val):
		return self.setIntegerValue(val,8);

	def uint64_t(self,val):
		return self.setIntegerValue(val,8);

	def float(self,val):
		return self.setFloatValue(val)


class Stream:

	def __init__(self):
		self.opStream=OutputByteStream()
		self.inStream=InputByteStream()


'''

opStream=OuputByteStream()
opStream.clearBuffer();
opStream.uint8_t(5);
opStream.uint8_t(6);
opStream.uint8_t(7);
opStream.uint8_t(8);
opStream.int8_t(-5);
opStream.uint16_t(3000);
opStream.int16_t(-30768);
opStream.uint32_t(540023423);
opStream.int32_t(-1231231234325);
opStream.uint64_t(213124013912912093);
opStream.int64_t(-324);
opStream.float(51231.3421123);
print(opStream.buffer,opStream.pointer,ord(opStream.buffer[0]));

inStream=InputByteStream()
inStream.assignDataBuffer(opStream.buffer);
print(inStream.uint8_t());
print(inStream.uint8_t());
print(inStream.uint8_t());
print(inStream.uint8_t());
print(inStream.int8_t());
print(inStream.uint16_t());
print(inStream.int16_t());
print(inStream.uint32_t());
print(inStream.int32_t());
print(inStream.uint64_t());
print(inStream.int64_t());
print(inStream.float());

'''