
"""This program handles the communication over I2C
between a Raspberry Pi and a MPU-6050 Gyroscope / Accelerometer combo.
Made by: MrTijn/Tijndagamer
Released under the MIT License
Copyright (c) 2015, 2016, 2017 MrTijn/Tijndagamer
"""

import smbus
import ctypes
import time
import numpy
import datetime

class mpu6050:

    #Initial Offsets
    x_accel_offset=0
    y_accel_offset=0
    z_accel_offset=0

    # Global Variables
    address = None
    bus = smbus.SMBus(1)

    # Scale Modifiers
    SCALE_MODIFIER_2G = 16384.0
    SCALE_MODIFIER_4G = 8192.0
    SCALE_MODIFIER_8G = 4096.0
    SCALE_MODIFIER_16G = 2048.0

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    # Accel Offset Modifiers
    ACCEL_X1 = 0x77
    ACCEL_X0 = 0x78
    ACCEL_Y1 = 0x7A
    ACCEL_Y0 = 0x7B
    ACCEL_Z1 = 0x7D
    ACCEL_Z0 = 0x7E


    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    ACCEL_CONFIG = 0x1C

    def __init__(self, address):
        self.address = address

        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.

        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def set_accel_offset(self):
	"""Sets the offset of the accelerometer values
        """
	#Write the new X acceleration offset
	self.bus.write_byte_data(self.address, self.ACCEL_X1, ctypes.c_int8(self.x_accel_offset >> 8).value)
	self.bus.write_byte_data(self.address, self.ACCEL_X0, ctypes.c_int8(self.x_accel_offset).value)
        #Write the new Y acceleration offset
	self.bus.write_byte_data(self.address, self.ACCEL_Y1, ctypes.c_int8(self.y_accel_offset >> 8).value)
	self.bus.write_byte_data(self.address, self.ACCEL_Y0, ctypes.c_int8(self.y_accel_offset).value)
        #Write the new X acceleration offset
	self.bus.write_byte_data(self.address, self.ACCEL_Z1, ctypes.c_int8(self.z_accel_offset >> 8).value)
	self.bus.write_byte_data(self.address, self.ACCEL_Z0, ctypes.c_int8(self.z_accel_offset).value)
	#Calibrating
	ox=0;oy=0;oz=0;
	ax1=0; ax2=0; ax3=0; ox1=0; ox2=0; ox3=0;
	ay1=0; ay2=0; ay3=0; oy1=0; oy2=0; oy3=0;
	az1=0; az2=0; az3=0; oz1=0; oz2=0; oz3=0;
	while ax1==0 or ax3==0 or ay1==0 or ay3==0 or az1==0 or az3==0:
		print("Initial Calibration")
		n=1000
		ax=0;ay=0;az=0;
		sx=0;sy=0;sz=0;
     		for i in range (0,n):
                        px=ax; py=ay; pz=az;
                        ax=mpu.read_i2c_word(mpu.ACCEL_XOUT0)
                        ay=mpu.read_i2c_word(mpu.ACCEL_YOUT0)
                        az=mpu.read_i2c_word(mpu.ACCEL_ZOUT0)
			sx=sx+ax;
			sy=sy+ay;
			sz=sz+az;
		ax=sx; ay=sy; az=sz;
		if (ax1==0 or ax3==0):
			if (ax<0):
				ox1=ox
				ax1=ax
				ox=ox+1000
			if (ax>0):
				ox3=ox
				ax3=ax
				ox=ox-1000
		if (ay1==0 or ay3==0):
			if (ay<0):
				oy1=oy
				ay1=ay
				oy=oy+1000
			if (ay>0):
				oy3=oy
				ay3=ay
				oy=oy-1000
		if (az1==0 or az3==0):
			if (az<0):
				oz1=oz
				az1=az
				oz=oz+1000
			if (az>0):
				oz3=oz
				az3=az
				oz=oz-1000
		#Write the new X acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_X1, ctypes.c_int8(ox >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_X0, ctypes.c_int8(ox).value)
        	#Write the new Y acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_Y1, ctypes.c_int8(oy >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_Y0, ctypes.c_int8(oy).value)
        	#Write the new X acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_Z1, ctypes.c_int8(oz >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_Z0, ctypes.c_int8(oz).value)

	cond1 = False
	cond2 = False
	cond3 = False
	while cond1==False or cond2==False or cond3==False:
		print("Sensitive Calibration")
		ox2=(ox1+ox3)//2
		oy2=(oy1+oy3)//2
		oz2=(oz1+oz3)//2
		cond1 = ox2==ox1 or ox2==ox3
		cond2 = oy2==oy1 or oy2==oy3
		cond3 = oz2==oz1 or oz2==oz3
		#Write the new X acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_X1, ctypes.c_int8(ox2 >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_X0, ctypes.c_int8(ox2).value)
        	#Write the new Y acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_Y1, ctypes.c_int8(oy2 >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_Y0, ctypes.c_int8(oy2).value)
        	#Write the new X acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_Z1, ctypes.c_int8(oz2 >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_Z0, ctypes.c_int8(oz2).value)
		ax=0;ay=0;az=0;
		sx=0;sy=0;sz=0;
		for i in range (0,n):
                        px=ax; py=ay; pz=az;
                        while (px==ax):
                                ax=mpu.read_i2c_word(mpu.ACCEL_XOUT0)
                        while (py==ay):
                                ay=mpu.read_i2c_word(mpu.ACCEL_YOUT0)
                        while (pz==az):
                                az=mpu.read_i2c_word(mpu.ACCEL_ZOUT0)
			sx=sx+ax;
			sy=sy+ay;
			sz=sz+az;
		ax2=sx; ay2=sy; az2=sz;
		if (cond1==False):
			if (ax2<0):
				ox1=ox2
				ax1=ax2
			if (ax2>0):
				ox3=ox2
				ax3=ax2
		if (cond2==False):
			if (ay2<0):
				oy1=oy2
				ay1=ay2
			if (ay2>0):
				oy3=oy2
				ay3=ay2
		if (cond3==False):
			if (az2<0):
				oz1=oz2
				az1=az2
			if (az2>0):
				oz3=oz2
				az3=az2

		#Write the new X acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_X1, ctypes.c_int8(ox2 >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_X0, ctypes.c_int8(ox2).value)
        	#Write the new Y acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_Y1, ctypes.c_int8(oy2 >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_Y0, ctypes.c_int8(oy2).value)
        	#Write the new X acceleration offset
		self.bus.write_byte_data(self.address, self.ACCEL_Z1, ctypes.c_int8(oz2 >> 8).value)
		self.bus.write_byte_data(self.address, self.ACCEL_Z0, ctypes.c_int8(oz2).value)

	#Write the new X acceleration offset
	self.bus.write_byte_data(self.address, self.ACCEL_X1, ctypes.c_int8(ox2 >> 8).value)
	self.bus.write_byte_data(self.address, self.ACCEL_X0, ctypes.c_int8(ox2).value)
        #Write the new Y acceleration offset
	self.bus.write_byte_data(self.address, self.ACCEL_Y1, ctypes.c_int8(oy2 >> 8).value)
	self.bus.write_byte_data(self.address, self.ACCEL_Y0, ctypes.c_int8(oy2).value)
        #Write the new X acceleration offset
	self.bus.write_byte_data(self.address, self.ACCEL_Z1, ctypes.c_int8(oz2 >> 8).value)
	self.bus.write_byte_data(self.address, self.ACCEL_Z0, ctypes.c_int8(oz2).value)



if __name__ == "__main__":
    mpu = mpu6050(0x68)
    mpu.set_accel_offset()
    print("Calibration Finished")


