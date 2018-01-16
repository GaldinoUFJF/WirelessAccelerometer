
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
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = smbus.SMBus(1)

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    # Accel Offset Modifiers
    ACCEL_X1 = 0x06
    ACCEL_X0 = 0x07
    ACCEL_Y1 = 0x08
    ACCEL_Y0 = 0x09
    ACCEL_Z1 = 0x0A
    ACCEL_Z0 = 0x0B


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

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.

        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formule given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.

        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)


    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to.

        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

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
		print("Calibragem Inicial")
		n=1000
		ax=0;ay=0;az=0;
     		for i in range (0,n):
			ax=ax+mpu.read_i2c_word(mpu.ACCEL_XOUT0)
			ay=ay+mpu.read_i2c_word(mpu.ACCEL_YOUT0)
			az=az+mpu.read_i2c_word(mpu.ACCEL_ZOUT0)
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
		print("Calibragem Fina")
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
		ax2=0;ay2=0;az2=0;
		for i in range (0,n):
			ax2=ax2+mpu.read_i2c_word(mpu.ACCEL_XOUT0)
			ay2=ay2+mpu.read_i2c_word(mpu.ACCEL_YOUT0)
			az2=az2+mpu.read_i2c_word(mpu.ACCEL_ZOUT0)
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


    def get_accel_data(self, g = True):
        """Gets and returns the X, Y and Z values from the accelerometer.

        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}


if __name__ == "__main__":
    mpu = mpu6050(0x68)
    #mpu.set_accel_offset()
    duracao=1
    n=duracao*600+1
    p=1000000/600
    accel_data= numpy.zeros((n+1,2))
    accel_data[0,0]=mpu.get_temp()
    filename=datetime.datetime.now().strftime("%H%M%S-%Y%m%d")
    titulo=unicode(datetime.datetime.now())
    tempo=int(round(time.time()*1000000))
    for i in range (1,n+1):
	#accel_data[i,0]=mpu.read_i2c_word(mpu.ACCEL_XOUT0)
	#accel_data[i,0]=mpu.read_i2c_word(mpu.ACCEL_YOUT0)
	accel_data[i,0]=mpu.read_i2c_word(mpu.ACCEL_ZOUT0)
	accel_data[i,1]=int(round(((time.time()*1000000)-tempo)/p))
	while int(round(((time.time()*1000000)-tempo)/p)) == accel_data[i,1]:
		pass

    """ for i in range (1,n+1):
	accel_data[i,0]=accel_data[i,0]/16384
	accel_data[i,1]=accel_data[i,1]/16384
	accel_data[i,2]=accel_data[i,2]/16384
    """
    #numpy.savetxt('teste.txt',accel_data,fmt='%f')
    numpy.savetxt(filename,accel_data,fmt='%i',header=titulo)


