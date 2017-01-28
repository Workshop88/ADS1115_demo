#!/usr/bin/env python

# Raspberry Pi 3 B 
# ADS1115 4 channel 16 bit I2C A2D converter
# Demo by D. Scott Williamson
# This code and documentation is provided without warranty, 
# use code and instructions at your own risk.

# ADS115 Datasheet can be found here:
# http://www.ti.com/lit/ds/symlink/ads1113.pdf

# To enable I2C on your Raspberry Pi
# 	Run Raspberry Pi Configuration from the Preferences menu, 
# 	select Interfaces tab and enable I2C

# Install I2C tools
# 	Use this to install the tools if you don't already have it:
# 	sudo apt-get install -y i2c-tools
# 	Use the following to see the addresses of devices connected to the bus:
# 	sudo i2cdetect -y 1
# 	the "1" represents teh I2C device
# 		0 = /dev/i2c-0 (port I2C0) 
# 		1 = /dev/i2c-1 (port I2C1) *this is the port on the GPIO's of RPi 3 B

# Electrical connections
# Required:
# 	Connect SDA from ADS1115 to GPIO header pin 3 (I2C1 SDA,GPIO2)
# 	Connect SCL from ADS1115 to GPIO header pin 5 (I2C1 SCL,GPIO3)
# Optional analog inputs:
# 	Connect a potentiometer (any value between 1k and 10k will be fine)
# 		Connect power across the potentiometer (the two pins that measure 
#			the most resistance while the wiper is in the middle)
# 		One leg to GND, GPIO header pin 9 (or one of the other GND pins).
#		Oppsite leg to 3.3v GPIO header pin 1 (or one of the other 3.3v pins)
#		Connect the wiper, usually the center pin, to an analog input
#	Connect any analog input to 3.3v or GND

################################################################################
#
# Imports
#
import time	# used for sleep function
import sys		# used for print function
import smbus	# I2Cimport spidev

################################################################################
#
# Variables
#

# get access to the SMBus (SMBus is a subset of the I2C protocol
# 0 = /dev/i2c-0 (port I2C0) 
# 1 = /dev/i2c-1 (port I2C1) *this is the port on the GPIO's of RPi 3 B
bus = smbus.SMBus(1)    # the 1 means use port I2C1

# Address for the device 

addr=0x48       # ADDR tied to GND
#addr=0x49       # ADDR tied to VDD
#addr=0x4A       # ADDR tied to SDA
#addr=0x4B       # ADDR tied to SCL

# Registers in the ADS1115
DEVICE_REG_CONVERSION	= 0x00
DEVICE_REG_CONFIG 		= 0x01
DEVICE_REG_LO_THRESH	= 0x02
DEVICE_REG_HI_THRESH 	= 0x03

# Configuration register fields

# Operational Status
CONFIG_OS		 				= 0X8000
CONFIG_OS_START 				= 0X8000
CONFIG_OS_PERFORMING_CONVERSION	= 0X0000
CONFIG_OS_NOT_PERFORMING_OPERATION= 0X8000

# Differential measurements
CONFIG_MUX_AIN0P_AIN1N			= 0X0000 # (default)
CONFIG_MUX_AIN1P_AIN3N			= 0X1000
CONFIG_MUX_AIN2P_AIN3N			= 0X2000
CONFIG_MUX_AIN3P_AIN3N			= 0X3000
# Single ended measurements
CONFIG_MUX_AIN0P_GNDN			= 0X4000 
CONFIG_MUX_AIN1P_GNDN			= 0X5000
CONFIG_MUX_AIN2P_GNDN			= 0X6000
CONFIG_MUX_AIN3P_GNDN			= 0X7000

# Programmable gain amplifier configuration
CONFIG_FSR_6V144 				= 0X0000
CONFIG_FSR_4V096 				= 0X0200
CONFIG_FSR_2V048 				= 0X0400 # (default)
CONFIG_FSR_1V024 				= 0X0600
CONFIG_FSR_0V512 				= 0X0800
CONFIG_FSR_0V256 				= 0X0A00
CONFIG_FSR_0V256 				= 0X0C00
CONFIG_FSR_0V256 				= 0X0E00

# Continuous or single shot mode
CONFIG_MODE_CONTINUOUS 			= 0X0000
CONFIG_MODE_SINGLE_SHOT 		= 0X0100 # (default)

# Data rate
CONFIG_DATA_RATE_8SPS			= 0X0000
CONFIG_DATA_RATE_16SPS			= 0X0020
CONFIG_DATA_RATE_32SPS			= 0X0040
CONFIG_DATA_RATE_64SPS			= 0X0060
CONFIG_DATA_RATE_128SPS			= 0X0080 #(default)
CONFIG_DATA_RATE_2508SPS		= 0X00A0
CONFIG_DATA_RATE_475SPS			= 0X00C0
CONFIG_DATA_RATE_860SPS			= 0X00E0

# Comparitor mode
CONFIG_COMP_MODE_TRADITIONAL	= 0X0000 #(default)
CONFIG_COMP_MODE_WINDOW 		= 0X0010

# Comparitor polarity
CONFIG_COMP_POL_ACTIVE_LOW 		= 0X0000 #(default)
CONFIG_COMP_POL_ACTIVE_HIGH		= 0X0008

# Comparitor latching
CONFIG_COMP_LAT 				= 0X0004
CONFIG_COMP_LAT_NON_LATCHING	= 0X0000 #(default)
CONFIG_COMP_LAT_LATCHING 		= 0X0004

# comparitor queue and disable
CONFIG_COMP_QUE 				= 0X0003
CONFIG_COMP_QUE_1_CONV 			= 0X0000
CONFIG_COMP_QUE_2_CONV 			= 0X0001
CONFIG_COMP_QUE_4_CONV 			= 0X0002
CONFIG_COMP_QUE_DISABLE 		= 0X0003 #(default)

################################################################################
#
# Functions
#

# swap endian of word (swap high and low bytes)
# This is needed because the communication protocol uses a different byte 
# order than the Raspberry Pi
def swap(a):
	return ((a&0xff00)>>8)|((a&0x00ff)<<8)

# read ADC channel (blocking but quick)
# all of the hardware access is in this function
def readAdc(channel):
	# sanity check the channel specified
	if ((channel > 3) or (channel < 0)):
		return -1

	# Build read command
	config=(CONFIG_OS_START +				# start conversion 
			CONFIG_MUX_AIN0P_GNDN  + 		# single ended conversion
			(channel<<12) + 				# select channel
			CONFIG_FSR_4V096 + 				# 4.096v pre amp (3.3v signal)
			CONFIG_MODE_SINGLE_SHOT + 		# single conversion and shutdown
			CONFIG_DATA_RATE_128SPS + 		# data rate
			CONFIG_COMP_MODE_TRADITIONAL + 	# comp conventional 
			CONFIG_COMP_POL_ACTIVE_LOW + 	# comp active low
			CONFIG_COMP_LAT_NON_LATCHING + 	# comp non latching
			CONFIG_COMP_QUE_DISABLE )		# comp disabled
	
	# Uncomment to see the config command in printed in hexadecimal
	# print 'config: %04X' % config

	#send read command (note byte swap)
	bus.write_word_data(addr, DEVICE_REG_CONFIG, swap(config))
	 
	# wait for conversion to complete (blocking)
	while (True):
		# read status (note byte swap)
		status=swap(bus.read_word_data(addr,DEVICE_REG_CONFIG))
		
		# Uncomment to see the status printed in hexadecimal
		# print 'status: %04X' % status
		
		# when the Operational Status is no longer performing a conversion
		# we can break out of this wait loop 
		if (status & CONFIG_OS) != CONFIG_OS_PERFORMING_CONVERSION:
			break

	# read result (note byte swap)
	result=swap(bus.read_word_data(addr,DEVICE_REG_CONVERSION))
	
	# return 16 bit integer A2D result for the specified channel
	return result

# program entry
if __name__ == '__main__':
	print "=========================================================="
	print "Raspberry Pi 3 b"
	print "ADS1115 4 channel 16 bit I2C A2D converter demo"
	print "by D. Scott Williamson, 2017"
	print "This code and documentation is provided without warranty."
	print "Use at your own risk."
	print "=========================================================="
	time.sleep(3)
	
	try:
		while True:
			s="(Ctrl-C to exit) ADC Result: "
			for i in range(0,4):
				val = readAdc(i)
				s+=str(i)+": %05d"%val+"  "
			print s
			time.sleep(0.05)
	except KeyboardInterrupt:
		# Ctrl-C to exit
		sys.exit(0)
