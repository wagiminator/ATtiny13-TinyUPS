#!/usr/bin/env python
#
# tinyUPS - Shutdown Request Script for RaspberryPi
#
# This script requests the start of the shutdown sequence from the TinyUPS.
# In this way, the power is disconnected from the RaspberryPi after the shutdown.
# You have to connect the request pin on the tinyUPS to a GPIO-pin on the Pi. This
# example uses pin 19, but you can change it if you want. The script uses the gpiozero
# library and treats the request line like an LED.
# To make it work automatically, follow the instructions in the readme file.
#
# 2020 by Stefan Wagner

from gpiozero 	import LED					# for controlling request line
from time		import sleep 				# for sleep function

UPSrequest = LED(19)						# treat the request line like an LED
UPSrequest.on()								# set the request line HIGH
sleep(3)									# hold HIGH for 3 seconds
UPSrequest.off()							# set the request line LOW again
