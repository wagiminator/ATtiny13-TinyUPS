#!/usr/bin/env python
#
# tinyUPS - Shutdown Script for RaspberryPi
#
# This script safely shuts down the RaspberryPi when the tinyUPS asks the Pi to do so.
# You have to connect the shutdown pin on the tinyUPS to a GPIO-pin on the Pi. This
# example uses pin 26, but you can change it if you want. The script uses the gpiozero
# library and treats the shutdown line like a button which also activates the internal
# pullup resistor.
# To make it work automatically, follow the instructions in the readme file.
# You can also use this script for a shutdown button. Simply connect the button between
# the GPIO pin and ground.
#
# 2020 by Stefan Wagner

from gpiozero 	import button					# for reading shutdown line
from signal		import pause 					# for pause function
import os 										# for shutdown control

SDline = Button(26, hold_time = 1) 				# shutdown line connected to GPIO pin 26
SDline.when_held = os.system("shutdown now -h")	# and shutdown when low for 1 second
pause()											# nothing else to do
