#!/usr/bin/env python
#
# tinyUPS - Shutdown Skript for RaspberryPi
#
# This skript safely shuts down the RaspberryPi when the tinyUPS asks the Pi
# to do so. You have to connect the shutdown pin on the tinyUPS to a GPIO-pin
# on the Pi. This example uses pin 23, but you can change it if you want. The
# skript uses the gpiozero library and treats the shutdown line like a button.
# In order for it to work automatically, you must instruct the Pi to run this
# skript on startup. Refer to the readme.txt to learn how to do this.
# You can also use this script for a shutdown button. Simply connect the
# button between the GPIO pin and ground.
#
# 2020 by Stefan Wagner

from gpiozero 	import Button		# for reading shutdown line
from time	import sleep 		# for sleep function
import os 				# for shutdown control

SHTDWNPIN = 23				# GPIO pin the shutdown line is connected to

SDline = Button(SHTDWNPIN) 		# this also activates internal pullup resistor
while True: 				# infinite loop
	if SDline.is_pressed: 		# check if shutdown line goes low
		sleep(1) 		# wait for a hold time of one second
		if SDline.is_pressed:   # check if shutdown is still low
			os.system("sudo shutdown now -h")	# shut down the RaspberryPi
	sleep(1) 					# sleep one second before checking again
