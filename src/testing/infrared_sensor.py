#!/bin/usr/python3

import RPi.GPIO as gp

gp.setmode(gp.BCM)
gp.setwarnings(False)

infraRedSensorPin = 19
gp.setup(infraRedSensorPin, gp.IN)

while True:
    print(gp.input(infraRedSensorPin))
