#!/usr/bin/env python3
import serial
import matplotlib.pyplot as plt
import numpy as np
import sys
import time
import pickle
from datetime import datetime
from datetime import date
import os
from time import sleep
from random import uniform
plt.style.use('ggplot')

today = date.today()
today = today.strftime("%m_%d_%y")

now = datetime.now()
current_time = now.strftime("%H_%M_%S")


if len(sys.argv) == 1:
    arduino_port = "/dev/cu.usbmodem47110001"
else:
    arduino_port = sys.argv[1]
baud = 115200

ser = serial.Serial(arduino_port, baud)
print("Connected to Arduino port: " + arduino_port)

channels = 288
time.sleep(1)  # Pause 1 second for suspense
line_1 = 0
# Create a numpy array to store the spectrometer data and the integral result
spec_data_1 = np.zeros([1000, channels+2])
# Create an array for all the wavelengths
points = np.linspace(340, 850, num=channels)

line_2 = 0
# Create a numpy array to store the spectrometer data and the integral result
spec_data_2 = np.zeros([1000, channels+2])
fig, (spec1, spec2) = plt.subplots(2, sharex=True)
fig.suptitle('Spectrometer Data')
spec1.set_ylabel("Spectrometer 1")
spec1.plot(points, spec_data_1[0, :-2])
spec2.set_ylabel("Spectrometer 2")
spec2.plot(points, spec_data_2[0, :-2])
fig.show()

while 1:
    getData = str(ser.readline())
    print(getData[2:5])
    pr = getData.split(',')
    if pr[0] == "b'257" or pr[0] == "b'258":
        newstr = pr[0:][1:-3]
        newstr = [int(i) for i in newstr]
        spec_line_str = getData[0:][2:-4]
        # Update lines
        if pr[0] == "b'257":
            spec1.cla()
            spec1.plot(points, newstr)

        if pr[0] == "b'258":
            spec2.cla()
            spec2.plot(points, newstr)
        plt.tight_layout()
        plt.pause(0.05)
    sleep(0.1)