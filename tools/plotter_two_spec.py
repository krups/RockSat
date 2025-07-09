#!/usr/bin/env python3
import serial
import matplotlib.pyplot as plt
import numpy as np
import sys
from time import sleep
plt.style.use('ggplot')

if len(sys.argv) == 1:
    arduino_port = "/dev/cu.usbmodem47110001"
else:
    arduino_port = sys.argv[1]
baud = 115200

ser = serial.Serial(arduino_port, baud)
print("Connected to Arduino port: " + arduino_port)

channels = 288
sleep(1)  # Pause 1 second for suspense
line_1 = 0
# Create a numpy array to store the spectrometer data and the integral result
spec_data_1 = np.zeros([1000, channels])
# Create an array for all the wavelengths
points = np.linspace(340, 965, num=channels)
#points = np.linspace(340, 850, num=235)

line_2 = 0
# Create a numpy array to store the spectrometer data and the integral result
fig1,ax1 = plt.subplots(1)
fig1.suptitle('Spectrometer Data')
ax1.set_ylabel("Spectrometer 1")
#ax1.plot(points, spec_data_1[0,:])
fig1.show()

while 1:
    getData = str(str(ser.readline(),'utf-8').strip())
    #print(getData[2:5])
    pr = getData.split(',')
    print(pr)
    
    
    newstr = [int(i) for i in pr[:288]]
    #newstr = [int(i) for i in pr[:235]]
    #newstr = [int(i) for i in pr[-6:]]
    
    # Update lines
    ax1.cla()
    ax1.plot(points, newstr)

    for i in [496,567,638,709,780,850]:
        plt.axvline(i, color = 'b', linestyle="--")


    # update bar chart
    #ax1.cla()
    #ax1.bar([i for i in range(0,6)], newstr)

    # if pr[0] == "b'258":
    #     spec2.cla()
    #     spec2.plot(points, newstr)
    # plt.tight_layout()
    # plt.pause(0.05)
    
    # if pr[0] == "b'5'": # TC
    #     newstr = pr[0:][1:]
    #     newstr = [int(i) for i in newstr]
    #     tc_line_str = getData[0:][2:]
    #     # Update lines
    #     if pr[0] == "b'5":
    #         tc1.cla()
    #         tc1.plot(points, newstr)

    #plt.tight_layout()
    #     plt.pause(0.05)
    plt.pause(0.05)
    sleep(0.1)