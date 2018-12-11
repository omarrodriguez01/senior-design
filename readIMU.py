from __future__ import division
from scipy import signal
from filter import AHRS
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math
import csv

import logging
import sys
import time
import numpy as np 
import RPi.GPIO as GPIO
from quatern_func import *


from Adafruit_BNO055 import BNO055

###########################
# get data
#############################

#start when button gets clicked
GPIO.setmode(GPIO.BCM) 
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

bno = BNO055.BNO055(serial_port='/dev/ttyAMA0', rst=18)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('error is verything connected right?')

print('Reading data, press Ctrl-C to quit...')
def GetPos():


    while True:
        sys, gyro, accel, mag = bno.get_calibration_status()
        print "accelerometer calibration status: ", accel 
        if accel == 3:
        # wait until the accelerometer calibration finish
            print "accelerometer calibration finished, press buttom 5 to start and press again to end"
            break

    while True:
        if not GPIO.input(5):
            # wait until the button is pressed
            print "start getting data"
            break

    t = 0 
    first_data = True

    while True:
        # Accelerometer data (in meters per second squared):
        AccX,AccY,AccZ = bno.read_linear_acceleration()

        # RawX,RawY,RawZ = bno.read_accelerometer()

        Acc_t = np.array([[AccX, AccY, AccZ]])
        # Orientation as a quaternion:
        Qx,Qy,Qz,Qw = bno.read_quaternion()
        Quat_t = np.array([[Qw, Qx, Qy, Qz]])

        if not first_data: 
            Acc = np.concatenate((Acc,Acc_t), axis = 0)
            Quat = np.concatenate((Quat,Quat_t), axis = 0)
        else:
            Acc = Acc_t
            Quat = Quat_t
            first_data = False
        
        t = t + 1
        time.sleep(1./100)
        if t > 50 and (not GPIO.input(5)):
            # Button is pressed
            print "end getting data"
            break    




    plt.rcParams['lines.linewidth'] = 0.5

    samplePeriod = 1./100
    accX = Acc[:,0]
    accY = Acc[:,1]
    accZ = Acc[:,2]

    t = len(accX)
    runtime = np.array([i*samplePeriod for i in xrange(t)])

    acc_mag = np.sqrt(accX**2 + accY**2 + accZ**2)

    #############################
    ## butterworth filters
    ############################
    filtCutOff = 0.001
    b, a = signal.butter(1,(2*filtCutOff)/(1/samplePeriod), 'high')
    acc_magFilt = signal.filtfilt(b, a, acc_mag)

    # % Compute absolute value
    acc_magFilt = np.absolute(acc_magFilt)

    # % LP filter accelerometer data
    filtCutOff = 5.
    b, a = signal.butter(1, (2*filtCutOff)/(1/samplePeriod), 'low')
    acc_magFilt = signal.filtfilt(b, a, acc_magFilt)

    accX_filt = signal.filtfilt(b, a, accX)
    accY_filt = signal.filtfilt(b, a, accY)
    accZ_filt = signal.filtfilt(b, a, accZ)

    accX_drift = np.mean(accX)
    accY_drift = np.mean(accY)
    accZ_drift = np.mean(accZ)

    ##############################
    # extract acceleration mean , minimize drift
    ##############################
    accX_fix_drift = accX_filt - accX_drift
    accY_fix_drift = accY_filt - accY_drift
    accZ_fix_drift = accZ_filt - accZ_drift

    # Threshold detection
    stationary = [a < 0.07 for a in acc_magFilt]
    l = len(accX)
    acc_fix_drift = np.concatenate((np.transpose([accX_fix_drift]), np.transpose([accY_fix_drift]), np.transpose([accZ_fix_drift])),axis = 1)

    for i in xrange(l):
        # acc = quaternRot([Gx,Gy,Gz], quaternInv([qw,qx,qy,qz]))
        # print acc_fix_drift[i,:],quaternInv(Quat[i,:])
        acc_d_t = quaternRot(acc_fix_drift[i,:],quaternInv(Quat[i,:]))
        if i == 0:
            acc_d = acc_d_t
        else:
            acc_d = np.concatenate((acc_d,acc_d_t),axis = 0)

    vel = np.zeros(acc_d.shape)

    t_total = len(vel) - 2

    for a in xrange(t_total):
        i = a + 1
        vel[[i],:] = vel[[i-1],:] + acc_d[[i],:] * samplePeriod
        if stationary[i]:
            vel[[i],:] = [[0,0,0]]

    #####################################################################
    ## calculate velocity drift
    #####################################################################

    velDrift = np.zeros(vel.shape)
    stationary_temp = [0] + np.diff(1 * np.array(stationary))
    stationaryStart = [i for i,x in enumerate(stationary_temp) if x == -1]
    stationaryEnd = [i for i,x in enumerate(stationary_temp) if x == 1]

    # print stationaryEnd, '\n', stationaryStart
    if len(stationaryEnd) > len(stationaryStart):
        if len(stationaryEnd) > 1:
        	stationaryEnd = stationaryEnd[1:]
        else:
        	stationaryEnd = []

    if len(stationaryStart) > len(stationaryEnd):
            stationaryEnd = stationaryEnd.append(t_total)

    if stationaryStart == [] :
        stationaryStart = [10]
    if stationaryEnd == [] or stationaryEnd == None:
    	stationaryEnd = [t_total]



    for i in xrange(len(stationaryEnd)):
        driftRate =  vel[stationaryEnd[i]-1,:] / (stationaryEnd[i]- stationaryStart[i])
        enum = range(0,(stationaryEnd[i] - stationaryStart[i]))
        drift = np.concatenate((np.transpose([enum])*driftRate[0],
                np.transpose([enum])*driftRate[1],
                np.transpose([enum])*driftRate[2]),axis = 1)
        velDrift[stationaryStart[i]:stationaryEnd[i],:] = drift

    vel = vel-velDrift

    pos = np.zeros(vel.shape)

    for a in xrange(t_total):
        i = a + 1
        pos[[i],:] = pos[[i-1],:] + vel[[i],:] * samplePeriod
    pos_x_min = np.min(pos[:,0])
    pos_x_max = np.max(pos[:,0])
    pos_y_min = np.min(pos[:,1])
    pos_y_max = np.max(pos[:,1])
    pos_z_min = np.min(pos[:,2])
    pos_z_max = np.max(pos[:,2])

    max_range = np.max([pos_x_max,pos_y_max,pos_z_max])
    min_range = np.min([pos_x_min,pos_y_min,pos_z_min])
    axis_range = max_range - min_range
    max_range = max_range + 0.1 * axis_range
    min_range = min_range - 0.1 * axis_range 

    l = len(pos[:,0])
    pos = pos[0:l-1,:]
    runtime = runtime[:l-1]


    np.savetxt('position_data' ,pos, delimiter=',')

    return pos