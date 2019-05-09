from __future__ import division
from scipy import signal
from filter import AHRS
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import math
import time
import csv

import logging
import sys
import time
import numpy as np
import RPi.GPIO as GPIO
from quatern_func import *
# import draw

from Adafruit_BNO055 import BNO055

###########################
# get data
#############################

GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)


bno = BNO055.BNO055(serial_port='/dev/serial0', rst=18)
#

# Enable verbose debug logging if -v is passed as a parameter.
if len(sys.argv) == 2 and sys.argv[1].lower() == '-v':
    logging.basicConfig(level=logging.DEBUG)

# Initialize the BNO055 and stop if something went wrong.
if not bno.begin():
    raise RuntimeError('Failed to initialize BNO055! Is the sensor connected?')

# Print system status and self test result.
status, self_test, error = bno.get_system_status()
print('System status: {0}'.format(status))
print('Self test result (0x0F is normal): 0x{0:02X}'.format(self_test))
# Print out an error if system status is in error mode.
if status == 0x01:
    print('System error: {0}'.format(error))
    print('See datasheet section 4.3.59 for the meaning.')

# Print BNO055 software revision and other diagnostic data.
sw, bl, accel, mag, gyro = bno.get_revision()
print('Software version:   {0}'.format(sw))
print('Bootloader version: {0}'.format(bl))
print('Accelerometer ID:   0x{0:02X}'.format(accel))
print('Magnetometer ID:    0x{0:02X}'.format(mag))
print('Gyroscope ID:       0x{0:02X}\n'.format(gyro))


print('Reading BNO055 data, press Ctrl-C to quit...')


GPIO.setmode (GPIO.BCM)
GPIO.setwarnings(False)

# setup gpio for echo & trig
echopin = [24,25]
trigpin = [23,17]

for j in range(2):
    GPIO.setup(trigpin[j], GPIO.OUT)
    GPIO.setup(echopin[j], GPIO.IN)
    print j, echopin[j], trigpin[j]
    print " "



# Get reading from HC-SR04
def ping(echo, trig):
    
    GPIO.output(trig, False)
    # Allow module to settle
    time.sleep(0.5)
    # Send 10us pulse to trigger
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)
    pulse_start = time.time()
    
    # save StartTime
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
    
    # save time of arrival
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
    
    # time difference between start and arrival
    pulse_duration = pulse_end - pulse_start
    # mutiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = pulse_duration * 17150
    
    distance = round(distance, 2)
    
    return distance




def checkValidity(a, b, c):
    
    # check condition
    if (a + b <= c) or (a + c <= b) or (b + c <= a) :
        return False
    else:
        return True




def calY(a,b,c):
    if checkValidity(a, b, c):
        print("Valid")
    else:
        print("Invalid")
        return
    
    
    top = (math.pow(a,2)+math.pow(b,2)-math.pow(c,2))
    bottom = (2.0 * a * b )
    aC = math.acos(top/bottom)
    aB = math.asin((b*math.sin(aC))/c)
    aA = math.asin((a*math.sin(aB))/b)

if (aB > math.radians(90)):
    aB = math.radians(180)-aB
    aY = math.radians(90) - aB
    #print math.degrees(aY)
    
    y = math.sin(aY)*a
    #print y
    return y



def GetPos(x,y):
    
    
    
    
    while True:
        sys, gyro, accel, mag = bno.get_calibration_status()
        print ("accelerometer calibration status: ", accel)
        if accel == 0: #3
            # wait until the accelerometer calibration finish
            print ("accelerometer calibration finished, press button 5 to start and press again to end")
            break

while True:
    if not GPIO.input(5):
        # wait until the button is pressed
        print ("start getting data")
        break
    
    c = 4.5
        
        for j in range(2):
            
            distance = ping(echopin[j], trigpin[j])
            print ("sensor", j+1,": ",distance,"cm")
            results = results + str(distance) + ","
            if j ==1:
                b = distance
            if j==0:
                a = distance

    print "done!"
    print a
    print b
    y = calY(a,b,c)
    x = math.sqrt(math.pow(a,2)-math.pow(y,2))
    print "y:",y
    print "x:",x
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
            print ("end getting data")
            break




plt.rcParams['lines.linewidth'] = 0.5

samplePeriod = 1./100
    
    
    
    accX = Acc[:,0]
    accY = Acc[:,1]
    accZ = Acc[:,2]
    
    t = len(accX)
    runtime = np.array([i*samplePeriod for i in xrange(t)])
    
    acc_mag = np.sqrt(accX**2 + accY**2 + accZ**2)
    
    ##########################
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
    
    
    
    quat = np.zeros((t,4))
)
    
    ######################################################
    ## Using quaternion to do the frame transformation (only for BNO055)
    ######################################################
    
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

# print acc_d.shape, acc.shape
# plt.figure('after rotate without drift')
# plt.plot(runtime, acc_d[:,[0]], 'r')
# plt.plot(runtime, acc_d[:,[1]], 'g')
# plt.plot(runtime, acc_d[:,[2]], 'b')
# plt.title('Acceleration')
# plt.xlabel('runTime (s)')
# plt.ylabel('Acceleration (m/s/s)')
# # plt.show()

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
    # print stationaryStart, '\n', stationaryEnd
    # print stationaryEnd[i]
    driftRate =  vel[stationaryEnd[i]-1,:] / (stationaryEnd[i]- stationaryStart[i])
    enum = range(0,(stationaryEnd[i] - stationaryStart[i]))
    drift = np.concatenate((np.transpose([enum])*driftRate[0],
                            np.transpose([enum])*driftRate[1],
                            np.transpose([enum])*driftRate[2]),axis = 1)
                            velDrift[stationaryStart[i]:stationaryEnd[i],:] = drift
                            
                            vel = vel-velDrift
                            
                            # plt.figure('velocity')
                            # plt.plot(runtime, vel[:,0], 'r')
                            # plt.plot(runtime, vel[:,1], 'g')
                            # plt.plot(runtime, vel[:,2], 'b')
                            # plt.title('Velocity')
                            # plt.xlabel('runTime (s)')
                            # plt.ylabel('Velocity (m/s)')
                            # plt.show()
                            
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
    pos = pos[0:l-1,:] #Jin Edit pos = pos[0:l-1,:]
    runtime = runtime[:l-1]
    
    # plt.figure('position')
    # plt.plot(runtime, pos[:,0], 'r')
    # plt.plot(runtime, pos[:,1], 'g')
    # plt.plot(runtime, pos[:,2], 'b')
    # plt.title('Position')
    # plt.xlabel('runTime (s)')
    # plt.ylabel('Position (m)')
    
    # plt.figure('trace')
    
    np.savetxt('position_data' ,pos, delimiter=',')
    ##    plt.figure('jsnks')
    ##    #print pos[:,0]
    ##    #print pos[:,1]
    ##    plt.plot(pos[:,0],pos[:,1],'r')
    ##    plt.plot([pos[0,0],pos[l-2,0]],[pos[0,1],pos[l-2,1]],'w')
    ##    print [pos[0,0],pos[l-2,0]],[pos[0,1],pos[l-2,1]]
    ##    print(pos)
    ##    plt.axis([min_range, max_range, min_range, max_range])
    ##    plt.title('movement')
    ##    plt.grid(True)
    #plt.show()
    
    return pos,
#draw_shape(pos)
