import pygame
import os
import numpy
import time
# import main
# from main import position
from readIMU import GetPos
import RPi.GPIO as GPIO
##############################################
## Inititlize parameter
#############################################
#os.putenv('SDL_VIDEODRIVER','fbcon')
os.putenv('SDL_FBDEV', '/dev/fb1')

screen_width=700
screen_height=400


GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#exit button
def GPIO17_callback(channel):
    exit()

GPIO.add_event_detect(17, GPIO.FALLING, callback=GPIO17_callback,bouncetime=300)

pygame.init()


#pygame.mouse.set_visible(False)
black = 0, 0, 0
size = width, height = 320, 240                    # Jin  edit from 320, 240, changes size window, but doesnt extend the points being plotted
screen = pygame.display.set_mode(size)
screen=pygame.display.set_mode([screen_width,screen_height])
screen.fill((0,0,0))

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




try:
    top = GetPos()
    print("top data done")
    right = GetPos()
    print("right data done")
    bottom = GetPos()
    print("bottom data done")
    left = GetPos()
    
    
    top = numpy.array(top)
    top = top[:,[0,1]]
    bottom = numpy.array(bottom)
    bottom = bottom[:,[0,1]]
    right = numpy.array(right)
    right = right[:,[0,1]]
    left = numpy.array(left)
    left = left[:,[0,1]]
    print("\n")
    print(max(top[:,1]))
    print("\n")
    mr = max(right[:,0])
    print(max(right[:,0]))
    print("\n")
    print(min(bottom[:,1]))
    print("\n")
    print(min(left[:,0]))
    print("\n")
    
    i =0
    while True:
        x =0
        y=0
        
        points = GetPos(x,y)
        
        points = numpy.array(points)
        print(points[0:1])
        print("\n")
        points = points[:,[0,1]]
        print(points[0:1])
        
        if any(x >mr for x in points[:,0]):
            print("\nte pasaste de verga\n")
    
        ##        if(points[:,0] > right[:,0]):
        ##            print("te pasaste de verga")
        ##        if(points[:,0] < left[:,0]):
        ##            print("te pasaste de verga")
        ##        if(points[:,1] > top[:,1]):
        ##            print("te pasaste de verga")
        ##        if(points[:,1] < bottom[:,1]):
        ##            print("te pasaste de verga")
        #print(points[0:1])
        ranges = numpy.ptp(points, axis=0)
        
        
        x_scale = 230/ranges[0]
        y_scale = 180/ranges[1]
        
        #print(points)
        scale = min(x_scale,y_scale)
        print("\n")
        
        points[:,0] = points[:,0]*scale*-1
        points[:,1] = points[:,1]*scale
        means = numpy.mean(points,axis=0)
        #print(points[:,0])
        print("\n")
        a = [-80,60,200,340,480]
        
        points[:,0] = points[:,0] + (160-means[0]) #+ a[i]
        points[:,1] = points[:,1] + (120-means[1])
        #print(points[:,0])
        ##        points[:,1] = points[:,1] -80
        ranges = numpy.ptp(points, axis=0)
        #i= i+1
        
        #screen.fill(black)
        pygame.draw.lines(screen, (255,255,255), False, points, 2)            #pygame.draw.circle(screen,(255,255,255),[60,250],40) Surface, color, closed, pointlist, width
        pygame.display.flip()
#if (i==5):
#i=0
#screen.fill(black)


except KeyboardInterrupt:
    print ("stop showing the record.")
