import pygame
import os
import numpy
# import main
# from main import position
from readIMU import GetPos
import RPi.GPIO as GPIO
##############################################
## Inititlize parameter
#############################################
os.putenv('SDL_VIDEODRIVER','fbcon')



GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# exit button
def GPIO17_callback(channel):
    exit()

GPIO.add_event_detect(17, GPIO.FALLING, callback=GPIO17_callback,bouncetime=300)

pygame.init()

pygame.mouse.set_visible(False)
black = 0, 0, 0
size = width, height = 320, 240
screen = pygame.display.set_mode(size)
screen.fill((0,0,0))

try:
    while True:
        points = GetPos()
        points = numpy.array(points)
        points = points[:,[0,1]]
        ranges = numpy.ptp(points, axis=0)
        x_scale = 230/ranges[0]
        y_scale = 180/ranges[1]

        scale = min(x_scale,y_scale)
        points[:,0] = points[:,0]*scale
        points[:,1] = points[:,1]*scale
        means = numpy.mean(points,axis=0)

        points[:,0] = points[:,0] + (160-means[0])
        points[:,1] = points[:,1] + (120-means[1])
        ranges = numpy.ptp(points, axis=0)

        screen.fill(black)
        pygame.draw.lines(screen, (255,255,255), False, points, 2)		
        pygame.display.flip()

except KeyboardInterrupt:
    print "stop showing the record."