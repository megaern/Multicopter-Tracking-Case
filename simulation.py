from math import exp,sqrt,atan2,cos,sin,fmod

from threading import Thread
from time import sleep,time
from pygame import *

from visualisation import *

class Simulation(Thread):


    def run(self):
        j = 0
        i = 0
        while (1):
            if (i == 0):
                # get new strength value
                if self.source == 'image':
                    self.s_curr = self.get_signal_from_image()
                elif self.source == 'sim':
                    self.s_curr = self.calc_sim_signal_strength()
                else:
                    print 'Wrong value for source type'

                #get new target set point
                (x_tar, y_tar,tag) = self.controller.decide(self.x_curr,self.y_curr,self.s_curr,time())
                if tag:
                    self.x_tar = x_tar
                    self.y_tar = y_tar
                ##print "New set point", self.x_tar, self.y_tar

            i += 1
            i = i % 10
            self.updateState()


			
            sleep(0.1)


    def __init__(self, controller, base_loc,base_max_r,source,(x0,y0), moving):

        Thread.__init__(self)

        self.controller = controller        
        self.isInsideFrame = 1
        self.source = source

        #copter's initial position in the frame (x0 in [0,914], y0 in [0,510])
        
        self.x0 = x0
        self.y0 = y0
        
        #copter's current position
        self.x_curr = 0
        self.y_curr = 0
        self.vel_curr = 0
        self.s_curr = -91
        self.theta_curr = 0

        #copter target position
        self.x_tar = 0
        self.y_tar = 0

        #base transmitter position
        self.base_loc = base_loc
        self.base_max_r = base_max_r


        self.img_size_x = 1024
        self.img_size_y = 768
        
        #Heatmap image
        #self.heatmap = pygame.image.load('imms-heatmap.bmp')
        self.heatmap = 0
        #Get top and min
        self.min_value, self.max_value = 0, 245 # self.get_image_max_min()
        #start function calls run() function
        self.daemon = False

        self.moving = moving
        self.start()
        


    def updateState(self):

        Td = 0.1
        self.theta_curr = atan2((self.y_tar - self.y_curr),(self.x_tar - self.x_curr))
        self.vel_curr = 1

        self.x_curr = self.x_curr + self.vel_curr * cos(self.theta_curr) * Td
        self.y_curr = self.y_curr + self.vel_curr * sin(self.theta_curr) * Td
        
        #print 'New pos', self.x_curr, self.y_curr, self.s_curr
        
    def getState(self):       
        return (self.x_curr,self.y_curr,self.s_curr)


    def calc_sim_signal_strength(self):

        dist_to_signal = sqrt(pow((self.x_curr-self.base_loc[0]),2)+pow((self.y_curr-self.base_loc[1]),2))
        
        if dist_to_signal > self.base_max_r:
            sim_signal_strength = -91
        elif 0 <= dist_to_signal and dist_to_signal <= self.base_max_r:
            sim_signal_strength = 1/(dist_to_signal+1)

            #print 'sim_signal_strength', sim_signal_strength
            sim_signal_strength = round(25*sim_signal_strength - 90)

        return sim_signal_strength


    def get_image_max_min(self):
        max_v = 0
        min_v = 255
        x, y = self.heatmap.get_size()[:]
        for i in range(0, x):
            for j in range(0, y):
                value = self.heatmap.get_at((i, j))[0]
                if (value < min_v):
                    min_v = value
                if (value > max_v):
                    max_v = value
        return min_v, max_v


    def get_signal_from_image(self):
        
        
        #meters to pixels conversion
        print 'Position in meters', (self.x_curr,self.y_curr)
        x,y = self.met2pix(self.x_curr,self.y_curr,self.x0,self.y0)

        print 'Position in pixels',(x,y)        
        value = self.heatmap.get_at((x, y))[0]        

        print 'Value on image',value
        
        maxValue = self.max_value
        minValue = self.min_value

        value = round((25.0 /(maxValue - minValue) * value - 25.0 * maxValue/(maxValue - minValue)-65.0))

        print 'Value in decibels',value
        return value


    def met2pix(self,x,y,x0,y0):

        #(x0,y0) - origin of the coordinate system
        #(x,y) - point values

        
        x = int(round(x * self.img_size_x / 20.0 + x0))
        
        if x < 0:
            x = 0
            
        elif x > (self.img_size_x - 1) :
            x = (self.img_size_x - 1)
        
            
        y = int(round(-y * self.img_size_y / 10.0 + y0))
        if y < 0:
            y = 0

        elif y >(self.img_size_y - 1):
            y = (self.img_size_y - 1)

        return x,y




