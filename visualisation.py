import pygame
from pygame import gfxdraw, locals
from threading import Thread
from time import sleep,time
import math

"""
SCREEN_WIDTH = 914
SCREEN_HEIGHT = 510
"""
SCREEN_WIDTH = 1024
SCREEN_HEIGHT = 768
BLUE_1 = pygame.Color(0, 0, 255)
BLUE_2 = pygame.Color(0, 153, 255)
BLUE_3 = pygame.Color(51, 255, 255)
BLACK = pygame.Color(0, 0, 0)
GREEN = pygame.Color(0, 255, 0)
RED = pygame.Color(255, 0, 0)
WHITE = pygame.Color(255, 255, 255)
RADIUS = 30


class Visualization(Thread):

    def run(self):
        while(1):
            #if self.simulation.moving == True:
            #    self.angle += 0.1
            # #   x = RADIUS * math.cos(self.angle * 1 / math.pi)
            #   y = RADIUS * math.sin(self.angle * 1 / math.pi)
            #else:
            #    x = 0
            #    y = 0
            #self.screen.blit(self.background, (x, y))
            self.background = self.screen.fill(BLACK, rect= None)
            x, y, value = self.simulation.getState()
            self.draw(self.screen, x, y, value)
            pygame.display.flip()
            sleep(0.02)

    def __init__(self, simulation):
        Thread.__init__(self)
        self.fpsClock = pygame.time.Clock()
        self.simulation = simulation
        self.pygame = pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.trace = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), depth=32)
        self.trace.fill((0, 0, 0))
        #self.background = pygame.image.load("imms-heatmap.bmp")
        self.background = self.screen.fill(BLACK, rect= None)
        #self.screen.blit(self.background, (0, 0))
        self.scroll = 0
        self.daemon = True
        self.angle = 0
        self.start()

    def draw(self, screen, x, y, value):
        #By default full circle

        #meters to pixels conversion

        x, y = self.simulation.met2pix(x, y, self.simulation.x0, self.simulation.y0)

        self.simulation.isInsideFrame = 1

        if (x <= 0) or (x >= SCREEN_WIDTH - 1):
            self.simulation.isInsideFrame = 0

        if (y <= 0) or (y >= SCREEN_HEIGHT - 1):
            self.simulation.isInsideFrame = 0

        #print (x, y)
        #print 'Inside', self.simulation.isInsideFrame
        if self.simulation.isInsideFrame:
            #Draw the path
            gfxdraw.filled_circle(self.trace, x, y, 3, GREEN)

            #Draw the target
            base_loc = (15, 5)
            base_max_r = 2.5
            new_base_loc_x = int(round(base_loc[0]*SCREEN_WIDTH/20))
            new_base_loc_y = 768 - int(round(base_loc[1]*SCREEN_HEIGHT/10))
            new_base_radius = int(round(base_max_r*SCREEN_HEIGHT/10))
            gfxdraw.filled_circle(self.trace, new_base_loc_x, new_base_loc_y, 5, RED)
            pygame.draw.circle(self.screen, RED, (new_base_loc_x, new_base_loc_y), 175, 1)

        screen.blit(self.trace, (0, 0), special_flags=locals.BLEND_RGBA_ADD)

        if self.simulation.isInsideFrame:
            #Draw the multicopter
            color = self.set_color(value)

            gfxdraw.filled_circle(self.screen, x, y, 10, color)

    def set_color(self, value):
        if value < -85:
            color = BLUE_1
        elif value < -75:
            color = BLUE_2
        else:
            color = BLUE_3
        return color

