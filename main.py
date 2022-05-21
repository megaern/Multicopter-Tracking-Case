from simulation import Simulation
from visualisation import *
from time import sleep
#from signal import pause
from controller import *


#class Controller:
#    """ add controller implementation here  """
#    def decide(self, x, y, s, timeStamp):
#
#        #tag = 1 if we want to change the previous target position, otherwise tag = 0
#        tag = 1
#        return x, y + 1, tag


#source = 'image' #get signal measurements from image
source = 'sim' #simulate signal measurements
#source = 'image'

#if source == 'image', base_loc and base_max_r will be ignored

base_loc = (15, 5) # Simulation Base Location
base_max_r = 2.5 # Simulation Base Range - Estimated to be 10m

initialPos = (0, 768)

#Moving target or no, False by default
moving = False
ctrl = Controller()
sim = Simulation(ctrl, base_loc, base_max_r, source, initialPos, moving)
vis = Visualization(sim)

#pause()
print 'hello'
