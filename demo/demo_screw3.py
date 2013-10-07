
from dynamic_graph.sot.screwing.vel_control_functions import get_2ht, screw_2ht, goToHalfSitting, closeGrippers, openGrippers

from dynamic_graph.sot.screwing.screwing_data import *

class WrapperInt:
    def __init__(self,x):
        self.value = x

    def incr(self,n):
        self.value += n
        return self.value

i = WrapperInt(-1)

tool = (0.4,-0.1,0.8,0.,0.,pi/2)

P72RPY = (0.75,-0.45,0.85,0.,0.,1.57)

P72 = RPYToMatrix(P72RPY)


#screw_2ht(robot,solver,tool,dot(P72,p72tohole[i.incr(1)]),4,0.5)
