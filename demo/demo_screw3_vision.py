
from dynamic_graph.sot.screwing.vel_control_functions import get_2ht, screw_2ht, goToHalfSitting, closeGrippers, openGrippers
from numpy import pi, array

from dynamic_graph.sot.screwing.vision_data import plugObject
from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo
from dynamic_graph.sot.core.utils import vectorToTuple

class WrapperInt:
    def __init__(self,x):
        self.value = x

    def incr(self,n):
        self.value += n
        return self.value


tool = (0.4,-0.1,0.8,0.,0.,pi/2)

plugObject(robot,P72)

# the out signal is always the actual goal
HOLEINWORLD = Multiply_of_matrixHomo("HOLEINWORLD")
plug(P72,HOLEINWORLD.sin2)

security_distance = 0.2



p72tohole1 = array([[0.,1.,0.,-0.22-security_distance],[0.,0.,-1.,0.2],[-1.,0.,0.,0.1],[0.,0.,0.,1.] ])
p72tohole2 = array([[0.,1.,0.,-0.22-security_distance],[0.,0.,-1.,0.1],[-1.,0.,0.,0.15],[0.,0.,0.,1.] ])
p72tohole3 = array([[0.,1.,0.,-0.22-security_distance],[0.,0.,-1.,0.],[-1.,0.,0.,0.1],[0.,0.,0.,1.] ])

p72tohole10 = array([[0.,1.,0.,-0.22-security_distance],[0.,0.,-1.,0.2],[-1.,0.,0.,-0.2],[0.,0.,0.,1.] ])
p72tohole9 = array([[0.,1.,0.,-0.22-security_distance],[0.,0.,-1.,0.1],[-1.,0.,0.,-0.25],[0.,0.,0.,1.] ])
p72tohole8 = array([[0.,1.,0.,-0.22-security_distance],[0.,0.,-1.,0.],[-1.,0.,0.,-0.2],[0.,0.,0.,1.] ])

p72tohole4 = p72tohole3 + 0.2*(p72tohole8-p72tohole3)
p72tohole5 = p72tohole3 + 0.4*(p72tohole8-p72tohole3)
p72tohole6 = p72tohole3 + 0.6*(p72tohole8-p72tohole3)
p72tohole7 = p72tohole3 + 0.8*(p72tohole8-p72tohole3)

p72tohole11 = p72tohole10 + 0.2*(p72tohole1-p72tohole10)
p72tohole12 = p72tohole10 + 0.4*(p72tohole1-p72tohole10)
p72tohole13 = p72tohole10 + 0.6*(p72tohole1-p72tohole10)
p72tohole14 = p72tohole10 + 0.8*(p72tohole1-p72tohole10)

p72tohole = array([p72tohole9,p72tohole10,p72tohole11,p72tohole12,p72tohole13,p72tohole14,p72tohole1,p72tohole2,p72tohole3,p72tohole4,p72tohole5,p72tohole6,p72tohole7,p72tohole8])

i = WrapperInt(-1)

#HOLEINWORLD.sin1.value = vectorToTuple(p72tohole[i.incr(1)])



