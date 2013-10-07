
from dynamic_graph import plug

from dynamic_graph.sot.screwing.vel_control_functions import get_2ht, screw_2ht, goToHalfSitting, closeGrippers, openGrippers
from numpy import pi, array

from dynamic_graph.sot.screwing.vision_data_sim import plugObjectFix
from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo
from dynamic_graph.sot.core.matrix_util import vectorToTuple, matrixToTuple
from dynamic_graph.sot.screwing.screwing_data import *

class WrapperInt:
    def __init__(self,x):
        self.value = x

    def incr(self,n):
        self.value += n
        return self.value

i = WrapperInt(-1)

tool = (0.4,-0.1,0.8,0.,0.,pi/2)

def createReference(robot):

    P72=plugObjectFix(robot)

    # the out signal is always the actual goal
    HOLEINWORLD = Multiply_of_matrixHomo("HOLEINWORLD")
    plug(P72,HOLEINWORLD.sin1)

    return (HOLEINWORLD.sin2,HOLEINWORLD.sout)

#HOLEINWORLD.sin2.value = matrixToTuple(p72tohole[i.incr(1)])



