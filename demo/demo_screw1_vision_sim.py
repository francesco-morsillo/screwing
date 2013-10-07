from dynamic_graph.sot.screwing.vision_data import plugObjectSim, createRos

from dynamic_graph import plug
from dynamic_graph.sot.screwing.vel_control_functions import get_2ht, screw_2ht, goToHalfSitting, closeGrippers, openGrippers
from numpy import pi, array, eye
from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo
from dynamic_graph.sot.core.matrix_util import matrixToTuple

tool = (0.4,-0.1,0.8,0.,0.,pi/2)

def createReference(robot):

    p72 = plugObjectSim(robot)

    # The out signal is the goal
    HOLEINWORLD = Multiply_of_matrixHomo("HOLEINWORLD")
    plug(p72,HOLEINWORLD.sin1)

    security_distance = 0.7

    orientation = array([[0.,1.,0.],[0.,0.,-1],[-1.,0.,0.]])

    p72tohole9 = eye(4); p72tohole9[0:3,0:3] = orientation
    p72tohole9[0:3,3] = array([0.1,0.2+security_distance,-0.25])

    sigIN = HOLEINWORLD.sin2
    sigIN.value = matrixToTuple(p72tohole9)

    sigOUT = HOLEINWORLD.sout

    return sigOUT
