from dynamic_graph import plug

from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo, Inverse_of_matrixHomo
from dynamic_graph.sot.core.matrix_util import RPYToMatrix

def plugObjectFix(robot):

    P72RPY = (0.75,-0.45,0.85,0.,0.,1.57)
    oIw = RPYToMatrix(P72RPY)

    robot.cameraFrameName = 'cameraBottomLeft'

    inv = Inverse_of_matrixHomo("inv")
    plug(robot.frames[robot.cameraFrameName].position,inv.sin)

    mult = Multiply_of_matrixHomo("mult")
    plug(inv.sout,mult.sin1)
    mult.sin2.value = oIw

    # The out signal is objectInWorld with simulated movement of the robot
    OBJECTINWORLD = Multiply_of_matrixHomo("OBJECTINWORLD")
    plug(mult.sout,OBJECTINWORLD.sin2)
    plug(robot.frames[robot.cameraFrameName].position,OBJECTINWORLD.sin1)

    return OBJECTINWORLD.sout

