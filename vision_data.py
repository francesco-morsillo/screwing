from dynamic_graph import plug
from dynamic_graph.ros import *

from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo

def plugObject(robot):

    robot.ros = Ros(robot)
    robot.rosImport = robot.ros.rosImport
    robot.rosExport = robot.ros.rosExport

    robot.rosExport.add('matrixHomoStamped','objectInCamera','/object_position')
    
    robot.cameraFrameName = 'cameraBottomLeft'




    # Change of reference orientation from Visp to dynamic_graph
    visp2dg = ((0.,0.,1.,0.),(-1.,0.,0.,0.),(0.,-1.,0.,0.),(0.,0.,0.,1.))

    # Multiplies the position of the object in the camera reference for the camera orientation of Visp
    # The out signal is the object position in a reference located on the camera, but oriented as used in DG
    OBJECTINCAMERA_DG = Multiply_of_matrixHomo("OBJECTINCAMERA_DG")
    OBJECTINCAMERA_DG.sin1.value = visp2dg
    plug(robot.rosExport.objectInCamera,OBJECTINCAMERA_DG.sin2)

    # The out signal is objectInWorld
    OBJECTINWORLD = Multiply_of_matrixHomo("OBJECTINWORLD")
    plug(OBJECTINCAMERA_DG.sout,OBJECTINWORLD.sin2)
    plug(robot.frames[robot.cameraFrameName].position,OBJECTINWORLD.sin1)

    return OBJECTINWORLD.sout
