from dynamic_graph import plug
from dynamic_graph.ros import *

from dynamic_graph.sot.core.math_small_entities import Multiply_of_matrixHomo, Inverse_of_matrix

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

def plugObjectSim(robot):

    objectInWorld = plugObject(robot)
    objectInWorld.recompute(robot.device.control.value)
    oIw=objectInWorld.value

    #since the real robot is not moving we have to compensate the changing of the camera position
    
    robot.ros = Ros(robot)
    robot.rosImport = robot.ros.rosImport
    robot.rosExport = robot.ros.rosExport
    robot.rosExport.add('matrixHomoStamped','objectInCamera','/object_position')    
    robot.cameraFrameName = 'cameraBottomLeft'

    inv = Inverse_of_matrix("inv")
    plug(robot.frames[robot.cameraFrameName].position,inv.sin)

    mult = Multiply_of_matrixHomo("mult")
    plug(inv.sout,mult.sin1)
    mult.sin2.value = oIw

    # The out signal is objectInWorld with simulated movement of the robot
    OBJECTINWORLD = Multiply_of_matrixHomo("OBJECTINWORLD")
    plug(mult.sout,OBJECTINWORLD.sin2)
    plug(robot.frames[robot.cameraFrameName].position,OBJECTINWORLD.sin1)

    return OBJECTINWORLD.sout

