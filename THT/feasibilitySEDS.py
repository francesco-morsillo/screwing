

from numpy import array,pi,linalg
from random import uniform
import sys

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph import plug

from dynamic_graph.sot.core.matrix_util import matrixToTuple, matrixToRPY, RPYToMatrix, vectorToTuple
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime
from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor, halfSittingConfig
from dynamic_graph.sot.core.feature_vector3 import *

from dynamic_graph.sot.screwing.utility import pos_err_des,TwoHandToolToScrewMatrix,TwoHandToolToTriggerMatrix
from dynamic_graph.sot.screwing.rob_view_lib import updateToolDisplay
from dynamic_graph.sot.application.acceleration.precomputed_tasks import initialize
from dynamic_graph.sot.screwing.openHRP.screw_2ht import screw_2ht
from dynamic_graph.sot.screwing.openHRP.get_2ht import get_2ht



class Robot:
    device = None
    dynamic = None
    timeStep = 0.005

    def __init__(self, device=None, dynamic=None, dt=0.005):
        self.device = device
        self.dynamic = dynamic
        self.timeStep = dt



# ------------------------------------------------------------------------------
# --- ROBOT DYNAMIC SIMULATION -------------------------------------------------
# ------------------------------------------------------------------------------

robot = Robot()

robotName = 'hrp14small'
robot.dimension   = robotDimension[robotName]
RobotClass = RobotSimu
robot.device      = RobotClass("robot")
robot.device.resize(robot.dimension)

x0=0.
y0=0.
z0=0.64870185118253043
robot.halfSitting = (x0,y0,z0,0,0,0)+halfSittingConfig[robotName][6:]

robot.device.set( robot.halfSitting )

addRobotViewer(robot.device,small=True,verbose=False)

robot.timeStep=5e-3


#-----------------------------------------------------------------------------
#---- DYN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

modelDir          = pkgDataRootDir[robotName]
xmlDir            = pkgDataRootDir[robotName]
specificitiesPath = xmlDir + '/HRP2SpecificitiesSmall.xml'
jointRankPath     = xmlDir + '/HRP2LinkJointRankSmall.xml'

robot.dynamic = Dynamic("dyn")
robot.dynamic.setFiles(modelDir,modelName[robotName],specificitiesPath,jointRankPath)
robot.dynamic.parse()

robot.dynamic.inertiaRotor.value = inertiaRotor[robotName]
robot.dynamic.gearRatio.value    = gearRatio[robotName]

plug(robot.device.state,robot.dynamic.position)
# for acceleration control the velocity plug is done in the initialization of the solver
#plug(robot.device.velocity,robot.dynamic.velocity)
robot.dynamic.velocity.value = robot.dimension*(0.,)
robot.dynamic.acceleration.value = robot.dimension*(0.,)

robot.dynamic.ffposition.unplug()
robot.dynamic.ffvelocity.unplug()
robot.dynamic.ffacceleration.unplug()

robot.dynamic.setProperty('ComputeBackwardDynamics','true')
robot.dynamic.setProperty('ComputeAccelerationCoM','true')

robot.device.control.unplug()

# ------------------------------------------------------------------------------
# --- SOLVER ----------------------------------------------------------------
# ------------------------------------------------------------------------------

solver = initialize(robot)

# ------------------------------------------------------------------------------
# --- MAIN LOOP ----------------------------------------------------------------
# ------------------------------------------------------------------------------

def inc():
    catch_exception = False
    robot.device.increment(robot.timeStep)
    attime.run(robot.device.control.time)
    updateComDisplay(robot.device,robot.dynamic.com)
    if state=='tryF': 
        updateToolDisplay(robot.mTasks['screw'],linalg.inv(TwoHandToolToScrewMatrix),robot.device)
    if 'state' in globals() : supervision()


@loopInThread
def loop():
    inc()
runner=loop()


# --- shortcuts -------------------------------------------------
@optionalparentheses
def go(): runner.play()
@optionalparentheses
def stop(): runner.pause()
@optionalparentheses
def next(): inc()
@optionalparentheses
def iter():         print 'iter = ',robot.state.time
@optionalparentheses
def status():       print runner.isPlay


# ------------------------------------------------------------------------------
# --- FUNCTIONS & CLASSES----------------------------------------------------------------
# ------------------------------------------------------------------------------


def random_tool_position(yMin, yMax, zMin, zMax,pitchMin,pitchMax):
    y = uniform(yMin,yMax)
    z = uniform(zMin,zMax)
    pitch = uniform(pitchMin,pitchMax)
    return (0.5,y,z,0.,pitch,1.57)

"""
def random_goal(yMin, yMax, zMin, zMax):
    return array([0.5,-0.3,1.0,0.,1.57,0.])
"""

def supervision():

    global state,tool,robot,solver,pos_err_des,gain,yMin, yMax, zMin, zMax,pitchMin,pitchMax, goal

    if state == 'init':
        robot.mTasks['lh'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            state = 'tryF'
            screw_2ht(robot,solver,tool,goal,gain)

    if state == 'tryF':
        robot.mTasks['screw'].feature.error.recompute(robot.device.control.time)
        controlNorm = linalg.norm(array(robot.device.control.value))
        if controlNorm > 100:
            print "\nControl Norm: "+str(controlNorm)

        if linalg.norm(array(robot.mTasks['screw'].feature.error.value)[0:3]) < pos_err_des:
            print "\nSUCCESS \n"
            fRes.append((tool[1],tool[2],tool[4],True))
            f_out.write(str((tool[1],tool[2],tool[4],True))+'\n')
            state = 'success'
            robot.device.setVelocity((0,)*36)
            robot.device.set(robot.halfSitting)
            tool = random_tool_position(yMin, yMax, zMin, zMax,pitchMin,pitchMax)
            robot.device.viewer.updateElementConfig('TwoHandTool',tool)
            get_2ht(robot,solver,tool,gain)

            
        elif controlNorm > 300:
            print "\nFAILURE \n"
            fRes.append((goal[1],goal[2],False))
            f_out.write(str((goal[1],goal[2],False))+'\n')
            state =  'failure'
            robot.device.setVelocity((0,)*36)
            robot.device.set(robot.halfSitting)
            tool = random_tool_position(yMin, yMax, zMin, zMax,pitchMin,pitchMax)
            robot.device.viewer.updateElementConfig('TwoHandTool',tool)
            get_2ht(robot,solver,tool,gain)
            
    
    if state == 'success':
        robot.mTasks['lh'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            state = 'tryF'
            screw_2ht(robot,solver,tool,goal,gain)

    if state == 'failure':
        robot.mTasks['lh'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            state = 'tryF'
            screw_2ht(robot,solver,tool,goal,gain)


# ------------------------------------------------------------------------------
# --- DATA ----------------------------------------------------------------
# ------------------------------------------------------------------------------

goal = array([0.5,-0.3,1.0,0.,1.57,0.])
robot.device.viewer.updateElementConfig('goal1',vectorToTuple(goal))

yMin = -0.05
yMax = -0.25
zMin = 0.8
zMax = 1.0
pitchMin = -pi/5
pitchMax = pi/5

gain = 100

# ------------------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------
# ------------------------------------------------------------------------------

tool = random_tool_position(yMin, yMax, zMin, zMax,pitchMin,pitchMax)
robot.device.viewer.updateElementConfig('TwoHandTool',tool)

get_2ht(robot,solver,tool,gain)

state = 'init'

fRes = list()

f_out = open('/tmp/feasibilitySEDS.txt','a')
