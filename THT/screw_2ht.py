

from numpy import array,pi,linalg,matrix
from random import uniform
import time
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

from dynamic_graph.sot.screwing.utility import pos_err_des,TwoHandToolToScrewMatrix,TwoHandToolToTriggerMatrix

from dynamic_graph.sot.screwing.rob_view_lib import *


############################################################
###   CHOICE OF FIRST OR SECOND ORDER
############################################################
#-------------------------------------------------------------------------
# VELOCITY CONTROL
from dynamic_graph.sot.application.velocity.precomputed_meta_tasks import initialize
from dynamic_graph.sot.screwing.vel_control_functions import screw_2ht, get_2ht, openGrippers, closeGrippers, goToHalfSit
gain = 5

# ACCELERATION CONTROL
"""
from dynamic_graph.sot.application.acceleration.precomputed_meta_tasks import initialize
from dynamic_graph.sot.screwing.acc_control_functions import screw_2ht, get_2ht, openGrippers, closeGrippers, goToHalfSit
gain = 50
"""
#-------------------------------------------------------------------------

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
    robot.device.increment(robot.timeStep)
    attime.run(robot.device.control.time)
    updateComDisplay(robot.device,robot.dynamic.com)
    if state >= 3 and state < 10: 
        updateToolDisplay(robot.mTasks['screw'],linalg.inv(TwoHandToolToScrewMatrix),robot.device)
    """
    if state >= 3 and state <= 8:
        record_zmp(robot.device,robot.dynamic,zmp_out,robot.timeStep)
	record_hip(robot.device,robot.dynamic,hip_out,robot.timeStep)
	record_pos(robot.device,robot.dynamic,pos_out,robot.timeStep)
    """
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
def iter():         print 'iter = ',robot.device.state.time
@optionalparentheses
def status():       print runner.isPlay

def forward(steps):
    for i in range(steps):
        next()

# ------------------------------------------------------------------------------
# --- FUNCTIONS & CLASSES----------------------------------------------------------------
# ------------------------------------------------------------------------------

def supervision():

    global state,tool,robot,solver,pos_err_des,gain,goal

    if state == 0:
        robot.mTasks['lh'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            openGrippers(robot,solver)
            #state = 1000  #needed if you want to make the robot wait
            state=1
            print "time = "+str(robot.device.control.time)

    if state == 1:
        robot.device.state.recompute(robot.device.control.time)
        if linalg.norm(array([ robot.device.state.value[28]-robot.mTasks['posture'].ref[28]  , robot.device.state.value[35]-robot.mTasks['posture'].ref[35] ])) < 0.003:
            robot.device.viewer.updateElementConfig('TwoHandTool',tool)
            closeGrippers(robot,solver)
            state += 1
            print "time = "+str(robot.device.control.time)
           
    if state == 2:
        robot.device.state.recompute(robot.device.control.time)
        if linalg.norm(array([ robot.device.state.value[28]-robot.mTasks['posture'].ref[28]  , robot.device.state.value[35]-robot.mTasks['posture'].ref[35] ])) < 0.003:
            print "Goal: " + str(goal[0])
            screw_2ht(robot,solver,tool,goal[0],gain)
            write_pos_py("/opt/grx3.0/HRP2LAAS/script/airbus_robot/",robot.device.state.value[6:36])
            state += 1
            print "time = "+str(robot.device.control.time)

    if state >= 3 and state<=6 :
        robot.mTasks['screw'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['screw'].feature.error.value)) < pos_err_des:
            print "state = "+str(state)
            if state<6:
                print "Goal: " + str(goal[state-2])
                screw_2ht(robot,solver,tool,goal[state-2],gain)

            state += 1
            print "time = "+str(robot.device.control.time)


    if state == 7:
        robot.mTasks['screw'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['screw'].feature.error.value)[0:3]) < pos_err_des:
            get_2ht(robot,solver,tool,gain)
            state += 1
            print "time = "+str(robot.device.control.time)

    if state == 8:
        robot.mTasks['lh'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            openGrippers(robot,solver)
            #state = 8000 #needed if you want to make the robot wait
            state = 9
            print "time = "+str(robot.device.control.time)

    if state == 9:
        robot.device.state.recompute(robot.device.control.time)
        if linalg.norm(array([ robot.device.state.value[28]-robot.mTasks['posture'].ref[28]  , robot.device.state.value[35]-robot.mTasks['posture'].ref[35] ])) < 0.002:
            robot.device.viewer.updateElementConfig('TwoHandTool',(0.,0.5,0.,0.,0.,0.))
            goToHalfSit(robot,solver)
            state = 10
            print "time = "+str(robot.device.control.time)

    # wait 200*dt (1 second) after state 1
    if state >= 1000 and state <1200:
        robot.device.state.recompute(robot.device.control.time)
        if linalg.norm(array([ robot.device.state.value[28]-robot.mTasks['posture'].ref[28]  , robot.device.state.value[35]-robot.mTasks['posture'].ref[35] ])) < 0.002:
            state += 1
        
    if state == 1200:
        state = 1
        print "time = "+str(robot.device.control.time)

    # wait 200*dt (1 second) after state 8
    if state >= 8000 and state <8200:
        robot.device.state.recompute(robot.device.control.time)
        if linalg.norm(array([ robot.device.state.value[28]-robot.mTasks['posture'].ref[28]  , robot.device.state.value[35]-robot.mTasks['posture'].ref[35] ])) < 0.002:
            state += 1
        
    if state == 8200:
        state = 9
        print "time = "+str(robot.device.control.time)
        

# ------------------------------------------------------------------------------
# --- DATA ----------------------------------------------------------------
# ------------------------------------------------------------------------------

tool = (0.35,-0.1,0.9,0.,0.,pi/2)
robot.device.viewer.updateElementConfig('TwoHandTool',(0.,0.5,0.,0.,0.,0.))

P72 = (0.7,0.,0.9,0.,0.,1.57)
robot.device.viewer.updateElementConfig('P72',P72)

limit1 = array(P72) + array([-0.23,-0.3,0.23,0.,1.57,-1.57])
limit2 = array(P72) + array([-0.22,0.3,0.32,0.,1.57,-1.57])
limit3 = array(P72) + array([-0.22,0.3,-0.33,0.,1.57,-1.57])
limit4 = array(P72) + array([-0.23,-0.3,-0.38,0.,1.57,-1.57])

goal1 = limit1
goal2 = limit1 + 0.3*(limit4-limit1)
goal3 = limit1 + 0.7*(limit4-limit1)
goal4 = limit1 + 1.0*(limit4-limit1)

goal = array([goal1,goal2,goal3,goal4])

robot.device.viewer.updateElementConfig('goal1',vectorToTuple(goal1))
robot.device.viewer.updateElementConfig('goal2',vectorToTuple(goal2))
robot.device.viewer.updateElementConfig('goal3',vectorToTuple(goal3))
robot.device.viewer.updateElementConfig('goal4',vectorToTuple(goal4))

# ------------------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------
# ------------------------------------------------------------------------------

get_2ht(robot,solver,tool,gain)

state = 0

"""
# Motion recording
zmp_out = open("/tmp/data.zmp","w")
hip_out = open("/tmp/data.hip","w")
pos_out = open("/tmp/data.pos","w")
"""


"""
OLD GOALS

#goals
goal1 = array(P72) + array([-0.3,-0.3,0.3,0.,1.57,-1.57])
#goal4 = array(P72) + array([-0.3,+0.3,0.3,0.,1.57,-1.57])
#goal3 = array(P72) + array([-0.3,+0.3,-0.4,0.,1.57,-1.57])
#goal2 = array(P72) + array([-0.3,-0.3,-0.4,0.,1.57,-1.57])

goal12 = array(P72) + array([-0.3,-0.3,0.1,0.,1.57,-1.57])
goal11 = array(P72) + array([-0.3,-0.3,-0.15,0.,1.57,-1.57])
goal10 = array(P72) + array([-0.3,-0.3,-0.4,0.,1.57,-1.57])

goal9 = array(P72) + array([-0.3,-0.1,-0.4,0.,1.57,-1.57])
goal8 = array(P72) + array([-0.3,+0.1,-0.4,0.,1.57,-1.57])
goal7 = array(P72) + array([-0.3,+0.3,-0.4,0.,1.57,-1.57])

goal6 = array(P72) + array([-0.3,+0.3,-0.2,0.,1.57,-1.57])
goal5 = array(P72) + array([-0.3,+0.3,0.1,0.,1.57,-1.57])
goal4 = array(P72) + array([-0.3,+0.3,0.3,0.,1.57,-1.57])

goal3 = array(P72) + array([-0.3,+0.1,0.3,0.,1.57,-1.57])
goal2 = array(P72) + array([-0.3,-0.1,0.3,0.,1.57,-1.57])

goal3 = array([0.5,0.0,0.5,0.,1.57,0.])
goal4 = array([0.5,-0.6,0.5,0.,1.57,0.])
goal1 = array([0.5,-0.6,1.2,0.,1.57,0.])
goal2 = array([0.5,0.0,1.2,0.,1.57,0.])
"""
