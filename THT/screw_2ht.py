# ______________________________________________________________________________
# ******************************************************************************
#
#    USE TOOL
#       Robot: HRP-2 N.14
#       Tasks: Screwing Mission
# 
# ______________________________________________________________________________
# ******************************************************************************  

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
from dynamic_graph.sot.core.utils.history import History

from numpy import linalg, array, pi, eye, dot


############################################################
###   CHOICE OF FIRST OR SECOND ORDER
############################################################
#-------------------------------------------------------------------------
# VELOCITY CONTROL

from dynamic_graph.sot.application.velocity.precomputed_meta_tasks import initialize
from dynamic_graph.sot.screwing.vel_control_functions import screw_2ht, get_2ht, openGrippers, closeGrippers, goToHalfSitting
gainMax = 10
gainMin = 0.5

# ACCELERATION CONTROL
"""
from dynamic_graph.sot.application.acceleration.precomputed_meta_tasks import initialize
from dynamic_graph.sot.screwing.acc_control_functions import get_2ht, openGrippers, closeGrippers, goToHalfSitting, screw_2ht
gainMax = 50
gainMin = 2
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

    if state >= 3 and state <= 21: 
        updateToolDisplay(robot.mTasks['screw'],linalg.inv(TwoHandToolToScrewMatrix),robot.device)
    """
    if state >= 3 and state <= 21:
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
# --- SUPERVISOR----------------------------------------------------------------
# ------------------------------------------------------------------------------

def supervision():

    global state,tool,robot,solver,pos_err_des,gainMax, gainMin, P72, p72tohole

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
            print "Goal: " + str(dot(P72,p72tohole[0]))
            screw_2ht(robot,solver,tool,P72,dot(P72,p72tohole[0]),gainMax, gainMin)
            #write_pos_py("/opt/grx3.0/HRP2LAAS/script/airbus_robot/",robot.device.state.value[6:36])
            state += 1
            print "time = "+str(robot.device.control.time)

    if state >= 3 and state<=19 :
        robot.mTasks['screw'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['screw'].feature.error.value)) < pos_err_des:
            print "state = "+str(state)
            if state<len(p72tohole)+2:
                print "Goal: " + str(dot(P72,p72tohole[state-2]))
                screw_2ht(robot,solver,tool,P72,dot(P72,p72tohole[state-2]),gainMax, gainMin)
                robot.device.viewer.updateElementConfig('goal1',vectorToTuple(array( matrixToRPY(dot( P72,p72tohole[state-2] )) )))

            if state == len(p72tohole)+2:
                state = 20
            else:
                state += 1
            print "time = "+str(robot.device.control.time)


    if state == 20:
        robot.mTasks['screw'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['screw'].feature.error.value)[0:3]) < pos_err_des:
            get_2ht(robot,solver,tool,gainMax, gainMin)
            state += 1
            print "time = "+str(robot.device.control.time)

    if state == 21:
        robot.mTasks['lh'].feature.error.recompute(robot.device.control.time)
        if linalg.norm(array(robot.mTasks['lh'].feature.error.value)[0:3]) < pos_err_des:
            openGrippers(robot,solver)
            #state = 8000 #needed if you want to make the robot wait
            state = 22
            print "time = "+str(robot.device.control.time)

    if state == 22:
        robot.device.state.recompute(robot.device.control.time)
        if linalg.norm(array([ robot.device.state.value[28]-robot.mTasks['posture'].ref[28]  , robot.device.state.value[35]-robot.mTasks['posture'].ref[35] ])) < 0.002:
            robot.device.viewer.updateElementConfig('TwoHandTool',(0.,0.5,0.,0.,0.,0.))
            goToHalfSitting(robot,solver,4)
            state = 23
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
    if state >= 21000 and state <21200:
        robot.device.state.recompute(robot.device.control.time)
        if linalg.norm(array([ robot.device.state.value[28]-robot.mTasks['posture'].ref[28]  , robot.device.state.value[35]-robot.mTasks['posture'].ref[35] ])) < 0.002:
            state += 1
        
    if state == 21200:
        state = 22
        print "time = "+str(robot.device.control.time)


# ------------------------------------------------------------------------------
# --- DATA ----------------------------------------------------------------
# ------------------------------------------------------------------------------

from dynamic_graph.sot.screwing.screwing_data import p72tohole

tool = (0.4,-0.1,0.8,0.,0.,pi/2)

P72RPY = (0.75,-0.45,0.85,0.,0.,1.57)

P72 = RPYToMatrix(P72RPY)

robot.device.viewer.updateElementConfig('TwoHandTool',(0.,0.,0.,0.,0.,0.))

robot.device.viewer.updateElementConfig('P72',P72RPY)

robot.device.viewer.updateElementConfig('goal1',vectorToTuple(array(matrixToRPY( dot(P72,p72tohole[0]) ))))

#-----------------------------------------------------------------------------
# --- RUN ----------------------------------------------------------------
#-----------------------------------------------------------------------------

get_2ht(robot,solver,tool,gainMax,gainMin)

state = 0

"""
# Motion recording
zmp_out = open("/tmp/data.zmp","w")
hip_out = open("/tmp/data.hip","w")
pos_out = open("/tmp/data.pos","w")
"""
