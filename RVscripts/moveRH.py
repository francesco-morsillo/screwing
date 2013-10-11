# ______________________________________________________________________________
# ******************************************************************************
#
#    BASIC EXAMPLE OF THE INVERSE KINEMATICS
#       Robot: HRP-2 N.14
#       Tasks: Move right hand to a target
# 
# ______________________________________________________________________________
# ******************************************************************************  

from dynamic_graph.sot.core import *
from dynamic_graph.sot.dynamics import *
from dynamic_graph.sot.dyninv import *
from dynamic_graph.script_shortcuts import optionalparentheses
from dynamic_graph import plug

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core.utils.viewer_helper import addRobotViewer,VisualPinger,updateComDisplay
from dynamic_graph.sot.core.utils.thread_interruptible_loop import *
from dynamic_graph.sot.core.utils.attime import attime

from dynamic_graph.sot.dyninv.robot_specific import pkgDataRootDir, modelName, robotDimension, initialConfig, gearRatio, inertiaRotor, halfSittingConfig
from dynamic_graph.sot.dyninv.meta_task_dyn_6d import MetaTaskDyn6d
from dynamic_graph.sot.dyninv.meta_tasks_dyn import MetaTaskDynCom, MetaTaskDynPosture, AddContactHelper, gotoNd

from dynamic_graph.sot.core.utils.history import History



############################################################
###   CHOICE OF FIRST OR SECOND ORDER
############################################################
#-------------------------------------------------------------------------
# VELOCITY CONTROL

from dynamic_graph.sot.application.velocity.precomputed_meta_tasks import initialize
from dynamic_graph.sot.screwing.vel_control_functions import moveRightHandToTarget
gainMax = 5

# ACCELERATION CONTROL
"""
from dynamic_graph.sot.application.acceleration.precomputed_meta_tasks import initialize
from dynamic_graph.sot.screwing.acc_control_functions import moveRightHandToTarget
gainMax = 40
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
pose = robot.halfSitting

robot.device.set( pose )

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

# initialize in acceleration mode
solver = initialize(robot)

# ------------------------------------------------------------------------------
# --- MAIN LOOP ----------------------------------------------------------------
# ------------------------------------------------------------------------------

def inc():
    robot.device.increment(robot.timeStep)
    attime.run(robot.device.control.time)
    updateComDisplay(robot.device,robot.dynamic.com)

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

"""
#-----------------------------------------------------------------------------
# --- TRACER ------------------------------------------------------------------
#-----------------------------------------------------------------------------
from dynamic_graph.tracer import *
tr = Tracer('tr')
tr.open('/tmp/','','.dat')
tr.start()
robot.after.addSignal('tr.triger')

dyn = robot.dynamic

tr.add('dyn.com','com2')
tr.add(taskRH.task.name+'.error','erh2')

tr.add(taskLim.name+".normalizedPosition","qn")
robot.after.addSignal(taskLim.name+".normalizedPosition")

tr.add("dyn.position","q")
robot.after.addSignal("dyn.position")
tr.add("dyn.velocity","dq")
robot.after.addSignal("dyn.velocity")
tr.add("dyn.acceleration","ddq")
robot.after.addSignal("dyn.acceleration")
"""

#-----------------------------------------------------------------------------
# --- RUN --------------------------------------------------------------------
#-----------------------------------------------------------------------------

# RH metaTask
target = (0.5,-0.2,1.3)
robot.device.viewer.updateElementConfig('zmp',target+(0,0,0))
moveRightHandToTarget(robot,solver,target,gainMax)
