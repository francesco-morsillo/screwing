
from numpy import*
from dynamic_graph.sot.screwing.utility import *
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate, matrixToRPY


# FUNCTION AND MATRIXES NEEDED IN THE TOOL CONTROL


#---------------TOOL DISPLAY-------------------------------------------------------------------------------
def updateToolDisplay(task,displacement,robot):
	task.feature.position.recompute(robot.control.time)
	tool = dot( array(task.feature.position.value) , displacement )
	robot.viewer.updateElementConfig('TwoHandTool',vectorToTuple(tool[0:3,3])+vectorToTuple(extractRPYFromRot(tool[0:3,0:3])) )

def record_zmp(robot,dyn,zmp_out,dt):
	zmp_out.write(str(robot.control.time*dt)+"\t")
	ZMP = array(dyn.com.value)
	ZMP[2] = 0. #-0.645

	# extract waist position
	ZMP = dot( linalg.inv(array(dyn.waist.value)) , hstack([ZMP,1]))	

	for i in range(3):
		zmp_out.write(str(ZMP[i])+"\t")
	zmp_out.write("\n")

def record_hip(robot,dyn,hip_out,dt):
	hip_out.write(str(robot.control.time*dt)+"\t")
	HIP = dyn.position.value[3:6]
	for i in range(3):
		hip_out.write(str(HIP[i])+"\t")
	hip_out.write("\n")

def record_pos(robot,dyn,pos_out,dt):
	pos_out.write(str(robot.control.time*dt)+"\t")
	POS = [0]*40
	POS[0:30] = dyn.position.value[6:36]
	for i in range(40):
		pos_out.write(str(POS[i])+"\t")
	pos_out.write("\n")


