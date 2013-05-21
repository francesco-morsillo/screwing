
from numpy import*

#---------- UTILITY -------------------------------------------------------
#---------- UTILITY -------------------------------------------------------
#---------- UTILITY -------------------------------------------------------

# FUNCTION AND MATRIXES NEEDED IN THE TOOL CONTROL

## The matrixes are always named in the form A-To-B, where A and B are two references.
## The HTM A-To-B is the transformation needed to express a vector v, known in the B
## reference, in the A reference through the operation v_A = A-To-B * v_B

## The expression 6position is used to identify the representation of both position and orientation

## A legend is present in each section





### RPY to Rotation Matrix conversion ---------------------------------------

# roll: rotation around the x axis
# pitch: rotation around the y axis
# yaw: rotation around the z axis

# It is worth remarking the function "calcRotFromRPY" return only the rotation matrix 3x3
# and not the HTM with the same rotation.

def extractRPYFromRot(Rot):
	YAW=math.atan2(Rot[1][0],Rot[0][0])
	PITCH=math.atan2(-Rot[2][0],sqrt(Rot[2][1]**2+Rot[2][2]**2))
	ROLL=math.atan2(Rot[2][1],Rot[2][2])
	return array([ROLL,PITCH,YAW])

def calcRotFromRPY(roll,pitch,yaw):
	yawRot=array([[cos(yaw),-sin(yaw),0.],[sin(yaw),cos(yaw),0.],[0.,0.,1.]])
	pitchRot=array([[cos(pitch),0.,sin(pitch)],[0.,1.,0.],[-sin(pitch),0.,cos(pitch)]])
	rollRot=array([[1.,0.,0.],[0.,cos(roll),-sin(roll)],[0.,sin(roll),cos(roll)]])
	Rot=dot(dot(yawRot,pitchRot),rollRot)
	return Rot


### Angle-Axis to Rotation Matrix conversion ---------------------------------------

# It is worth remarking the function "calcRotFromAA" return only the rotation matrix 3x3
# and not the HTM with the same rotation.

def extractAAFromRot(Rot):
	theta = arccos((Rot[0,0]+Rot[1,1]+Rot[2,2]-1)/2)
	rx = 0.5*(Rot[2,1]-Rot[1,2])/sin(theta)
	ry = 0.5*(Rot[0,2]-Rot[2,0])/sin(theta)
	rz = 0.5*(Rot[1,0]-Rot[0,1])/sin(theta)
	return array([rx,ry,rz,theta])

def calcRotFromAA(rx,ry,rz,theta):
	ct = cos(theta)
	st = sin(theta)
	l1 = array([ rx*rx*(1-ct)+ct , rx*ry*(1-ct)-rz*st , rx*rz*(1-ct)+ry*st])
	l2 = array([ rx*ry*(1-ct)+rz*st , ry*ry*(1-ct)+ct , ry*rz*(1-ct)-rx*st])
	l3 = array([ rx*rz*(1-ct)-ry*st , rz*ry*(1-ct)+rx*st , rz*rz*(1-ct)+ct])
	return array([l1,l2,l3])


### Two-hands Screw-driver Homogeneous Transformation Matrixes ------------------------

# TwoHandToll = reference of the screw driver
# trigger = 6position where the left hand is supposed to be positioned
# support = 6position where the right hand is supposed to be positioned
# screw = 6position of the head of the screwing part

# From the reference to the handles.
TwoHandToolToTriggerMatrix=eye(4); TwoHandToolToTriggerMatrix[0:3,0:3]=array([[-1.,0.,0.],[0.,-1.,0.],[0.,0.,1.]])
TwoHandToolToTriggerMatrix[0:3,3] = (0.25,0.,0.)
TwoHandToolToSupportMatrix=eye(4); TwoHandToolToSupportMatrix[0:3,3] = (-0.03,0.,0.)

# From the reference to the screwing part (rotation on z)
TwoHandToolToScrewMatrix=array([[1.,0.,0.,-0.11],[0.,0.,-1.,-0.03],[0.,1.,0.,0.],[0.,0.,0.,1.]])

# From the screw to the support
TwoHandScrewToSupportMatrix=dot(linalg.inv(TwoHandToolToScrewMatrix),TwoHandToolToSupportMatrix)

# From the support to the trigger
TwoHandSupportToTriggerMatrix=dot(linalg.inv(TwoHandToolToSupportMatrix),TwoHandToolToTriggerMatrix)



### One-hand Screw-driver Homogeneous Transformation Matrixes ------------------------

# OneHandTool = reference of the screw-driver
# trigger = 6position where the right hand is supposed to be positioned
# screw = 6position of the head of the screwing part

# From the reference to the handle.
_xt=0.025
_yt=-0.09
OneHandToolToTriggerMatrix=eye(4); OneHandToolToTriggerMatrix[0:3,3] = array([_xt,_yt,0.])
theta = math.atan2(_xt,-_yt)
OneHandToolToTriggerMatrix[0:3,0:3] = array(([-sin(theta),0.,cos(theta)],[cos(theta),0.,sin(theta)],[0.,1.,0.]))

# From the reference to the screwing part (rotation on z).
OneHandToolToScrewMatrix=array([[0.,0.,-1,-0.12],[0.,1.,0.,0.],[1.,0.,0.,0.],[0.,0.,0.,1.]])

# Homogeneous Matrix screw to trigger
OneHandScrewToTriggerMatrix=dot(linalg.inv(OneHandToolToScrewMatrix),OneHandToolToTriggerMatrix)


### Demanded errors -------------------------------------------------------------------

# Position Error
pos_err_des = 0.0001
def position_error(task):
	task.recompute(robot.control.time)
	return linalg.norm( array(task.ref)[0:3,3] - array(task.feature.position.value)[0:3,3] )







