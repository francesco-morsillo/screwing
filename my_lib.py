
from numpy import*

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
