
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
TwoHandToolToScrewMatrix=array([[1.,0.,0.,-0.115],[0.,0.,-1.,-0.03],[0.,1.,0.,0.],[0.,0.,0.,1.]])

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



### SCRIPT UTILITIES

def write_xml(src_rep,pos36):	#dyn.position.value
	
	f1 = open(src_rep+"HRP2LAASPart1.xml","r")
	f3 = open(src_rep+"HRP2LAASPart3.xml","r")

	part1 = f1.read()
	part3 = f3.read()

	part2 = ''
	part2=part2+'            <property name="WAIST.translation" value="'+str(pos36[0])+' '+str(pos36[1])+' '+str(pos36[2])+' "/>\n'
	AA = extractAAFromRot(calcRotFromRPY(pos36[3],pos36[4],pos36[5]))
	part2=part2+'            <property name="WAIST.rotation" value="'+str(AA[0])+' '+str(AA[1])+' '+str(AA[2])+' '+str(AA[3])+' "/>\n'
	part2=part2+'            <property name="WAIST.angle" value="0.0"/>\n'
	part2=part2+'            <property name="RLEG_JOINT0.angle" value="'+str(pos36[6])+'"/>\n'
	part2=part2+'            <property name="RLEG_JOINT1.angle" value="'+str(pos36[7])+'"/>\n'
	part2=part2+'            <property name="RLEG_JOINT2.angle" value="'+str(pos36[8])+'"/>\n'
	part2=part2+'            <property name="RLEG_JOINT3.angle" value="'+str(pos36[9])+'"/>\n'
	part2=part2+'            <property name="RLEG_JOINT4.angle" value="'+str(pos36[10])+'"/>\n'
	part2=part2+'            <property name="RLEG_JOINT5.angle" value="'+str(pos36[11])+'"/>\n'
	part2=part2+'            <property name="LLEG_JOINT0.angle" value="'+str(pos36[12])+'"/>\n'
	part2=part2+'            <property name="LLEG_JOINT1.angle" value="'+str(pos36[13])+'"/>\n'
	part2=part2+'            <property name="LLEG_JOINT2.angle" value="'+str(pos36[14])+'"/>\n'
	part2=part2+'            <property name="LLEG_JOINT3.angle" value="'+str(pos36[15])+'"/>\n'
	part2=part2+'            <property name="LLEG_JOINT4.angle" value="'+str(pos36[16])+'"/>\n'
	part2=part2+'            <property name="LLEG_JOINT5.angle" value="'+str(pos36[17])+'"/>\n'
	part2=part2+'            <property name="CHEST_JOINT0.angle" value="'+str(pos36[18])+'"/>\n'
	part2=part2+'            <property name="CHEST_JOINT1.angle" value="'+str(pos36[19])+'"/>\n'
	part2=part2+'            <property name="HEAD_JOINT0.angle" value="'+str(pos36[20])+'"/>\n'
	part2=part2+'            <property name="HEAD_JOINT1.angle" value="'+str(pos36[21])+'"/>\n'
	part2=part2+'            <property name="RARM_JOINT0.angle" value="'+str(pos36[22])+'"/>\n'
	part2=part2+'            <property name="RARM_JOINT1.angle" value="'+str(pos36[23])+'"/>\n'
	part2=part2+'            <property name="RARM_JOINT2.angle" value="'+str(pos36[24])+'"/>\n'
	part2=part2+'            <property name="RARM_JOINT3.angle" value="'+str(pos36[25])+'"/>\n'
	part2=part2+'            <property name="RARM_JOINT4.angle" value="'+str(pos36[26])+'"/>\n'
	part2=part2+'            <property name="RARM_JOINT5.angle" value="'+str(pos36[27])+'"/>\n'
	part2=part2+'            <property name="RARM_JOINT6.angle" value="'+str(pos36[28])+'"/>\n'
	part2=part2+'            <property name="LARM_JOINT0.angle" value="'+str(pos36[29])+'"/>\n'
	part2=part2+'            <property name="LARM_JOINT1.angle" value="'+str(pos36[30])+'"/>\n'
	part2=part2+'            <property name="LARM_JOINT2.angle" value="'+str(pos36[31])+'"/>\n'
	part2=part2+'            <property name="LARM_JOINT3.angle" value="'+str(pos36[32])+'"/>\n'
	part2=part2+'            <property name="LARM_JOINT4.angle" value="'+str(pos36[33])+'"/>\n'
	part2=part2+'            <property name="LARM_JOINT5.angle" value="'+str(pos36[34])+'"/>\n'
	part2=part2+'            <property name="LARM_JOINT6.angle" value="'+str(pos36[35])+'"/>\n'


	out = open(src_rep+"HRP2LAASTry.xml","w")

	out.write(part1)
	out.write(part2)
	out.write(part3)

	out.close()

def write_pos_py(src_rep,pos30): #no free-floating

	pos30 = 180/3.14*array(pos30)

	f1 = open(src_rep+"airbus_screwing_part1.py","r")
	f3 = open(src_rep+"airbus_screwing_part3.py","r")

	part1 = f1.read()
	part3 = f3.read()

	part2 = ''
	part2=part2+'CF_init_pose.rleg = "'+str(pos30[0])+' '+str(pos30[1])+' '+str(pos30[2])+' '+str(pos30[3])+' '+str(pos30[4])+' '+str(pos30[5])+'"\n'
	part2=part2+'CF_init_pose.lleg = "'+str(pos30[6])+' '+str(pos30[7])+' '+str(pos30[8])+' '+str(pos30[9])+' '+str(pos30[10])+' '+str(pos30[11])+'"\n'
	part2=part2+'CF_init_pose.chest = "'+str(pos30[12])+' '+str(pos30[13])+'"\n'
	part2=part2+'CF_init_pose.head = "'+str(pos30[14])+' '+str(pos30[15])+'"\n'
	part2=part2+'CF_init_pose.rarm = "'+str(pos30[16])+' '+str(pos30[17])+' '+str(pos30[18])+' '+str(pos30[19])+' '+str(pos30[20])+' '+str(pos30[21])+' '+str(pos30[22])+'"\n'
	part2=part2+'CF_init_pose.larm = "'+str(pos30[23])+' '+str(pos30[24])+' '+str(pos30[25])+' '+str(pos30[26])+' '+str(pos30[27])+' '+str(pos30[28])+' '+str(pos30[29])+'"\n'

	out = open(src_rep+"airbus_screwing_new.py","w")

	out.write(part1)
	out.write(part2)
	out.write(part3)

	out.close()



























