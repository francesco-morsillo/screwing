
from dynamic_graph.sot.core.matrix_util import RPYToMatrix
from numpy import pi, array, eye, dot

def getData(security_distance):

    orientation = array([[0.,1.,0.],[0.,0.,-1],[-1.,0.,0.]])
    
    p72tohole1 = eye(4); p72tohole1[0:3,0:3] = orientation
    p72tohole1[0:3,3] = array([0.2,0.22+security_distance,0.1])
    
    p72tohole2 = eye(4); p72tohole2[0:3,0:3] = orientation
    p72tohole2[0:3,3] = array([0.1,0.22+security_distance,0.15])
    
    p72tohole3 = eye(4); p72tohole3[0:3,0:3] = orientation
    p72tohole3[0:3,3] = array([0.,0.22+security_distance,0.1])
    
    p72tohole10 = eye(4); p72tohole10[0:3,0:3] = orientation
    p72tohole10[0:3,3] = array([0.2,0.2+security_distance,-0.2])
    
    p72tohole9 = eye(4); p72tohole9[0:3,0:3] = orientation
    p72tohole9[0:3,3] = array([0.1,0.2+security_distance,-0.25])
    
    p72tohole8 = eye(4); p72tohole8[0:3,0:3] = orientation
    p72tohole8[0:3,3] = array([0.,0.2+security_distance,-0.2])
    
    
    p72tohole4 = eye(4); p72tohole4[0:3,0:3] = orientation
    p72tohole4[0:3,3] = p72tohole3[0:3,3] + 0.2*(p72tohole8[0:3,3] - p72tohole3[0:3,3])
    
    p72tohole5 = eye(4); p72tohole5[0:3,0:3] = orientation
    p72tohole5[0:3,3] = p72tohole3[0:3,3] + 0.4*(p72tohole8[0:3,3] - p72tohole3[0:3,3])
    
    p72tohole6 = eye(4); p72tohole6[0:3,0:3] = orientation
    p72tohole6[0:3,3] = p72tohole3[0:3,3] + 0.6*(p72tohole8[0:3,3] - p72tohole3[0:3,3])
    
    p72tohole7 = eye(4); p72tohole7[0:3,0:3] = orientation
    p72tohole7[0:3,3] = p72tohole3[0:3,3] + 0.8*(p72tohole8[0:3,3] - p72tohole3[0:3,3])
    
    
    p72tohole11 = eye(4); p72tohole11[0:3,0:3] = orientation
    p72tohole11[0:3,3] = p72tohole10[0:3,3] + 0.2*(p72tohole1[0:3,3] - p72tohole10[0:3,3])
    
    p72tohole12 = eye(4); p72tohole12[0:3,0:3] = orientation
    p72tohole12[0:3,3] = p72tohole10[0:3,3] + 0.4*(p72tohole1[0:3,3] - p72tohole10[0:3,3])
    
    p72tohole13 = eye(4); p72tohole13[0:3,0:3] = orientation
    p72tohole13[0:3,3] = p72tohole10[0:3,3] + 0.6*(p72tohole1[0:3,3] - p72tohole10[0:3,3])
    
    p72tohole14 = eye(4); p72tohole14[0:3,0:3] = orientation
    p72tohole14[0:3,3] = p72tohole10[0:3,3] + 0.8*(p72tohole1[0:3,3] - p72tohole10[0:3,3])
    
    p72tohole = array([p72tohole9,p72tohole10,p72tohole11,p72tohole12,p72tohole13,p72tohole14,p72tohole1,p72tohole2,p72tohole3,p72tohole4,p72tohole5,p72tohole6,p72tohole7,p72tohole8])

    return p72tohole
