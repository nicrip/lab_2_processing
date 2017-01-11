
import numpy as np
np.set_printoptions(precision=3, suppress=True)

from itertools import product, combinations
from math import sin, cos, sqrt, pi


def normalize(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v

def q_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    q = normalize(q)
    w, x, y, z = q
    return (w, -x, -y, -z)

def qv_mult(q1, v1):
    #v1 = normalize(v1)
    q2 = (0.0,) + v1
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[1:]

def axisangle_to_q(v, theta):
    v = normalize(v)
    x, y, z = v
    theta /= 2
    w = cos(theta)
    x = x * sin(theta)
    y = y * sin(theta)
    z = z * sin(theta)
    return w, x, y, z

def q_to_axisangle(q):
    w, v = q[0], q[1:]
    theta = acos(w) * 2.0
    return normalize(v), theta

def SE3_v((q, t), v):
    x,y,z = t
    x_,y_,z_ = qv_mult(q,v)
    return x_+x, y_+y, z_+z

def q2R(q):
    #Convert quaternion to linear rotation matrix.
    #Fixed frame rotation
    #q = [scalar vector]
    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];
    nrm = sqrt(w*w+x*x+y*y+z*z)
    if (np.abs(nrm) < 0.999):
        print "q2C -- not a unit quaternion"
        return np.eye(3)
    #compute
    nrm = 1./nrm
    w = w*nrm
    x = x*nrm
    y = y*nrm
    z = z*nrm
    x2 = x*x
    y2 = y*y
    z2 = z*z
    w2 = w*w
    xy = 2*x*y
    xz = 2*x*z
    yz = 2*y*z
    wx = 2*w*x
    wy = 2*w*y
    wz = 2*w*z
    #build matrix
    R = np.zeros((3,3))
    R[0,0] = w2+x2-y2-z2
    R[0,1] = xy-wz
    R[0,2] = xz+wy
    R[1,0] = xy+wz
    R[1,1] = w2-x2+y2-z2
    R[1,2] = yz-wx
    R[2,0] = xz-wy
    R[2,1] = yz+wx
    R[2,2] = w2-x2-y2+z2
    return R


def skew(v):
    #if len(v) == 4: v = v[:3]/v[3]
    #skv = roll(roll(diag(v.flatten()), 1, 1), -1, 0)
    #return skv - skv.T
    return np.matrix([[0., -v[2], v[1]],[v[2],0.,-v[0]],[-v[1],v[0],0.]])    
    
    
    
def insRrows02_col0_at(M,R,col,rwo=0):
    M[rwo+0,col] = R[0,0]
    M[rwo+1,col] = R[1,0]
    M[rwo+2,col] = R[2,0]

def thrMixerMatrix(param, conf):
    M = np.zeros((6,6))
    #Force thr1-6
    insRrows02_col0_at(M,np.dot(param.cRm, q2R(conf.q12)), 0)
    insRrows02_col0_at(M,np.dot(param.cRm, q2R(conf.q12)), 1)
    insRrows02_col0_at(M,np.dot(param.cRm, q2R(conf.q34)), 2)
    insRrows02_col0_at(M,np.dot(param.cRm, q2R(conf.q34)), 3)
    insRrows02_col0_at(M,np.dot(param.cRm, q2R(conf.q56)), 4)
    insRrows02_col0_at(M,np.dot(param.cRm, q2R(conf.q56)), 5)
    #Torque thr1-6
    if (True):
        insRrows02_col0_at(M, np.dot(np.dot(param.cRm, skew(np.array(conf.p1))), q2R(conf.q12)), 0, 3)
        insRrows02_col0_at(M, np.dot(np.dot(param.cRm, skew(np.array(conf.p2))), q2R(conf.q12)), 1, 3)
        insRrows02_col0_at(M, np.dot(np.dot(param.cRm, skew(np.array(conf.p3))), q2R(conf.q34)), 2, 3)
        insRrows02_col0_at(M, np.dot(np.dot(param.cRm, skew(np.array(conf.p4))), q2R(conf.q34)), 3, 3)
        insRrows02_col0_at(M, np.dot(np.dot(param.cRm, skew(np.array(conf.p5))), q2R(conf.q56)), 4, 3)
        insRrows02_col0_at(M, np.dot(np.dot(param.cRm, skew(np.array(conf.p6))), q2R(conf.q56)), 5, 3)
    
    return M


    
    
    
    
    
    
class MechParam():
    def __init__(self):
        #self.setMechCamOrients(axisangle_to_q((0,1,0), -pi/4))
        self.setMechCamOrients(q_mult( axisangle_to_q((0,0,1),0*pi/4), 
                                       axisangle_to_q((0,1,0), -1*pi/4)))
        #self.setMechCamOrients(axisangle_to_q((0,0,1), -pi/4))
        #self.setMechCamOrients((1,0,0,0))
        
    def setMechCamOrients(self, mQc):
        self.mQc = mQc
        self.mRc = q2R(self.mQc)
        self.cQm = q_conjugate(self.mQc)
        self.cRm = q2R(self.cQm)

        
class Config01:
    def __init__(self):
        #Thruster 1&2
        the_2 = 3*pi/4/2
        self.q12 = (cos(the_2),0,sin(the_2),0)
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        the_2 = 3*pi/4/2
        self.q34 = (cos(the_2),0,-sin(the_2),0)
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        the_2 = 2*pi/4/2
        self.q56 = (cos(the_2),0,0,sin(the_2))
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))


class Config02:
    def __init__(self):
        #Thruster 1&2
        the_2 = 3*pi/4/2
        self.q12 = (cos(the_2),0,sin(the_2),0)
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        the_2 = 3*np.pi/4/2
        self.q34 = (cos(the_2),0,-sin(the_2),0)
        self.p5 = (1,1,0)
        self.p6 = (-1,-1,0)
        #Thruster 5&6
        the_2 = 2*pi/4/2
        self.q56 = (cos(the_2),0,0,sin(the_2))
        self.p3 = (0,0,sqrt(2))
        self.p4 = (0,0,-sqrt(2))


class Config03:
    def __init__(self):
        #Thruster 1&2
        the_2 = 2*pi/4/2
        self.q12 = (cos(the_2),0,0,sin(the_2))
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        the_2 = 3*pi/4/2
        self.q34 = (cos(the_2),0,-sin(the_2),0)
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        the_2 = -1*pi/4/2
        self.q56 = (cos(the_2),0,sin(the_2),0)
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))


class Config04:
    def __init__(self):
        #Thruster 1&2
        the_2 = 2*pi/4/2
        self.q12 = (cos(the_2),0,0,sin(the_2))
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        the_2 = 3*pi/4/2
        self.q34 = (cos(the_2),0,-sin(the_2),0)
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        the_2 = -1*pi/4/2
        self.q56 = (cos(the_2),0,sin(the_2),0)
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))


class Config05():
    def __init__(self):
        #Thruster 1&2
        self.q12 = axisangle_to_q((0,0,1), -pi/2)
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        self.q34 = axisangle_to_q((0,1,0), -3*pi/4)
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        self.q56 = axisangle_to_q((0,1,0), -1*pi/4)
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))

# force on Njord, in efficient prop direction.
#rank 5
class Config06:
    def __init__(self):
        #Thruster 1&2
        self.q12 = axisangle_to_q((0,1,0), -1*pi/4)
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        self.q34 = axisangle_to_q((0,1,0), 1*pi/4)
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        self.q56 = axisangle_to_q((0,0,1), -pi/2)
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))

        
# force on Njord, in efficient prop direction.
# weird rank(6)
class Config07:
    def __init__(self):
        #Thruster 1&2
        self.q12 = q_mult(axisangle_to_q((0,0,1), 0*pi/4), 
                          axisangle_to_q((0,1,0), -1*pi/4))
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        self.q34 = q_mult(axisangle_to_q((0,0,1), -pi/4),
                          axisangle_to_q((0,1,0), 1*pi/4))
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        self.q56 = axisangle_to_q((0,0,1), -pi/2)
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))


class Config08:
    def __init__(self):
        #Thruster 1&2
        self.q12 = q_mult(axisangle_to_q((0,0,1), 0*pi/4), 
                          axisangle_to_q((0,1,0), 1*pi/2))
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        self.q34 = q_mult(axisangle_to_q((0,0,1), 0*pi/4),
                          axisangle_to_q((0,1,0), 0*pi/4))
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        self.q56 = axisangle_to_q((0,0,1), 1*pi/2)
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))  


class Config09:
    def __init__(self):
        #Thruster 1&2
        self.q12 = q_mult(axisangle_to_q((0,0,1), 0*pi/4), 
                          axisangle_to_q((0,1,0), 1*pi/2))
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        self.q34 = q_mult(axisangle_to_q((0,0,1), -1*pi/4),
                          axisangle_to_q((0,1,0), 0*pi/4))
        self.p3 = (1,1,0)
        self.p4 = (-1,-1,0)
        #Thruster 5&6
        self.q56 = q_mult(axisangle_to_q((0,0,1), 1*pi/4),
                          axisangle_to_q((0,1,0), 0*pi/4))
        self.p5 = (0,0,sqrt(2))
        self.p6 = (0,0,-sqrt(2))

        
class Config10:
    def __init__(self):
        qAll = q_mult(axisangle_to_q((0,1,0), 0*pi/8),
                      axisangle_to_q((0,0,1), 1*pi/4))
        #Thruster 1&2
        self.q12 = q_mult(qAll,
                          axisangle_to_q((0,1,0), 1*pi/2))
        self.p1 = (1,-1,0)
        self.p2 = (-1,1,0)
        #Thruster 3&4
        self.q34 = q_mult(qAll,
                          axisangle_to_q((0,0,1), -1*pi/4))
        yo = 1.0
        self.p3 = (sqrt(2-yo*yo),yo,0)
        self.p4 = (-sqrt(2-yo*yo),-yo,0)
        #Thruster 5&6
        self.q56 = q_mult(qAll,
                          axisangle_to_q((0,0,1), -3*pi/4))
        z = 2
        self.p5 = (0,0,sqrt(z))
        self.p6 = (0,0,-sqrt(z))
        
class Config11:
    def __init__(self):
        qAll = q_mult(axisangle_to_q((0,1,0), 0*pi/8),
                      axisangle_to_q((0,0,1), 1*pi/4))
        #Thruster 1&2
        self.q34 = q_mult(qAll,
                          axisangle_to_q((0,1,0), 1*pi/2))
        self.p3 = (1,-1,0)
        self.p4 = (-1,1,0)
        #Thruster 3&4
        self.q12 = q_mult(qAll,
                          axisangle_to_q((0,0,1), -1*pi/4))
        yo = 1.0
        self.p1 = (sqrt(2-yo*yo),yo,0)
        self.p2 = (-sqrt(2-yo*yo),-yo,0)
        #Thruster 5&6
        self.q56 = q_mult(qAll,
                          axisangle_to_q((0,0,1), -3*pi/4))
        z = 2
        self.p5 = (0,0,sqrt(z))
        self.p6 = (0,0,-sqrt(z))
    
