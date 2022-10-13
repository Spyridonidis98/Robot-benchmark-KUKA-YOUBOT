from controller import Robot 
import numpy as np 
from numpy import sin,cos, pi
import sys

################# numpy matmul and matrix inverse dosent work in robot benchmark if we use float numbers so we write functions for this job 

def matinv(m):
    if(len(m)!=3):
        sys.exit("can only find inverse of 3*3 matrix")
    m = np.array(m)
    
    det = m[0][0]*(m[1][1]*m[2][2]-m[2][1]*m[1][2]) - m[0][1]*(m[1][0]*m[2][2]-m[2][0]*m[1][2]) + m[0][2]*(m[1][0]*m[2][1]-m[2][0]*m[1][1])
    if(det==0):
        sys.exit("det 0 matrix dosent exist")

    inv = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])
    inv[0,0] = m[1,1]*m[2,2]-m[2,1]*m[1,2]
    inv[0,1] = m[0,2]*m[2,1]-m[2,2]*m[0,1]
    inv[0,2] = m[0,1]*m[1,2]-m[1,1]*m[0,2]
    inv[1,0] = m[1,2]*m[2,0]-m[2,2]*m[1,0]
    inv[1,1] = m[0,0]*m[2,2]-m[2,0]*m[0,2]
    inv[1,2] = m[0,2]*m[1,0]-m[1,2]*m[0,0]
    inv[2,0] = m[1,0]*m[2,1]-m[2,0]*m[1,1]
    inv[2,1] = m[0,1]*m[2,0]-m[2,1]*m[0,0]
    inv[2,2] = m[0,0]*m[1,1]-m[1,0]*m[0,1]
    return inv/det

def matmul(A,B):
    A = np.array(A)
    B = np.array(B)
    if(len(B.shape)==1):
        return np.matmul(A,B)

    rowsA  = A.shape[0]
    colsA = A.shape[1]
    
    rowsB  = B.shape[0]
    colsB = B.shape[1]

    if(colsA != rowsB):
        sys.exit("matrix dimention dosent match") 
    
    C = []
    for i in range(rowsA):
        c = []
        for j in  range(colsB):
            c.append(np.dot(A[i,:],B[:,j]))
        C.append(c)
    return np.array(C)

def DHMat(th, d, r, a):
    return np.array([[cos(th), -sin(th)*cos(a), sin(th)*sin(a), r*cos(th)],
                     [sin(th),  cos(th)*cos(a), -cos(th)*sin(a), r* sin(th)],
                     [0 , sin(a), cos(a), d],
                     [0,0,0,1]])


robot = Robot()
timestep = int(robot.getBasicTimeStep())

wheels = [] 
wheels.append(robot.getMotor("wheel1"))
wheels.append(robot.getMotor("wheel2"))
wheels.append(robot.getMotor("wheel3"))
wheels.append(robot.getMotor("wheel4"))

armMotors = []
armMotors.append(robot.getMotor("arm1"))
armMotors.append(robot.getMotor("arm2"))
armMotors.append(robot.getMotor("arm3"))
armMotors.append(robot.getMotor("arm4"))
armMotors.append(robot.getMotor("arm5"))
maxVel =armMotors[0].getMaxVelocity()
for arm in armMotors:
    arm.setVelocity(maxVel)

armPositionSensors = []
armPositionSensors.append(robot.getPositionSensor("arm1sensor"))
armPositionSensors.append(robot.getPositionSensor("arm2sensor"))
armPositionSensors.append(robot.getPositionSensor("arm3sensor"))
armPositionSensors.append(robot.getPositionSensor("arm4sensor"))
armPositionSensors.append(robot.getPositionSensor("arm5sensor"))
for sensor in armPositionSensors:
    sensor.enable(timestep)

finger1 = robot.getMotor("finger1")
finger2 = robot.getMotor("finger2")
finger1.setVelocity(0.03)
finger2.setVelocity(0.03)

posb_x = -2.4
posb_z = 0
posb_thy = 0
ub_x = 0
ub_z = 0
ub_th = 0

def TWB(xb,zb,thb):
        return np.array([[cos(thb),0, sin(thb), xb],
                        [0, 1, 0, 0.1],
                        [-sin(thb), 0, cos(thb),zb],
                        [0,0,0,1]])

def TWE(xb,zb,thb,th1,th2,th3,th4,th5):
        l1  = 0.072
        l2 = 0.075
        l3 = 0.155
        l4 = 0.135
        l5 = 0.081
        l6 = 0.1365
        Twb = TWB(xb,zb,thb)
        Tb0 = DHMat(0,0,0.19,-pi/2)
        T01 = DHMat(th1, l1+l2, 0, pi/2)
        T12 = DHMat(th2+pi/2, 0, l3, 0)
        T23 = DHMat(th3, 0, l4, 0)
        T34 = DHMat(th4-pi/2, 0, 0,0 -pi/2)
        T45 = DHMat(th5, l5+l6, 0, pi/2)
        Tw0 = matmul(Twb, Tb0)
        Tw1 = matmul(Tw0, T01)
        Tw2 = matmul(Tw1, T12)
        Tw3 = matmul(Tw2, T23)
        Tw4 = matmul(Tw3, T34)
        Tw5 = matmul(Tw4, T45)
        return Tw5

def moveBase(uxb, uzb, uthb):
    global posb_x, posb_z, posb_thy,ub_x,ub_z,ub_th
    
    Twb = TWB(posb_x,posb_z,posb_thy)[0:3, 0:3]
    Tbw = matinv(Twb)
    vec = matmul(Tbw, np.array([uxb, 0, uzb]))
    uxb = vec[0]
    uzb = vec[2]

    if(abs(ub_x)<abs(uxb)):
        if(uxb>0):
            ub_x+=0.01
            uxb = ub_x
        else:
            ub_x-=0.01
            uxb = ub_x
    else:
        ub_x = uxb

    if(abs(ub_z)<abs(uzb)):
        if(uzb>0):    
            ub_z+=0.01
            uzb = ub_z
        else:
            ub_z-=0.01
            uzb = ub_z
    else:
        ub_z = uzb

    if(abs(ub_th)<abs(uthb)):
        if(uthb>0):  
            ub_th+=0.01
            uthb = ub_th
        else:
            ub_th-=0.01
            uthb = ub_th
    else:
        ub_th = uthb

    max_wheel_vel = wheels[0].getMaxVelocity()
    uthw1 = 0
    uthw2 = 0
    uthw3 = 0
    uthw4 = 0
    
    uthw1 += uthb
    uthw2 -= uthb
    uthw3 += uthb
    uthw4 -= uthb
    
    meters_to_rad = 3.183098861837906715 * 6.283185307179586476925286766559
    uradxb = meters_to_rad*uxb

    uthw1 += uradxb
    uthw2 += uradxb
    uthw3 += uradxb
    uthw4 += uradxb

    uradzb = meters_to_rad*uzb
    uthw1 -= uradzb
    uthw2 += uradzb
    uthw3 += uradzb
    uthw4 -= uradzb


    uthw = np.array([uthw1, uthw2, uthw3, uthw4])
    ratio = 1
    if(max(abs(uthw))>max_wheel_vel):
        ratio *= max_wheel_vel/max(abs(uthw)+0.01)
        uthw=uthw*ratio
        uxb*=ratio
        uzb*=ratio
        
    for i in range(len(uthw)):
        wheels[i].setVelocity(abs(uthw[i]))
        if(uthw[i]>0):
            wheels[i].setPosition(10000)  
        else:
            wheels[i].setPosition(-10000)


    uthb/= 10.35
    uzb/=1.444

    vec = matmul(Twb, [uxb, 0,uzb])
    uxb = vec[0]
    uzb = vec[2]

    posb_x+= uxb*timestep/1000
    posb_z+= uzb*timestep/1000
    posb_thy+= uthb*timestep/1000

    
def basegoto(x,z,thy):
    global robot, posb_x, posb_z, posb_thy
    target = np.array([x, z, thy])
    k = np.array([2.0,5,10])
    while(True):
        pos = np.array([posb_x, posb_z, posb_thy])
        e = target-pos
        if(np.linalg.norm(e)<0.001):
            break
        moveBase(k[0]*e[0], k[1]*e[1], k[2]*e[2])
        robot.step(timestep)
    for wheel in wheels:
        wheel.setVelocity(0)  

def JacobianArm(xb,zb,thb,th1,th2,th3,th4,th5):
        l1  = 0.072
        l2 = 0.075
        l3 = 0.155
        l4 = 0.135
        l5 = 0.081
        l6 = 0.137
        Twb = TWB(xb,zb,thb)
        Tb0 = DHMat(0,0,0.19,-pi/2)
        T01 = DHMat(th1, l1+l2, 0, pi/2)
        T12 = DHMat(th2+pi/2, 0, l3, 0)
        T23 = DHMat(th3, 0, l4, 0)
        T34 = DHMat(th4-pi/2, 0, 0,0 -pi/2)
        T45 = DHMat(th5, l5+l6, 0, pi/2)

        pw = np.array([0,0,0,1])
        
        Tw0 = matmul(Twb, Tb0)
        p0 = matmul(Tw0, pw)
        

        Tw1 = matmul(Tw0, T01)
        p1 = matmul(Tw1, pw)

        Tw2 = matmul(Tw1, T12)
        p2  = matmul(Tw2, pw)
        
        Tw3  = matmul(Tw2, T23)
        p3 = matmul(Tw3, pw)

        Tw4 = matmul(Tw3, T34)
        p4  = matmul(Tw4, pw)
         
        Tw5 = matmul(Tw4, T45)
        p5 = matmul(Tw5, pw)

        zw = np.array([0.0,0.0,1.0])

        z0 = matmul(Tw0[0:3,0:3],zw)
        jc3 = np.cross(z0, p5[0:3]-p0[0:3])
        #jc3 = np.concatenate((jc3, z0))
        jc3  = np.transpose([jc3])

        z1 = matmul(Tw1[0:3,0:3],zw)
        jc4 = np.cross(z1, p5[0:3]-p1[0:3])
        #jc4 = np.concatenate((jc4, z1))
        jc4  = np.transpose([jc4])

        z2 = matmul(Tw2[0:3,0:3],zw)
        jc5 = np.cross(z2, p5[0:3]-p2[0:3])
       #jc5 = np.concatenate((jc5, z2))
        jc5  = np.transpose([jc5])

        z3 = matmul(Tw3[0:3,0:3],zw)
        jc6 = np.cross(z3, p5[0:3]-p3[0:3])
        #jc6 = np.concatenate((jc6, z3))
        jc6  = np.transpose([jc6])

        z4 = matmul(Tw4[0:3,0:3],zw)
        jc7 = np.cross(z4, p5[0:3]-p4[0:3])
        #jc7 = np.concatenate((jc7, z4))
        jc7  = np.transpose([jc7])

        return np.concatenate((jc3, jc4, jc5,jc6,jc7), axis=1)

def JacobianPlusArm(xb,zb, thb,th1,th2,th3,th4,th5):
        J = JacobianArm(xb,zb,thb,th1,th2,th3,th4,th5)
        return matmul(np.transpose(J),matinv(matmul(J,np.transpose(J))))

def moveJoints(uth1,uth2,uth3,uth4,uth5):
        max_uth = armMotors[0].getMaxVelocity()
        uth = np.array([uth1, uth2, uth3, uth4, uth5])
        if(max(abs(uth))>max_uth):
            uth = uth*max_uth/max(abs(uth)+0.001)
        
        for i in range(len(armMotors)):
            if uth[i] >= 0:
                armMotors[i].setPosition(armMotors[i].getMaxPosition())
            else:
                armMotors[i].setPosition(armMotors[i].getMinPosition())

        for i in range(len(armMotors)):
            armMotors[i].setVelocity(abs(uth[i]))

def armgoto(x,y,z):
    global posb_x, posb_z, posb_thy
    q = np.zeros(8)
    pw = np.array([0,0,0,1.0])
    target = np.array([x,y,z])
    moveJoints(0,-1.0,-1.0,-1.0,0)
    robot.step(timestep)
    while(True):
        robot.step(timestep)
        q[0] = posb_x
        q[1] = posb_z
        q[2] = posb_thy
        q[3] = armPositionSensors[0].getValue()
        q[4] = armPositionSensors[1].getValue()
        q[5] = armPositionSensors[2].getValue()
        q[6] = armPositionSensors[3].getValue()
        q[7] = armPositionSensors[4].getValue()
        
        Twe = TWE(q[0], q[1], q[2], q[3], q[4],q[5], q[6], q[7])
        pos = matmul(Twe,pw)[0:3]
        e = target - pos
        k= 10
        if(np.linalg.norm(e)<0.01):
            break
        Jplus = JacobianPlusArm(q[0], q[1], q[2], q[3], q[4],q[5], q[6], q[7])
        uq = matmul(Jplus, k*e)
        moveJoints(uq[0], uq[1],uq[2], uq[3], uq[4])
    for arm in armMotors:
        arm.setVelocity(0)
def armOpen():
    fingerMaxPosition = finger1.getMaxPosition()
    finger1.setPosition(fingerMaxPosition)
    finger2.setPosition(fingerMaxPosition)
    robot.step(20*timestep)

def armClose():
    fingerMinPosition = finger1.getMinPosition()
    finger1.setPosition(fingerMinPosition)
    finger2.setPosition(fingerMinPosition)
    robot.step(20*timestep)


basegoto(0.5,0,0)
armOpen()
armgoto(1,0.205,0)
armClose()
armgoto(1,0.505,0)
basegoto(0.5,0,pi)
basegoto(-1.8,-0.813,pi)
armgoto(-2.18,0.305,-0.813)
armOpen()

        

        





    

        