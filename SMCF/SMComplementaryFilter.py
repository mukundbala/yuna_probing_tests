import argparse
import numpy as np
import math
from math import pi
from numpy import linalg as LA
import Tools.spinCalcQBottom as psc
from Tools.transforms import *
import sys
import time
from copy import copy
import hebi



class SMComplementaryFilter(object):
    def __init__(self, offsets=None):

        self.numModules = 6 
        self.robotType = "XMonster" 
        self.moduleLength = 0.0639         #[m]
        self.moduleDiameter = 0.0508       #[m]
        robotShape = np.zeros((4,4,7))
        robotShape[0:3,0:3,1] = rotz(pi/6)[0:3,0:3];  # should joint 1
        robotShape[0:3,0:3,2] = rotz(-pi/6)[0:3,0:3]; # should joint 2
        robotShape[0:3,0:3,3] = rotz(pi/2)[0:3,0:3]; # should joint 3
        robotShape[0:3,0:3,4] = rotz(-pi/2)[0:3,0:3]; # should joint 4
        robotShape[0:3,0:3,5] = rotz(5*pi/6)[0:3,0:3]; # should joint 5
        robotShape[0:3,0:3,6] = rotz(-5*pi/6)[0:3,0:3]; # should joint 6
        self.robotShape = copy(robotShape)
        self.axisPerm =  np.identity(3)


        if offsets != None:
            self.gyroOffset = offsets[0]
            self.accelOffset = offsets[1]
            self.torqueOffset = offsets[2]
        else:
            self.accelOffset = np.zeros(3, self.numModules)
            self.gyroOffset = np.zeros(3, self.numModules)
            self.torqueOffset = np.zeros(1, 3*self.numModules)


        self.firstRun = True
        self.everUpdated = False
        self.gyrosTrustability = np.ones(self.numModules)
        self.R = None



    def update(self, fbk):
            self.removeGyroAccelOffset(fbk)

            if self.firstRun:
                    self.previousTime = fbk.receive_time[0]
                    self.firstRun = False
            if (fbk.receive_time[0]-self.previousTime)>0.01:
                    self.updateFilter(fbk)
                    self.everUpdated = True


    def updateFilter(self, fbk):
            # Weight on accelerometer correction term
            # Empirically set value
            accelWeight = .5

            dt = fbk.receive_time[0] - self.previousTime
            dt = max(min(dt, 1), 0.01)

            #########################
            # ACCELEROMETERS UPDATE #
            #########################
            


            accelVecModule = np.vstack((copy(fbk.accelerometer[:,0]), copy(fbk.accelerometer[:,1]), copy(fbk.accelerometer[:,2])))
           
            # Rotate accelerometer vectors into the body frame
            accelVecBody = np.zeros(accelVecModule.shape)

            for i in range(0, self.numModules):
                    accelVecBody[:,i] = np.dot(self.robotShape[0:3,0:3,i+1],accelVecModule[:,i])

            accelVecAvg = np.mean(accelVecBody, axis=1)

            #print(accelVecAvg)
            

            ###############
            # GYROS UPDATE#
            ###############

            gyroVecModule = np.vstack((copy(fbk.gyro[:,0]), copy(fbk.gyro[:,1]), copy(fbk.gyro[:,2])))

            # Rotate gyros into the body frame, taking into account joint angle velocities.
            gyroVecBody = np.zeros(gyroVecModule.shape);
            for i in range(0, self.numModules):
                    gyroVecBody[:,i] = np.matmul(self.robotShape[0:3,0:3,i+1], gyroVecModule[:,i])

            # Average gyros
            gyroVecAvg = np.mean(gyroVecBody[:,self.gyrosTrustability > 0], axis=1)

            #print(gyroVecAvg)

            # TODO REMOVE HACK

            #############################
            # CALCULATE THE ORIENTATION #
            #############################

            # ACCELEROMETER GRAVITY VECTOR

            gravityVec = 1.0*accelVecAvg/LA.norm(accelVecAvg)

            upVec = np.array([[0, 0, 1]])

            if not self.everUpdated:
                    accelAxis = np.cross(upVec, gravityVec)
                    #print(accelAxis)
                    accelAxis = 1.0*accelAxis/LA.norm(accelAxis)

                    accelAngle = math.acos(upVec.dot(gravityVec.T)) # radians
                    self.q = np.real(psc.EV2Q(np.append(accelAxis, accelAngle), tol=1E-6, ichk=False))

            # ESTIMATE NEW ORIENTATION BY FORWARD INTEGRATING GYROS
            w_x = gyroVecAvg[0]
            w_y = gyroVecAvg[1]
            w_z = gyroVecAvg[2]
            q_from_gyros = self.quatRotate(w_x, w_y, w_z, self.q, dt)

            orientDCM = psc.Q2DCM(q_from_gyros, tol=1E-6, ichk=False)
            orientDCM = orientDCM.transpose()

            # gravityVec
            self.accelGrav = np.matmul(orientDCM.T,gravityVec)

            accelAxis = np.cross(upVec, self.accelGrav)
            if not LA.norm(accelAxis) == 0:
                    accelAxis = 1.0*accelAxis/LA.norm(accelAxis)

                    accelAngle = math.acos(upVec.dot(self.accelGrav.T))

                    # MESS W/ THE ACCEL WEIGHT

                    # Scale down if gyro readings are large
                    gyroMag = LA.norm(gyroVecAvg)
                    gyroScale = 1

                    accelWeight = 1.0*accelWeight/(1+gyroScale*gyroMag)

                    # Scale down if accelerometers deviate from 1g.
                    accelMag = LA.norm(accelVecAvg)
                    accelThresh = 1 # 0.1

                    accelDev = math.fabs(accelMag - 9.81) > accelThresh

                    if accelDev:
                            accelWeight = 0.
                    else:
                            accelWeight = accelWeight * (1.0 - accelDev/accelThresh)

                    R_error = np.real(psc.EV2DCM(np.append(-accelAxis, (accelWeight*accelAngle)), tol=1E-6, ichk=False))
                    updatedDCM = np.matmul(R_error.T, orientDCM.T)

            else:
                updatedDCM = orientDCM.T

            #print('nlnr',updatedDCM)
            self.R = updatedDCM
            q = np.real(psc.DCM2Q(updatedDCM, tol=1E-6, ichk=False))
            self.q = psc.Qnormalize(q)
            self.previousTime = copy(fbk.receive_time[0])


    def resetCoordinates(self):
        # resets the frame of reference of the pose
        self.R = np.identity(3)

    def removeGyroAccelOffset(self, fbk):

        #print(fbk.gyro)
        #print(fbk.receive_time[0])

        fbk.gyro[:,0] = fbk.gyro[:,0] - self.gyroOffset[0,:]
        fbk.gyro[:,1] = fbk.gyro[:,1] - self.gyroOffset[1,:] 
        fbk.gyro[:,2] = fbk.gyro[:,2] - self.gyroOffset[2,:]

        fbk.accelerometer[:,0] = fbk.accelerometer[:,0] - self.accelOffset[0,:]
        fbk.accelerometer[:,1] = fbk.accelerometer[:,1] - self.accelOffset[1,:] 
        fbk.accelerometer[:,2] = fbk.accelerometer[:,2] - self.accelOffset[2,:]

    def getBodyPose(self):
        return self.R;

    def getAccelVec(self):
        return self.accelGrav
    

         



    def quatRotate(self, wX, wY, wZ, q, dt=0.001):
            if type(q) is list:
                    q=np.array(q);
            elif type(q) is tuple:
                    q=np.array(q);
            if len(q.shape)==1:
                    if q.shape[0] % 4 == 0:
                            q.shape=[int(q.size/4),4]
                    else:
                            print ("Wrong number of elements")
                            sys.exit(1)
            if q.shape[1] != 4:
                    q.shape=[int(q.size/4),4]

    # Convert q into scalar top fnormormat
            qModified = np.c_[q[0,3], -q[0,0], -q[0,1], -q[0,2]]
            qNew = np.empty_like(q)
            qNew = qModified + dt*0.5*self.quatMultiply(qModified, np.c_[0, wX, wY, wZ])
            psc.Qnormalize(qNew)

            # return qNew after converting back to original format: scalar at bottom
            return np.c_[-qNew[0,1], -qNew[0,2], -qNew[0,3], qNew[0,0]]
            # return qNew

    def quatMultiply(self,q1,q2):
            if type(q1) is list:
                    q1=np.array(q1);
            elif type(q1) is tuple:
                    q1=np.array(q1);
            if len(q1.shape)==1:
                    if q1.shape[0] % 4 == 0:
                            q1.shape=[int(q1.size/4),4]
                    else:
                            print ("Wrong number of elements")
                            sys.exit(1)
            if q1.shape[1] != 4:
                    q1.shape=[int(q1.size/4),4]

            if type(q2) is list:
                    q2=np.array(q2);
            elif type(q2) is tuple:
                    q2=np.array(q2);
            if len(q2.shape)==1:
                    if q2.shape[0] % 4 == 0:
                            q2.shape=[int(q1.size/4),4]
                    else:
                            print ("Wrong number of elements")
                            sys.exit(1)
            if q2.shape[1] != 4:
                    q2.shape=[int(q1.size/4),4]

            qProduct = np.empty_like(q1)
            qProduct[0,0] =  q1[0,0]*q2[0,0] - q1[0,1]*q2[0,1] - q1[0,2]*q2[0,2] - q1[0,3]*q2[0,3]
            qProduct[0,1] =  q1[0,0]*q2[0,1] + q1[0,1]*q2[0,0] + q1[0,2]*q2[0,3] - q1[0,3]*q2[0,2]
            qProduct[0,2] =  q1[0,0]*q2[0,2] + q1[0,2]*q2[0,0] + q1[0,3]*q2[0,1] - q1[0,1]*q2[0,3]
            qProduct[0,3] =  q1[0,0]*q2[0,3] + q1[0,3]*q2[0,0] + q1[0,1]*q2[0,2] - q1[0,2]*q2[0,1]
            # qProduct[0,0] =  q1[0,3]*q2[0,0] - q1[0,2]*q2[0,1] + q1[0,1]*q2[0,2] + q1[0,0]*q2[0,3]
            # qProduct[0,1] =  q1[0,2]*q2[0,0] + q1[0,3]*q2[0,1] - q1[0,0]*q2[0,2] + q1[0,1]*q2[0,3]
            # qProduct[0,2] =  q1[0,1]*q2[0,0] + q1[0,0]*q2[0,1] + q1[0,3]*q2[0,2] + q1[0,2]*q2[0,3]
            # qProduct[0,3] =  q1[0,0]*q2[0,0] - q1[0,1]*q2[0,1] - q1[0,2]*q2[0,2] + q1[0,3]*q2[0,3]

            return qProduct

def decomposeSO3(rotationMatrix):
        thetaX = math.atan2(rotationMatrix[2,1],rotationMatrix[2,2])
        thetaY = math.atan2(-rotationMatrix[2,0],math.hypot(rotationMatrix[2,1],rotationMatrix[2,2]))
        thetaZ = math.atan2(rotationMatrix[1,0],rotationMatrix[0,0])
        return np.array((thetaX, thetaY, thetaZ))




