import hebi
import pybullet as p
import pybullet_data
from math import pi
from time import sleep, time
import numpy as np
import robot_setup
from robot_setup.yunaKinematics import *
import SMCF


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

##utility functions
def jointToBullet(joints):
    bullet = np.empty(18)
    bullet[:3] = joints[0:3] ##front left
    bullet[9:12] = joints[3:6] ##front right
    bullet[3:6]  = joints[6:9] ##mid left
    bullet[12:15] = joints[9:12] ##mid right
    bullet[6:9] = joints[12:15] ##back left
    bullet[15:] = joints[15:] ##back right
    return bullet

def bulletToJoint(bullet):
    joints = np.empty(18)
    joints[0:3] = bullet[:3] ##front left
    joints[3:6] = bullet[9:12]  ##front right
    joints[6:9] = bullet[3:6]   ##mid left
    joints[9:12] = bullet[12:15]  ##mid right
    joints[12:15] = bullet[6:9]  ##back left
    joints[15:] = bullet[15:]  ##back right
    return joints

def getJoints():
    current = np.empty(18)
    curr_id = 0
    for id in actuators:
        current[curr_id] = p.getJointState(Yuna , id)[0]
        curr_id +=1
    return bulletToJoint(current)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF (plane)
planeId = p.loadURDF("plane.urdf")
p.setGravity(0,0,-10)

YunaStartPos = [0,0,0.5]
YunaStartOrientation = p.getQuaternionFromEuler([0,0,0])
Yuna = p.loadURDF("urdf/yuna.urdf",YunaStartPos, YunaStartOrientation)
joint_num = p.getNumJoints(Yuna)

actuators = [i for i in range(joint_num) if p.getJointInfo(Yuna,i)[2] != p.JOINT_FIXED]

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

xmk = HexapodKinematics()
pi_ = np.pi

extend_theta = 30
extend_theta_rad = pi_ * (extend_theta/180)

setup_pos = np.array(
    [0, 0, -1.57, 
     0, 0, 1.57, 
     0, 0, -1.57, 
     0, 0, 1.57, 
     0, 0, -1.57, 
     0, 0, 1.57])

probe_prep = np.array( ##<base , shoulder , elbow>
    [0, 0, -1.57, ##front left
     0, 0, 1.57,  ##front right
     -5*pi_/18, 0, -1.57, ##mid left
     5*pi_/18, 0, 1.57,  ##mid right
     0, 0, -1.57, ##back left
     0, 0, 1.57]) ##back right

probe_pos = np.array( ##<base , shoulder , elbow>
    [-0.52, -1.57, -1.57, ##front left
     0.52, 1.57, 1.57,  ##front right
     -5*pi_/18, 0, -1.57, ##mid left
     5*pi_/18, 0, 1.57,  ##mid right
     0, 0, -1.57, ##back left
     0, 0, 1.57]) ##back right

probe_extend = np.array( ##<base , shoulder , elbow>
    [-0.52, -1.57+extend_theta_rad, -1.57+extend_theta_rad, ##front left
     0.52, 1.57-extend_theta_rad, 1.57-extend_theta_rad,  ##front right
     -5*pi_/18, extend_theta_rad, -1.57+extend_theta_rad, ##mid left
     5*pi_/18, -extend_theta_rad, 1.57-extend_theta_rad,  ##mid right
     0, extend_theta_rad, -1.57+extend_theta_rad, ##back left
     0, -extend_theta_rad, 1.57-extend_theta_rad]) ##back right


target = None
state = "SETUP"  #SETUP , PROBE_PREP . PROBE_POS . PROBE_EXTEND . PROBE_POS
step = 1
max_step = 50
current = getJoints()

p.setRealTimeSimulation(1)
while True:
    current  = getJoints()
    forces = [60.] * len(actuators)
    if state == "SETUP":
        target = setup_pos
        state = "PROBE_PREP"
    
    elif state == "PROBE_PREP":
        target = probe_prep
        state = "PROBE_POS"
    
    elif state == "PROBE_POS":
        target = probe_pos
        state = "PROBE_EXTEND"
    
    elif state == "PROBE_EXTEND":
        target = probe_extend
        state = "PROBE_POS"
    
    step_dist = target - current ##both follow our convention
    step_dist /= max_step

    while step <= max_step:
        command = np.add(current , step_dist * step)
        command = jointToBullet(command)
        p.setJointMotorControlArray(Yuna, actuators, controlMode=p.POSITION_CONTROL, targetPositions=command,
                                 positionGains=[0.5]*len(actuators),velocityGains=[1]*len(actuators),forces=forces)
        step+=1
        sleep(0.1)
    
    step = 1 ##reset the step

    p.setJointMotorControlArray(Yuna, actuators, controlMode=p.POSITION_CONTROL, targetPositions=jointToBullet(target),
                            positionGains=[0.5]*len(actuators),velocityGains=[1]*len(actuators),forces=forces)


p.disconnect()


