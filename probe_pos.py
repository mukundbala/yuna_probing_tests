import hebi
from math import pi
from math import asin
from time import sleep, time
import os
import numpy as np
import robot_setup
from robot_setup.yunaKinematics import *
import SMCF


def find_compression_center(ground_force: np.ndarray, position: np.ndarray):
    """

    :param ground_force: 4*3 ground force on the end effector base on the body frame
    :param position: 4*3 end effector position base on the body frame
    :return:
    dx: the distance G should move along x
    dy: the distance G should move along y
    """
    total_force = sum(ground_force[:, 2])
    torque_x = sum(ground_force[:, 2] * position[:, 0])
    torque_y = sum(ground_force[:, 2] * position[:, 1])
    gx = torque_x / total_force  # G坐标
    gy = torque_y / total_force  # G坐标
    # 寻找型心
    position = position[[0, 2, 3, 1], :]
    x_g_up = 0
    down = 0
    y_g_up = 0
    for i in range(3):
        x_g_up += position[i][0] ** 2 * position[i + 1][1] + position[i][0] * position[i + 1][0] * position[i + 1][1]
        x_g_up -= position[i + 1][0] ** 2 * position[i][1] + position[i][0] * position[i + 1][1] * position[i][1]

        y_g_up += position[i][0] * position[i + 1][1] ** 2 + position[i][0] * position[i][1] * position[i + 1][1]
        y_g_up -= position[i + 1][0] * position[i][1] ** 2 + position[i][0] * position[i + 1][0] * position[i][1]

        down += position[i][0] * position[i + 1][1] - position[i + 1][0] * position[i][1]

    x_g_up += position[3][0] ** 2 * position[0][1] - position[0][0] ** 2 * position[3][1] + position[3][0] * \
              position[0][0] * position[0][1] - position[3][0] * position[0][1] * position[3][1]
    down += position[3][0] * position[0][1] - position[0][0] * position[3][1]

    y_g_up += position[3][0] * position[0][1] ** 2 - position[0][0] * position[3][1] ** 2 + position[3][0] * \
              position[3][1] * position[0][1] - position[3][0] * position[0][0] * position[3][1]

    return x_g_up / down / 3 - gx, y_g_up / down / 3 - gy


def pose_shift(delta_x, delta_y,positions):
    # body pose shifting
    hexapod.get_next_feedback(reuse_fbk=group_feedback)
    oldpos = group_feedback.position
    curpos = xmk.getLegPositions(np.array([oldpos]))
    curpos[0, 2:] -= delta_x
    curpos[1, 2:] -= delta_y
    curpos[2, 2:] = np.array([-0.320874, -0.32093, -0.306083, -0.292495])  # fixing the z-coord for leg3-6
    newpos = xmk.getLegIK(curpos)
    newpos[:6] = oldpos[:6]

    positions[:, 0] = oldpos
    positions[:, 1] = newpos

    time_vector = [0, 3]

    trajectory = hebi.trajectory.create_trajectory(time_vector, positions)
    duration = trajectory.duration
    start = time()
    t = time() - start

    while t < duration:
        # Serves to rate limit the loop without calling sleep
        hexapod.get_next_feedback(reuse_fbk=group_feedback)
        t = time() - start

        pos, vel, acc = trajectory.get_state(t)
        group_command.position = pos
        hexapod.send_command(group_command)


xmk, imu, hexapod, fbk_imu, fbk_hp = robot_setup.setup_xmonster()
group_command = hebi.GroupCommand(hexapod.size)
hexapod.command_lifetime = 0
pi_ = np.pi

offsets = np.load('/home/mukund/legged_robot/probe_pos/robot_setup/setupFiles/offsets.npy', allow_pickle=True,
                  encoding='latin1')

CF = SMCF.SMComplementaryFilter(offsets)
pose = None
while type(pose) == type(None):
    fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)
    while fbk_imu == None:
        fbk_imu = imu.get_next_feedback(reuse_fbk=fbk_imu)
    CF.update(fbk_imu)
    CF.updateFilter(fbk_imu)
    pose = CF.getBodyPose()

group_feedback = hebi.GroupFeedback(hexapod.size)
group_feedback = hexapod.get_next_feedback(reuse_fbk=group_feedback)

while type(group_feedback) == None:
    group_feedback = hexapod.get_next_feedback(reuse_fbk=group_feedback)
cur_position = group_feedback.position

extend_theta = 30
extend_theta_rad = pi_ * (extend_theta/180)
delta_z = 0.325 * (1 - cos(extend_theta_rad))
extend_back_four = asin(delta_z/0.325)
##positions to reach
setup_pos = np.array(
    [0, 0, -1.57, 
     0, 0, 1.57, 
     0, 0, -1.57, 
     0, 0, 1.57, 
     0, 0, -1.57, 
     0, 0, 1.57])


probe_prep_1 = np.array( ##<base , shoulder , elbow>
    [0, 0, -1.57, ##front left
     0, 0, 1.57,  ##front right
     -5*pi_/18, -pi_/9, -1.57, ##mid left
     5*pi_/18, pi_/9, 1.57,  ##mid right
     0, 0, -1.57, ##back left
     0, 0, 1.57]) ##back right

probe_prep_2 = np.array( ##<base , shoulder , elbow>
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
     0.52,   1.57-extend_theta_rad,  1.57-extend_theta_rad,  ##front right
     -5*pi_/18, 0, -1.57, ##mid left
     5*pi_/18, 0, 1.57,  ##mid right
     0, 0, -1.57, ##back left
     0,0, 1.57]) ##back right


print("Starting Log!")
experiment = 0
log_dir = "/home/mukund/legged_robot/probe_pos/logs"
log_path = os.path.join(log_dir , f"exp_{experiment}")

hexapod.start_log(log_path , 'hexapod.hebilog')
imu.start_log(log_path , 'imu.hebilog')

while True:
    JoyData = input('joystick key: ')
    feedback = hexapod.get_next_feedback(reuse_fbk=group_feedback)
    current_pos = feedback.position
    current_effort = feedback.effort

    if JoyData == 'S': ##S in Setup
        positions = np.zeros((18,4))
        positions[:, 0] = current_pos
        positions[:, 1] = setup_pos
        positions[:, 2] = probe_prep_1
        positions[:, 3] = probe_prep_2
        time_vec = [0,2,3,4]
        trajectory = hebi.trajectory.create_trajectory(time_vec, positions)
        duration = trajectory.duration
        start = time()
        t = time() - start
        while t < duration:
            hexapod.get_next_feedback(reuse_fbk=group_feedback)
            t = time() - start
            pos, vel, acc = trajectory.get_state(t)
            # print(pos)
            group_command.position = pos
            hexapod.send_command(group_command)

    elif JoyData == 'X': ##
        positions = np.zeros((18,2))
        positions[: , 0] = current_pos
        positions[: , 1] = probe_pos
        time_vec = [0,2]
        trajectory = hebi.trajectory.create_trajectory(time_vec, positions)
        duration = trajectory.duration
        start = time()
        t = time() - start
        while t < duration:
            hexapod.get_next_feedback(reuse_fbk=group_feedback)
            t = time() - start
            pos, vel, acc = trajectory.get_state(t)
            # print(pos)
            group_command.position = pos
            hexapod.send_command(group_command)
    
    elif JoyData == "W":
        positions = np.zeros((18,2))
        positions[:,0] = current_pos
        positions[:,1] = probe_extend
        time_vec = [0,2]
        trajectory = hebi.trajectory.create_trajectory(time_vec, positions)
        duration = trajectory.duration
        start = time()
        t = time() - start
        while t < duration:
            hexapod.get_next_feedback(reuse_fbk=group_feedback)
            t = time() - start
            pos, vel, acc = trajectory.get_state(t)
            # print(pos)
            group_command.position = pos
            hexapod.send_command(group_command)

    elif JoyData == "Q":
        positions = np.zeros((18,2))
        positions[:,0] = current_pos
        positions[:,1] = probe_pos
        time_vec = [0,3]
        trajector = hebi.trajectory.create_trajectory(time_vec , positions)
        duration = trajectory.duration
        start = time()
        t = time() - start
        while t < duration:
            hexapod.get_next_feedback(reuse_fbk=group_feedback)
            t = time() - start
            pos, vel, acc = trajectory.get_state(t)
            # print(pos)
            group_command.position = pos
            hexapod.send_command(group_command)

    elif JoyData == "D":
        break

hexapod.stop_log()
print("Hexapod Logging Stopped!")
imu.stop_log()
print("IMU Logging Stopped!")
print("All Logging Stopped!")