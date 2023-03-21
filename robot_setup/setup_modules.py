
##what setup_xmonster does:
'''
Establishes connection with hebi modules and creates hexapod and imu groups

imu group: array of 6 actuators attached to the base of the hexapod
hexapod group: array of 18 actuators
'''

import numpy as np
import hebi
from time import sleep


# import rospkg

##WHICH MOTORS DO LX_JX_<BASE/SHOULDER/ELBOW> refer to?
def setup_modules():
    # Get Names of All the Modules
    bases = ['L1_J1_Base', 'L2_J1_Base', 'L3_J1_Base', 'L4_J1_Base', 'L5_J1_Base', 'L6_J1_Base']
    names = ['L1_J1_Base', 'L1_J2_Shoulder', 'L1_J3_Elbow', 'L2_J1_Base', 'L2_J2_Shoulder',
             'L2_J3_Elbow', 'L3_J1_Base', 'L3_J2_Shoulder', 'L3_J3_Elbow', 'L4_J1_Base',
             'L4_J2_Shoulder', 'L4_J3_Elbow', 'L5_J1_Base', 'L5_J2_Shoulder',
             'L5_J3_Elbow', 'L6_J1_Base', 'L6_J2_Shoulder', 'L6_J3_Elbow']

    #Lookup class manages a background discovery process that can find modules on a local network using UDP broadcast messages
    HebiLookup = hebi.Lookup() ##this lookup class discovers the various modules  (motors etc)
    imu = HebiLookup.get_group_from_names(['*'], bases) ##imu consists of the base motors
    hexapod = HebiLookup.get_group_from_names('*', names) ##this selects all the names and puts them into the hexapod group
    while imu == None or hexapod == None or imu.size != 6 or hexapod.size != 18: ##okay, just keep looking up until we find everything
        print('Waiting for modules')
        imu = HebiLookup.get_group_from_names('*', bases)
        hexapod = HebiLookup.get_group_from_names('*', names)

    print('Found {} modules in shoulder group, {} in robot.'.format(imu.size, hexapod.size))

    # Set the Gains (Multiple Times)

    gains_command = hebi.GroupCommand(hexapod.size) ##Group command with
    gains_command.read_gains('./robot_setup/setupFiles/gains18.xml')
    for i in range(3): ##WHY 3 TIMES?
        hexapod.send_command(gains_command)
        sleep(0.1)
    hexapod.command_lifetime = 5
    hexapod.feedback_frequency = 100

    return imu, hexapod ##


if __name__ == "__main__":
    imu, hexapod = setup_modules() ##RETURNS IMU AND HEXAPOD GROUPS



