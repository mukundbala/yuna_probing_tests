import hebi
from math import pi
from math import asin
from time import sleep, time
import os
import numpy as np

log_dir = "/home/mukund/legged_robot/probe_pos/logs"
experiment = "exp_1"

# hexapod_logpath = os.path.join(log_dir,experiment,"hexapod.hebilog")
# imu_logpath = os.path.join(log_dir,experiment,"imu.hebilog")

# hexapod_logfile = hebi.util.load_log(hexapod_logpath)
# imu_logfile = hebi.util.load_log(imu_logpath)


folder = "/home/mukund/legged_robot/probe_pos/logs/exp_0"
hname = "hexapod.hebilog"
iname = "imu.hebilog"

print(folder)
hlog = hebi.util.load_log('{}/{}'.format(folder, hname))
ilog = hebi.util.load_log('{}/{}'.format(folder, iname))

print(np.array(ilog._data)
hebi.util.plot_logs(ilog, fbk_field="L1_J1_Base")
#print(log_file.get_next_feedback)

# hebi.util.plot_logs(log_file , fbk_field="position")



