from pymavlink import mavutil, mavparm
import sys

mav = mavutil.mavlink_connection("/dev/ttyACM1", baud=115200)
heartbeat = mav.wait_heartbeat(blocking=True, timeout=3)
if(heartbeat == None):
	sys.exit("Did not get heartbeat....exiting.")
mav.target_system = 81
mav.target_component = 50

param = mavparm.MAVParmDict()
print(param.mavset(mav, "BFLOW_F_THLD", 40))

mav.close()