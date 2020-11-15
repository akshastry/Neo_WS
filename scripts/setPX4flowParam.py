from pymavlink import mavutil, mavparm
import sys

mav = mavutil.mavlink_connection("/dev/ttyACM1", baud=115200)
heartbeat = mav.wait_heartbeat(blocking=True, timeout=3)
if(heartbeat == None):
	sys.exit("Did not get heartbeat....exiting.")
mav.target_system = 81
mav.target_component = 50

param = mavparm.MAVParmDict()
if(param.mavset(mav, "BFLOW_F_THLD", 40) == False):
	sys.exit("Not able to set BFLOW_F_THLD...exiting")
if(param.mavset(mav, "BFLOW_HIST_FIL", 1) == False):
	sys.exit("Not able to set BFLOW_HIST_FIL...exiting")
if(param.mavset(mav, "BFLOW_GYRO_COM", 1) == False):
	sys.exit("Not able to set BFLOW_GYRO_COM...exiting")
if(param.mavset(mav, "BFLOW_LP_FIL", 1) == False):
	sys.exit("Not able to set BFLOW_LP_FIL...exiting")
if(param.mavset(mav, "BFLOW_W_NEW", 1) == False):
	sys.exit("Not able to set BFLOW_W_NEW...exiting")
if(param.mavset(mav, "SONAR_FILTERED", 1) == False):
	sys.exit("Not able to set SONAR_FILTERED...exiting")
if(param.mavset(mav, "USB_SEND_GYRO", 0) == False):
	sys.exit("Not able to set USB_SEND_GYRO...exiting")
if(param.mavset(mav, "USB_SEND_VIDEO", 0) == False):
	sys.exit("Not able to set USB_SEND_VIDEO...exiting")
# if(param.mavset(mav, "SYS_SEND_LPOS", 0) == False):
# 	sys.exit("Not able to set SYS_SEND_LPOS...exiting")
# if(param.mavset(mav, "BFLOW_RATE", 200) == False):
# 	sys.exit("Not able to set BFLOW_RATE...exiting")

mav.close()