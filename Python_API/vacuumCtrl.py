
# 7Bot Robotic Arm Example 7: Robot End-effector Vacuum Contrl

# Date:    July 20th, 2020
# Author:  Jerry Peng
 
import time 
from lib.Arm7Bot import Arm7Bot


# assign serial port to 7Bot. 
# ATTENTION: Change the parameter "/dev/cu.SLAB_USBtoUART" below 
#            according to your own computer OS system. Open whatever port is 7Bot on your computer.
# Usually:  "/dev/cu.SLAB_USBtoUART" on Mac OS
#           "/dev/ttyUSB0" on Linux
#           'COM1' on Windows
arm = Arm7Bot("/dev/cu.SLAB_USBtoUART") 


# 1.turn on vacuum  
arm.setVacuum(1)
time.sleep(3)

# 2.turn off vacuum  
arm.setVacuum(0)

