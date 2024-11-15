# -*- coding: utf-8 -*
'''!
  @file get_gnss.py
  @brief Reading all gnss data
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [ZhixinLiu](zhixin.liu@dfrobot.com)
  @version    V1.0
  @date       2024-10-28
  @url https://github.com/DFRobot/DFRobot_RTK_4G
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")
import time
from DFRobot_RTK_4G import *

'''
  Select to use i2c or UART
  I2C_MODE
  UART_MODE
'''
ctype = I2C_MODE

if ctype == I2C_MODE:
  I2C_1 = 0x01
  rtk = DFRobot_RTK_4G_I2C (I2C_1, DEVICE_ADDR)
elif ctype == UART_MODE:
  rtk = DFRobot_RTK_4G_UART(115200)

user_name = "chwj163776"
user_password = "40497291"
server_addr = "119.3.136.126"
mount_point = "RTCM33"
port = 8002

def setup():
  while (rtk.begin() == False):
    print("Sensor initialize failed!!")
    time.sleep(1)
  print("Sensor initialize success!!")
  rtk.set_module(MODULE_4G)
  time.sleep(1)
  rtk.set_user_name(user_name)
  rtk.set_user_password(user_password)
  rtk.set_server_addr(server_addr)
  rtk.set_mount_point(mount_point)
  rtk.set_port(port)
  print("please wait 4g module init !")
  print("connecting network please wait !")
  result = rtk.connect()
  if result == CONNECT_SUCCESS:
    print(CONNECT_SUCCESS)
  else:
    print(result)
  
def loop():
  rslt = rtk.get_all_gnss()
  if len(rslt) != 0:
    print(rslt)
  if not rtk.get_connect_state():
    print("restart connect .....")
    rtk.reconnect()
    time.sleep(0.5)

if __name__ == "__main__":
  setup()
  while True:
    loop()
