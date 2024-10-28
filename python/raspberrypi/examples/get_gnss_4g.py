# -*- coding: utf-8 -*
'''!
  @file get_gnss_4g.py
  @brief Reading gnss data
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

utc = struct_utc_tim()
lat_lon = struct_lat_lon()

user_name = "chwj068746"
user_password = "16409678"
server_addr = "119.3.136.126"
mount_point = "RTCM33"
port = 8002

def setup():
  while (rtk.begin() == False):
    print("Sensor initialize failed!!")
    time.sleep(1)
  print("Sensor initialize success!!")
  rtk.set_module(MODULE_4G)
  
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

  if rtk.get_data_flush():
    utc = rtk.get_date()
    utc = rtk.get_utc()
    lat_lon = rtk.get_lat()
    lat_lon = rtk.get_lon()
    alt = rtk.get_alt()
    num = rtk.get_num_sta_used()
    hdop = rtk.get_hdop()
    sep = rtk.get_sep()
    quality = rtk.get_quality()
    siteid = rtk.get_site_id()
    diftime = rtk.get_dif_time() 
    
    print(str(utc.year) + "/" + str(utc.month) + "/" + str(utc.date) + "/" + str(utc.hour) + ":" + str(utc.minute) + ":" + str(utc.second))
    print("star used number = " + str(num))
    #print("latutide DDMM.MMMMM  = " + str(lat_lon.latitude) + "   direction = " + lat_lon.lat_direction)
    #print("lonutide DDDMM.MMMMM = " + str(lat_lon.lonitude) + "   direction = " + lat_lon.lon_direction)
    print("latutide degree = " + str(lat_lon.latitude_degree) + "   direction = " + lat_lon.lat_direction)
    print("lonutide degree = " + str(lat_lon.lonitude_degree) + "   direction = " + lat_lon.lon_direction)
    print("alt = " + str(alt))
    print("hdop = " + str(hdop))
    print("quality = " + str(quality))
    print("site id  = " + str(siteid))
    print("diftime = " + str(diftime))
    print("")
    print("gga = " + str(rtk.get_gnss_message(GNGGA_MODE)))
    print("rmc = " + str(rtk.get_gnss_message(GNRMC_MODE)))
    print("gll = " + str(rtk.get_gnss_message(GNGLL_MODE)))
    print("vtg = " + str(rtk.get_gnss_message(GNVTG_MODE)))
  if not rtk.get_connect_state():
    print("restart connect .....")
    rtk.reconnect()
  time.sleep(0.5)

if __name__ == "__main__":
  setup()
  while True:
    loop()
