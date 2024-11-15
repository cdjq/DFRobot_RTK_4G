# -*- coding: utf-8 -*
'''!
  @file DFRobot_RTK_4G.py
  @brief DFRobot_GNSS Class infrastructure, implementation of underlying methods
  @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license The MIT License (MIT)
  @author [ZhixinLiu](zhixin.liu@dfrobot.com)
  @version V1.0
  @date 2024-08-14
  @url https://github.com/DFRobot/DFRobot_RTK_4G
'''
import sys
import serial
import time
import smbus

REG_YEAR_H 		= 0
REG_YEAR_L 		= 1
REG_MONTH 		= 2
REG_DATE  		= 3
REG_HOUR  		= 4
REG_MINUTE 		= 5
REG_SECOND 		= 6
REG_LAT_1 		= 7
REG_LAT_2 		= 8
REG_LAT_X_24 	= 9
REG_LAT_X_16 	= 10
REG_LAT_X_8  	= 11
REG_LAT_DIS  	= 12
REG_LON_1 		= 13
REG_LON_2 		= 14
REG_LON_X_24 	= 15
REG_LON_X_16 	= 16
REG_LON_X_8  	= 17
REG_LON_DIS  	= 18
REG_GPS_STATE	= 19
REG_USE_STAR 	= 20
REG_HDOP_Z		= 21
REG_HDOP_X		= 22
REG_ALT_H 		= 23
REG_ALT_L 		= 24
REG_ALT_X 		= 25
REG_SEP_H 		= 26
REG_SEP_L 		= 27
REG_SEP_X 		= 28
REG_DIF_Z			= 29
REG_DIF_X			= 30
REG_DIFID_H		= 31
REG_DIFID_L		= 32
REG_I2C_ID    = 50     # uart device id

REG_DATA_FLUSH= 80
REG_GGA_LEN		= 81
REG_GGA_ALL		= 82
REG_RMC_LEN		= 83
REG_RMC_ALL		= 84
REG_GLL_LEN		= 85
REG_GLL_ALL		= 86
REG_VTG_LEN		= 87
REG_VTG_ALL		= 88
REG_ALL_LEN_H	= 89
REG_ALL_LEN_L	= 90
REG_ALL				= 91
REG_ALL_MODE	= 92
REG_OPERATION	= 93
REG_LORA_BAUD	= 94
REG_4G_BAUD 	= 95
REG_UNO_BAUD	= 96
REG_TRANS_AT	= 97
REG_T_AT_LEN	= 98
REG_TRANMIT 	= 99
REG_RECV_AT	  = 100
REG_R_AT_LEN	= 101
REG_USER_NAME	= 102
REG_USER_NAME_LEN	= 103
REG_USER_PASSWORD	= 104
REG_USER_PASSWORD_LEN	= 105
REG_SERVER_ADDR	= 106
REG_SERVER_ADDR_LEN	= 107
REG_MOUNT_POINT	= 108
REG_MOUNT_POINT_LEN	= 109
REG_PORT_H	= 110
REG_PORT_L	= 111
REG_CONNECT	= 112
REG_CONNECT_STATE	= 113
I2C_MODE  = 0x01
UART_MODE = 0x02
CONNECT_SUCCESS = "CONNECT SUCCESSFUL"
CONNECT_TIMEOUT = "TIMER OUT"
CONNECT_ERROR = "CONNECT ERROR"
DEVICE_ADDR = 0x20
BAUD_2400 = 0
BAUD_4800 = 1
BAUD_9600 = 2
BAUD_14400 = 3
BAUD_19200 = 4
BAUD_38400 = 5
BAUD_56000 = 6
BAUD_57600 = 7
BAUD_115200 = 8
BAUD_256000 = 9
BAUD_512000 = 10
BAUD_921600 = 11
MODULE_4G   = 20
MODULE_LORA = 10
GNGGA_MODE = 0
GNRMC_MODE = 1
GNGLL_MODE = 2
GNVTG_MODE = 3
CONNECT_START = 0xaa
CONNECT_SUC   = 0x55


class struct_utc_tim:
  def __init__(self):
    self.year=2000
    self.month=1
    self.date=1
    self.hour=0
    self.minute=0
    self.second=0

class struct_lat_lon:
  def __init__(self):
    self.lat_dd = 0
    self.lat_mm = 0
    self.lat_mmmmm = 0
    self.lat_direction = "S"
    self.latitude_degree = 0.00
    self.latitude = 0.00
    self.lon_ddd = 0
    self.lon_mm = 0
    self.lon_mmmmm = 0
    self.lon_direction = "W"
    self.lonitude = 0.00
    self.lonitude_degree = 0.00

utc = struct_utc_tim()
lat_lon = struct_lat_lon()


class DFRobot_RTK_4G(object):
  __m_flag   = 0                # mode flag
  __count    = 0                # acquisition count    
  __txbuf        = [0]          # i2c send buffer
  __uart_i2c     =  0
  ERROR_COUNT     = 0X05
  def __init__(self, bus, Baud):
    if bus != 0:
      self.i2cbus = smbus.SMBus(bus)
      self.__uart_i2c = I2C_MODE
    else:
      self.ser = serial.Serial("/dev/ttyAMA0", baudrate=Baud,stopbits=1, timeout=0.5)
      self.__uart_i2c = UART_MODE
      if self.ser.isOpen == False:
        self.ser.open()
    self._connect = 0
    self._connect_count = 0

  def begin(self):
    '''!
      @brief Init sensor 
      @return True or False
    '''
    if self.__uart_i2c == UART_MODE:
      rslt = self.read_reg(REG_I2C_ID, 1)
      if rslt[0] == DEVICE_ADDR:
        return True
      else:
        return False
    else:
      try:
        rslt = self.read_reg(REG_I2C_ID, 1)
        if rslt[0] == DEVICE_ADDR:
          return True
        else:
          return False
      except:
        return False    
     
    
  def get_data_flush(self):
    '''!
      @brief get_data_flush 
      @return True or False
    '''
    rslt = self.read_reg(REG_DATA_FLUSH, 1)
    if rslt[0] != 0:
      return True
    else:
      return False
    
  def get_date(self):
    '''!
      @brief Get date information, year, month, day 
      @return struct_utc_tim type, represents the returned year, month, day
    '''
    rslt = self.read_reg(REG_YEAR_H, 4)
    if rslt != -1:
      utc.year = rslt[0]*256 + rslt[1]
      utc.month = rslt[2]
      utc.date = rslt[3]
    return utc

  def get_utc(self):
    '''!
      @brief Get time information, hour, minute second 
      @return struct_utc_tim type, represents the returned hour, minute, second 
    '''
    rslt = self.read_reg(REG_HOUR, 3)
    if rslt != -1:
      utc.hour = rslt[0]
      utc.minute = rslt[1]
      utc.second = rslt[2]
    return utc

  def get_lat(self):
    '''!
      @brief Get latitude 
      @return struct_lat_lon type, represents the returned latitude 
    '''
    rslt = self.read_reg(REG_LAT_1, 6)
    if rslt != -1:
      lat_lon.lat_dd = rslt[0]
      lat_lon.lat_mm = rslt[1]
      lat_lon.lat_mmmmm = rslt[2]*65536 + rslt[3]*256 + rslt[4]
      lat_lon.lat_direction = chr(rslt[5])
      lat_lon.latitude = lat_lon.lat_dd*100.0 + lat_lon.lat_mm + lat_lon.lat_mmmmm/100000.0
      lat_lon.latitude_degree = lat_lon.lat_dd + lat_lon.lat_mm/60.0 + lat_lon.lat_mmmmm/100000.0/60.0
      lat_lon.latitude_degree = round(lat_lon.latitude_degree, 7)
    return lat_lon

  def get_lon(self):
    '''!
      @brief Get longitude 
      @return struct_lat_lon type, represents the returned longitude 
    '''
    rslt = self.read_reg(REG_LON_1, 6)
    if rslt != -1:
      lat_lon.lon_ddd = rslt[0]
      lat_lon.lon_mm = rslt[1]
      lat_lon.lon_mmmmm = rslt[2]*65536 + rslt[3]*256 + rslt[4]
      lat_lon.lon_direction = chr(rslt[5])
      lat_lon.lonitude = lat_lon.lon_ddd*100.0 + lat_lon.lon_mm + lat_lon.lon_mmmmm/100000.0
      lat_lon.lonitude_degree = lat_lon.lon_ddd + lat_lon.lon_mm/60.0 + lat_lon.lon_mmmmm/100000.0/60.0
      lat_lon.lonitude_degree = round(lat_lon.lonitude_degree, 7)
    return lat_lon

  def get_num_sta_used(self):
    '''!
      @brief Get the number of the used satellite used
      @return uint8_t type, represents the number of the used satellite
    '''
    rslt = self.read_reg(REG_USE_STAR, 1)
    if rslt != -1:
      return rslt[0]
    else:
      return 0

  def get_alt(self):
    '''!
      @brief Altitude information
      @return double type, represents altitude 
    '''
    sign = 1
    rslt = self.read_reg(REG_ALT_H, 3)
    if rslt != -1:
      if rslt[0] & 0x80:
        rslt[0] &= 0x7F
        sign = -1
      high = (rslt[0]*256 + rslt[1] + rslt[2]/100.0)*sign
    else:
      high = 0.0
    return high

  def get_sep(self):
    '''!
      @brief At the height of geoid
      @return Float data(unit: degree)
    '''
    sign = 1.0
    rslt = self.read_reg(REG_SEP_H, 3)
    if rslt[0] & 0x80:
      rslt[0] &= 0x7F
      sign = -1.0
    high = float((int(rslt[0]) << 8) | rslt[1]) + float(rslt[2]) / 100.0
    return high*sign
    
  def get_hdop(self):
    '''!
      @brief Indicates the horizontal accuracy of positioning
      @return double
    '''
    rslt = self.read_reg(REG_HDOP_Z, 2)
    hdop = float(rslt[0]) + float(rslt[1]) / 100.0
    return hdop

  def get_quality(self):
    '''!
      @brief get message Quality 
      @return message Quality
    '''
    rslt = self.read_reg(REG_GPS_STATE, 1)
    return rslt[0]

  def get_site_id(self):
    '''!
      @brief get site id
      @return site id
    '''
    rslt = self.read_reg(REG_DIFID_H, 2)
    return rslt[0]*256 | rslt[1]
  
  def get_dif_time(self):
    '''!
      @brief The number of seconds in which a differential signal was last received
      @return differential time
    '''
    rslt = self.read_reg(REG_DIF_Z, 2)
    time = float(rslt[0]) + float(rslt[1]) / 100.0
    return time

  def get_module(self):
    '''!
      @brief Get the Module run mode
      @return mode
    '''
    time.sleep(0.1)
    rslt = self.read_reg(REG_OPERATION, 1)
    return rslt[0]
      
  def set_module(self, mode):
    '''!
      @brief Set the Module
      @param mode 4G or lora
    '''
    if mode == self.get_module():
      pass
    else:
      self.__txbuf[0] = mode
      self.write_reg(REG_OPERATION, self.__txbuf)
      if mode == MODULE_4G:
        time.sleep(2)
      time.sleep(1)
      
  def set_module_baud(self, baud):
    '''!
      @brief Set the Module Baud rate
      @param baud rate
    '''
    if baud > BAUD_921600:
       raise ValueError("Invalid baud rate error.")
    self.__txbuf[0] = baud
    self.write_reg(REG_UNO_BAUD, self.__txbuf)


  def set_lora_baud(self, baud):
    '''!
      @brief Set the recevie Lora Baud rate
      @param baud rate
    '''
    raise ValueError("Configuration not supported.")
    #self.__txbuf[0] = baud
    #self.write_reg(REG_LORA_BAUD, self.__txbuf)
    
  def baud_match(self, baud):
    '''!
      @brief baud rate match
      @return baud rate
    '''
    if baud == BAUD_2400:
      return 2400
    elif baud == BAUD_4800:
      return 4800
    elif baud == BAUD_9600:
      return 9600
    elif baud == BAUD_14400:
      return 14400
    elif baud == BAUD_19200:
      return 19200
    elif baud == BAUD_38400:
      return 38400
    elif baud == BAUD_56000:
      return 56000
    elif baud == BAUD_57600:
      return 57600
    elif baud == BAUD_115200:
      return 115200
    elif baud == BAUD_256000:
      return 256000
    elif baud == BAUD_512000:
      return 512000
    elif baud == BAUD_921600:
      return 921600
    else:
      raise ValueError("Invalid baud rate error.")

  def get_moudle_baud(self):
    '''!
      @brief Get the Module baud
      @return baud
    '''
    time.sleep(0.1)
    rslt = self.read_reg(REG_UNO_BAUD, 1)
    return self.baud_match(rslt[0])
     
  def get_lora_baud(self):
    '''!
      @brief Get the Lora Baud rate
      @return baud
    '''
    time.sleep(0.1)
    rslt = self.read_reg(REG_LORA_BAUD, 1)
    return self.baud_match(rslt[0])
  
  def transmit_at(self, cmd):
    '''!
      @brief Interface for transparent transmission of gnss commands
      @return char * return commands
    '''
    pass
    
  def get_gnss_message(self, mode):
    '''!
      @brief Get different types of gps data
      @param mode
      @return char* 
    '''
    send = [0]
    writelen = 0
    if mode == GNGGA_MODE:
      reg_len, reg_all = REG_GGA_LEN, REG_GGA_ALL
    elif mode == GNRMC_MODE:
      reg_len, reg_all = REG_RMC_LEN, REG_RMC_ALL
    elif mode == GNGLL_MODE:
      reg_len, reg_all = REG_GLL_LEN, REG_GLL_ALL
    else:
      reg_len, reg_all = REG_VTG_LEN, REG_VTG_ALL

    rslt = self.read_reg(reg_len, 1)
    length = rslt[0]
    len1 = length // 32
    len2 = length % 32
    all_data = []

    if self.__uart_i2c == I2C_MODE:
      for num in range(len1 + 1):
        send[0] = writelen & 0x00FF
        self.write_reg(reg_len, send)
        read_len = len2 if num == len1 else 32
        rslt = self.read_reg(reg_all, read_len)
        all_data.extend(rslt)
        writelen += read_len
        time.sleep(0.001)
      time.sleep(0.01)
    else:
      all_data = self.read_reg(reg_all, length)
    return ''.join(map(chr, all_data))

    
  def get_gnss_len(self):
    '''!
      @brief Get length of GNSS data 
      @return length of GNSS data 
    '''
    self.__txbuf[0] = 0xAA
    time.sleep(0.001)
    self.write_reg(REG_ALL_MODE, self.__txbuf)
    time.sleep(0.001)
    self.write_reg(REG_ALL_MODE, self.__txbuf)
    time.sleep(0.001)
    count = 0
    while True:
      time.sleep(0.1)
      rslt = self.read_reg(REG_ALL_MODE, 1)
      if rslt[0] == 0x55:
        rslt = self.read_reg(REG_ALL_MODE, 1)
        break
      if rslt[0] == 0xFF:
        return 0
      count += 1
      if count > 20:
        return 0
    rslt = self.read_reg(REG_ALL_LEN_H, 2)
    return rslt[0]*256 + rslt[1]
    
  def set_user_name(self, name):
    '''!
      @brief Set user name
      @param name user name
    '''
    writelen = len(name)&0x00ff
    send = [writelen]
    self.write_reg(REG_USER_NAME_LEN, send)
    time.sleep(0.01)
    # Convert name to byte array and send it
    name_bytes = [ord(char) for char in name]
    self.write_reg(REG_USER_NAME, name_bytes)
    time.sleep(0.01)
    
  
  def set_user_password(self, password):
    '''!
      @brief Set user password
      @param password user password
    '''
    writelen = len(password)&0x00ff
    send = [writelen]
    self.write_reg(REG_USER_PASSWORD_LEN, send)
    time.sleep(0.01)
    # Convert name to byte array and send it
    name_bytes = [ord(char) for char in password]
    self.write_reg(REG_USER_PASSWORD, name_bytes)
    time.sleep(0.01)
    
  def set_server_addr(self, addr):
    '''!
      @brief Set server address
      @param addr server address
    '''
    writelen = len(addr)&0x00ff
    send = [writelen]
    self.write_reg(REG_SERVER_ADDR_LEN, send)
    time.sleep(0.01)
    # Convert name to byte array and send it
    name_bytes = [ord(char) for char in addr]
    self.write_reg(REG_SERVER_ADDR, name_bytes)
    time.sleep(0.01)
    
    
  def set_mount_point(self, point):
    '''!
      @brief Set mount point
      @param point mount point
    '''
    writelen = len(point)&0x00ff
    send = [writelen]
    self.write_reg(REG_MOUNT_POINT_LEN, send)
    time.sleep(0.01)
    # Convert name to byte array and send it
    name_bytes = [ord(char) for char in point]
    self.write_reg(REG_MOUNT_POINT, name_bytes)
    time.sleep(0.01)
    
  def set_port(self, port):
    '''!
      @brief Set port number
      @param port port number
    '''
    len_high = (port>>8)&0x00ff
    len_low  = (port)&0x00ff
    send = [len_high, len_low]
    self.write_reg(REG_PORT_H, send)
    time.sleep(0.01)
    
  def connect(self):
    '''!
      @brief Attempt to connect to the server
      @return CONNECT_SUCCESS if the connection is established,
              CONNECT_TIMEOUT if the connection times out,
              CONNECT_ERROR if there is an unknown error.
    '''
    send = [CONNECT_START]
    self.write_reg(REG_CONNECT, send)
    time.sleep(0.5)
    for i in range(10):
      _send_data = self.read_reg(REG_CONNECT_STATE, 1)
      if _send_data[0] == CONNECT_SUC:
        return CONNECT_SUCCESS
      time.sleep(2)
      if i >= 9:
        return CONNECT_TIMEOUT
    return CONNECT_ERROR
  
  def reconnect(self):
    '''
      @brief Reconnect to the server by resetting connection parameters
    '''
    _send_data = [CONNECT_START]
    self.write_reg(REG_CONNECT, _send_data)
    self._connect_count = 0
    if self.__uart_i2c == UART_MODE:
      time.sleep(1.5)

  def get_connect_state(self):
    '''!
      @brief Get connect state
      @return True if connected, else False
    '''
    _recv_data = self.read_reg(REG_CONNECT_STATE, 1)
    if _recv_data[0] != 0x55:
      self._connect_count += 1      # Record the number of connections.
    else:
      self._connect_count = 0       # Connect and clear the connection log.

    if self._connect_count > 15:
      return False  # Connection lost
    return True  # Connection is stable    
  
  def get_all_gnss(self):
    '''!
      @brief Get all GNSS data
      @return gnss all data
    '''
    send = [0, 0]
    all_data = []
    writelen = 0
    length = self.get_gnss_len()
    if length == 0:
      return all_data
    time.sleep(0.1)
    if self.__uart_i2c == UART_MODE:
      len1 = length // 250
      len2 = length % 250
      for num in range(len1 + 1):
        read_len = len2 if num == len1 else 250
        rslt = self.read_reg(REG_ALL, read_len)
        all_data.extend(rslt)
        writelen += 250
    else:
      len1 = length // 32
      len2 = length % 32
      for num in range(len1 + 1):
        send[0] = (writelen >> 8) & 0x00FF
        send[1] = writelen & 0x00FF
        self.write_reg(REG_ALL_LEN_H, send)
        time.sleep(0.001)
        read_len = len2 if num == len1 else 32
        rslt = self.read_reg(REG_ALL, read_len)
        all_data.extend(rslt)    
        writelen += 32
        time.sleep(0.001)
    return ''.join(map(chr, all_data))

class DFRobot_RTK_4G_I2C(DFRobot_RTK_4G): 
  def __init__(self, bus, addr):
    self.__addr = addr
    super(DFRobot_RTK_4G_I2C, self).__init__(bus,0)

  def write_reg(self, reg, data):
    self._connect = 0
    while True:
      try:
        self.i2cbus.write_i2c_block_data(self.__addr, reg, data)
        break
      except:
        self._error_handling()
      if self._connect > self.ERROR_COUNT:
        raise ValueError("Please check the rtk_4g connection or Reconnection sensor!!!")

  def read_reg(self, reg, len):
    self._connect = 0
    while True:
      try:
        rslt = self.i2cbus.read_i2c_block_data(self.__addr, reg, len)
        return rslt
      except:
        result = self._error_handling(len)
      if self._connect > self.ERROR_COUNT:
        raise ValueError("Please check the rtk_4g connection or Reconnection sensor!!!")

  def _error_handling(self, lens=0):
    result = [0] * lens
    self._connect += 1    
    #print("rtk_lora iic communication faild, please wait")
    time.sleep(0.5)
    return result

class DFRobot_RTK_4G_UART(DFRobot_RTK_4G):

  def __init__(self, Baud):
    self.__Baud = Baud
    super(DFRobot_RTK_4G_UART, self).__init__(0, Baud)

  def write_reg(self, reg, data):
    send = [0x55, reg | 0x80]    
    send.extend(data)
    send.append(0xAA)
    self.ser.write(send)
    time.sleep(0.005)
    return
    
  def read_reg(self, reg, length):
    send = [0x55, reg & 0x7f, length, 0xAA]
    self.ser.write(bytearray(send))
    recv = []
    timenow = time.time()
    timeout = 1
    timeout_time = timenow + timeout
    
    while len(recv) < length and time.time() < timeout_time:
      count = self.ser.inWaiting()  
      if count > 0:
        data = self.ser.read(min(count, length - len(recv)))
        recv.extend(data)        
        time.sleep(0.005)
    if len(recv) == length:
      if sys.version_info[0] < 3:
        return [ord(c) for c in recv]
      else:
        return list(recv)
    else:
      print("Timeout: Not enough data read.")
      return recv

