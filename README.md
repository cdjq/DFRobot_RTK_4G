# DFRobot_RTK_4G
- [中文版](./README_CN.md)
This RTK high-precision positioning kit uses the combination of base station and mobile station to achieve centimeter-level positioning within a certain range of open areas. The calibration data obtained by the base station is transparently transmitted to the mobile station module in RTCM format through the Lora module, and the mobile station module realizes centimeter-level positioning output through calibration operations. The kit consists of two Lora modules, a mobile station module, a base station module, and two GNSS antennas supporting the L1+L5 band.
The mobile station adopts the remote LC29HDA module, which is a dual-band, multi-constellation GNSS module, supporting the simultaneous reception of GNSS, GPS, GLONASS, Galileo, BDS and QZSS satellites. Also tracking GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5 bands, Can quickly obtain longitude and latitude, time, altitude and other data. The number of visible satellites is greatly increased compared with ordinary GPS, and combined with the base station module, after receiving differential data, centimeter-level positioning can be achieved in open areas.
The base station adopts the remote LC29HBS module, which is a dual-band, multi-constellation GNSS module, supporting the simultaneous reception of GNSSGPS, GLONASS, Galileo, BDS and QZSS satellites. Simultaneous tracking of GPS L1 C/A, GLONASS L1, Galileo E1, BDS B1I, QZSS L1 C/A, GPS L5, GalileoE5a, BDS B2a and QZSS L5 bands provides fast and accurate acquisition, Provide correction data.
The mobile end of the kit supports I2C/UART output, and is compatible with Arduino, ESP32, Raspberry PI and other main control devices. As an entry-level module, it is simple to use, without complex wiring and data link construction, and can be used for RTK entry learning, positioning tracking car, item tracking, high-precision control, etc.


![效果图](resources/images/xxx.jpg)

## Product Link(https://www.dfrobot.com)

    SKU：TEL0171-4G

## Table of Contents

* [Summary](#Summary)
* [Installation](#Installation)
* [Methods](#Methods)
* [Compatibility](#Compatibility)
* [History](#History)
* [Credits](#Credits)

## Summary

Provide an Arduino library for the RTK module with the following features:
Retrieval of GNSS data
Retrieval of raw GNSS data.
Config module parameter

## Installation
There are two methods for using this library:<br>
1. Open Arduino IDE, search for "DFRobot_RTK_4G" on the status bar in Tools ->Manager Libraries and install the library.<br>
2. Download the library file before use, paste it into \Arduino\libraries directory, then open the examples folder and run the demo in the folder.<br>

## Methods

```C++
/**
 * @fn getUTC
 * @brief Get UTC, standard time 
 * @return sTim_t type, represents the returned hour, minute and second 
 * @retval sTim_t.hour hour 
 * @retval sTim_t.minute minute 
 * @retval sTim_t.second second 
 */
  sTim_t getUTC(void);

/**
 * @fn getDate
 * @brief Get date information, year, month, day 
 * @return sTim_t type, represents the returned year, month, day 
 * @retval sTim_t.year year
 * @retval sTim_t.month month 
 * @retval sTim_t.day day 
 */
  sTim_t getDate(void);

/**
 * @fn getLat
 * @brief Get latitude 
 * @return sLonLat_t type, represents the returned latitude  
 * @retval sLonLat_t.latDD   Latitude degree(0-90)
 * @retval sLonLat_t.latMM   The first and second digits behind the decimal point 
 * @retval sLonLat_t.latMMMMM Latitude  The third and seventh digits behind the decimal point 
 * @retval sLonLat_t.latitude Latitude value with 7 decimal digits
 * @retval sLonLat_t.latDirection Direction of latitude
 */
  sLonLat_t getLat(void);

/**
 * @fn getLon
 * @brief Get longitude 
 * @return sLonLat_t Type, represents the returned longitude
 * @retval sLonLat_t.lonDDD  Longitude degree(0-90)
 * @retval sLonLat_t.lonMM   Longitude  The first and second digits behind the decimal point
 * @retval sLonLat_t.lonMMMMM Longitude The third and seventh digits behind the decimal point
 * @retval sLonLat_t.lonitude Longitude value with 7 decimal digits
 * @retval sLonLat_t.lonDirection Direction of longitude 
 */
  sLonLat_t getLon(void);

/**
 * @fn getNumSatUsed
 * @brief Get the number of the used satellite used
 * @return uint8_t type, represents the number of the used satellite
 */
  uint8_t getNumSatUsed(void);

/**
 * @fn getAlt
 * @brief Altitude information
 * @return double type, represents altitude 
 */
  double getAlt(void);

/**
 * @fn getSep
 * @brief At the height of geoid
 * @return double 
 */
  double getSep(void);

/**
 * @fn getHdop
 * @brief Indicates the horizontal accuracy of positioning
 * @return double
 */
  double getHdop(void);

/**
 * @fn getQuality
 * @brief message Quality
 * @return uint8_t 
 */
  uint8_t getQuality(void);

/**
 * @fn getSiteID
 * @brief The site id of differential gps data, commonly used for differential gps positioning
 * @return uint16_t
 */
  uint16_t getSiteID(void);

/**
 * @fn getDifTime
 * @brief The number of seconds in which a differential signal was last received
 * @return double 
 */
  double getDifTime(void);

/**
 * @fn getDataFlush
 * @brief Gets whether data is refreshed
 * @return bool
 * @retval True if data is refreshed
 * @retval false if data is not refreshed
 */
  bool getDataFlush(void);

/**
 * @fn setModule
 * @brief Set the Module
 * @param mode 4G or lora
 */
  void setModule(eModuleMode_t mode);

/**
 * @fn getModule
 * @brief Get the Module run mode
 * @return eModuleMode_t 
 */
  eModuleMode_t getModule(void);

  /**
   * @fn transmitAT
   * @brief Interface for transparent transmission of gnss commands
   * @return char * return commands
   */
  char * transmitAT(const char* cmd);

/**
 * @fn getGnssMessage
 * @brief Get different types of gps data
 * @param mode eGnssData_t type
 * @return char* 
 */
  char * getGnssMessage(eGnssData_t mode);

/**
 * @fn getAllGnss
 * @brief Get GNSS data, call back and receive
 * @return null
 */
  void getAllGnss(void);

/**
 * @fn setModuleBaud
 * @brief Set the Module Baud rate
 * @param baud eModuleBaud_t
 */
  void setModuleBaud(eModuleBaud_t baud);

/**
 * @fn set4gBaud
 * @brief Set the receive 4g Baud rate
 * @param baud eModuleBaud_t
 */
  void set4gBaud(eModuleBaud_t baud);

/**
 * @fn setLoraBaud
 * @brief Set the recevie Lora Baud rate
 * @param baud eModuleBaud_t
 */
  void setLoraBaud(eModuleBaud_t baud);

/**
 * @fn getModuleBaud
 * @brief Get the Module Baud rate
 * @return uint32_t Baud rate of serial communication
 */
  uint32_t getModuleBaud(void);

/**
 * @fn getLoraBaud
 * @brief Get the Lora Baud rate
 * @return uint32_t Baud rate of serial communication
 */
  uint32_t getLoraBaud(void);

/**
 * @fn get4gBaud
 * @brief Get the 4G Baud rate
 * @return uint32_t Baud rate of serial communication
 */
  uint32_t get4gBaud(void);

/**
 * @fn setUserName
 * @brief Set the 4G User Name
 * @param name user name eg:"chw123456789"
 * @param len name length
 */
  void setUserName(const char *name, uint8_t len);

/**
 * @fn setUserPassword
 * @brief Set the 4G User Password
 * @param password eg:"12345678"
 * @param len password length
 */
  void setUserPassword(const char *password, uint8_t len);

/** 
 * @fn setServerAddr
 * @brief Set the Server Addr
 * @param addr ip address eg: "192.168.1.1"
 * @param len addr length
 */
  void setServerAddr(const char *addr, uint8_t len);

/**
 * @fn setServerAddr
 * @brief set Mount Point
 * @param point eg："RTCM33"
 * @param len point length
 */
  void setMountPoint(const char *point, uint8_t len);

/**
 * @fn setPort
 * @brief Set the Port
 * @param port eg: 8002
 */
  void setPort(uint16_t port);

/**
 * @fn connect
 * @brief connect 
 * @return String connect state
 */
  String connect(void);

/**
 * @fn getConnectState
 * @brief get connect state
 * @return true or false
 */
  bool getConnectState(void);

/**
 * @fn reConnect
 * @brief re connect
 */
  void reConnect(void);

/**
 * @fn setCallback
 * @brief Set callback function type
 * @param  call function name 
 * @return null
 */
  void setCallback(void (*call)(char *, uint8_t));
```

## Compatibility

MCU                | Work Well    |   Work Wrong    | Untested    | Remarks
------------------ | :----------: | :-------------: | :---------: | :----:
Arduino Uno        |      √       |                 |             |
Arduino MEGA2560   |      √       |                 |             |
Arduino Leonardo   |      √       |                 |             |
FireBeetle-ESP8266 |      √       |                 |             |
FireBeetle-ESP32   |      √       |                 |             |
FireBeetle-M0      |      √       |                 |             |
Micro:bit          |      √       | nonsupport uart |             |


## History

- 2024/10/28 - Version V0.1.0 released.

## Credits

Written by ZhixinLiu(zhixin.liu@dfrobot.com), 2024. (Welcome to our website)
