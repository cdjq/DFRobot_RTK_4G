 /*!
  * @file  configParam.ino
  * @brief config moudle param
  * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  * @license The MIT License (MIT)
  * @author ZhixinLiu(zhixin.liu@dfrobot.com)
  * @version V0.5.0
  * @date 2024-10-28
  * @url https://github.com/DFRobot/DFRobot_RTK_4G
  */

#include "DFRobot_RTK_4G.h"

// must use iic config parameter
DFRobot_RTK_4G_I2C rtk(&Wire ,DEVICE_ADDR);
void setup()
{
  Serial.begin(115200);
  while(!rtk.begin()){
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("Device connected !");

  /**
   * | Support Baud | UNO/ESP8266 | Leonardo/Mega2560 | ESP32 |  M0 |
   * | baud_9600    |      √      |         √         |   √   |  √  |
   * | baud_14400   |      √      |         √         |   √   |  √  |
   * | baud_19200   |      √      |         √         |   √   |  √  |
   * | baud_38400   |      √      |         √         |   √   |  √  |
   * | baud_56000   |      √      |         √         |   √   |  √  |
   * | baud_57600   |      √      |         √         |   √   |  √  |
   * | baud_115200  |             |         √         |   √   |  √  |
   * | baud_256000  |             |                   |   √   |  √  |
   * | baud_512000  |             |                   |   √   |  √  |
   * | baud_921600  |             |                   |   √   |  √  |
   */
  rtk.setModuleBaud(baud_115200);

  /**
   * The baud rate for receiving 4G information takes effect only when it is synchronized with the 4G module. It is not recommended for common users to change it
   * baud_9600
   * baud_19200
   * baud_38400
   * baud_57600
   * baud_115200  defult baud
   * baud_256000
   * baud_921600
  */
  //rtk.set4gBaud(baud_115200);
  Serial.print("module mode = ");
  Serial.println(rtk.getModule());

  Serial.print("moudle buad = ");
  Serial.println(rtk.getModuleBaud());
  Serial.print("4g buad = ");
  Serial.println(rtk.get4gBaud());
}

void loop()
{
  // Reserved interface, direct communication with gnss firmware, use with the original factory data manual
  // Serial.println(rtk.transmitAT("$PQTMVERNO*58\r\n"));
  delay(2000);
}