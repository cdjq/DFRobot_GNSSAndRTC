/*!
 * @file  getAllGNSS.ino
 * @brief read all gnss data
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @version V1.0
 * @date 2022-08-30
 * @url https://github.com/DFRobot/DFRobot_GNSSAndRTC
 */

#include "DFRobot_GNSSAndRTC.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
DFRobot_GNSSAndRTC_I2C gnss(&Wire, MODULE_I2C_ADDRESS);
#else
 /* ---------------------------------------------------------------------------------------------------------------------
  *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
  *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
  *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
  *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
  *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
  * ----------------------------------------------------------------------------------------------------------------------*/
  /* Baud rate cannot be changed  */
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
SoftwareSerial mySerial(4, 5);
DFRobot_GNSSAndRTC_UART gnss(&mySerial, UART_BAUDRATE);
#elif defined(ESP32)
DFRobot_GNSSAndRTC_UART gnss(&Serial1, UART_BAUDRATE,/*rx*/D2,/*tx*/D3);
#else
DFRobot_GNSSAndRTC_UART gnss(&Serial1, UART_BAUDRATE);
#endif
#endif

void callback(char* data, uint8_t len)
{
  for (uint8_t i = 0; i < len; i++) {
    Serial.print((char)data[i]);
  }
  delay(1);
}

void setup()
{
  Serial.begin(115200);
  while (!gnss.begin()) {
    Serial.println("NO Deivces !");
    delay(1000);
  }
  gnss.enablePower();      // Enable gnss power

  /** Set GNSS to be used
   *   eGPS              use gps
   *   eBeiDou           use beidou
   *   eGPS_BeiDou       use gps + beidou
   *   eGLONASS          use glonass
   *   eGPS_GLONASS      use gps + glonass
   *   eBeiDou_GLONASS   use beidou +glonass
   *   eGPS_BeiDou_GLONASS use gps + beidou + glonass
   */
  gnss.setGnss(gnss.eGPS_BeiDou_GLONASS);

  gnss.setCallback(callback);
}

void loop()
{
  Serial.println("\n---------------------------Raw data from L76K-------------------------------");
  gnss.getAllGnss();
  delay(3000);
}
