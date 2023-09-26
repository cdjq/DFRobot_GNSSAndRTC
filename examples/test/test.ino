// 自测专用，测完即删

/*!
 * @file test.ino
 * @brief 运行这个例程控制引脚32K输出方波
 * @n 运行这个例程来实现倒计时功能
 * @n 读取所有GNSS数据
 * @n 获取gnss简单数据
 * @n 运行此例程，先设置内部时钟，然后循环获取时钟、温度和电压数据
 * @n 运行此例程，先设置内部时钟和中断触发，到达设定时间后触发中断
 * @n 运行这个例程来读写RTC模块中的RAM数据
 * @copyright    Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license      The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @version V1.0
 * @date 2022-08-30
 * @url https://github.com/DFRobot/DFRobot_GNSSAndRTC
 */
#include "DFRobot_GNSSAndRTC.h"

#define I2C_COMMUNICATION  //use I2C for communication, but use the serial port for communication if the line of codes were masked

#ifdef  I2C_COMMUNICATION
DFRobot_GNSSAndRTC_I2C rtc(&Wire, MODULE_I2C_ADDRESS);
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
DFRobot_GNSSAndRTC_UART rtc(&mySerial, UART_BAUDRATE);
#elif defined(ESP32)
DFRobot_GNSSAndRTC_UART rtc(&Serial1, UART_BAUDRATE,/*rx*/D2,/*tx*/D3);
#else
DFRobot_GNSSAndRTC_UART rtc(&Serial1, UART_BAUDRATE);
#endif
#endif

/***********************************************************************************************
                    config
************************************************************************************************/

volatile  int8_t alarmFlag = 0;
void interrupt(void)
{
    alarmFlag = 1;
}

void callback(char* data, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        Serial.print((char)data[i]);
    }
    delay(1);
}

void configInterrupt(void)
{
#if defined(ESP32)||defined(ARDUINO_SAM_ZERO)
    attachInterrupt(digitalPinToInterrupt(D7)/*Query the interrupt number of the D6 pin*/, interrupt, FALLING);
#elif defined(ESP8266)
    attachInterrupt(digitalPinToInterrupt(D5)/*Query the interrupt number of the D6 pin*/, interrupt, FALLING);
#else
    /*    The Correspondence Table of AVR Series Arduino Interrupt Pins And Terminal Numbers
    * ---------------------------------------------------------------------------------------
    * |                                        |  DigitalPin  | 2  | 3  |                   |
    * |    Uno, Nano, Mini, other 328-based    |--------------------------------------------|
    * |                                        | Interrupt No | 0  | 1  |                   |
    * |-------------------------------------------------------------------------------------|
    * |                                        |    Pin       | 2  | 3  | 21 | 20 | 19 | 18 |
    * |               Mega2560                 |--------------------------------------------|
    * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  | 5  |
    * |-------------------------------------------------------------------------------------|
    * |                                        |    Pin       | 3  | 2  | 0  | 1  | 7  |    |
    * |    Leonardo, other 32u4-based          |--------------------------------------------|
    * |                                        | Interrupt No | 0  | 1  | 2  | 3  | 4  |    |
    * |--------------------------------------------------------------------------------------
    */
    /*                      The Correspondence Table of micro:bit Interrupt Pins And Terminal Numbers
    * ---------------------------------------------------------------------------------------------------------------------------------------------
    * |             micro:bit                       | DigitalPin |P0-P20 can be used as an external interrupt                                     |
    * |  (When using as an external interrupt,      |---------------------------------------------------------------------------------------------|
    * |no need to set it to input mode with pinMode)|Interrupt No|Interrupt number is a pin digital value, such as P0 interrupt number 0, P1 is 1 |
    * |-------------------------------------------------------------------------------------------------------------------------------------------|
    */
    attachInterrupt(digitalPinToInterrupt(2), interrupt, FALLING);
#endif
}

/***********************************************************************************************
                    setup and loop
************************************************************************************************/

void setup()
{
    Serial.begin(115200);
    /*Wait for the chip to be initialized completely, and then exit*/
    while (!rtc.begin()) {
        Serial.println("Failed to init chip, please check if the chip connection is fine. ");
        delay(1000);
    }

    configInterrupt();

    /** Set GNSS to be used
     *   eGPS              use gps
     *   eBeiDou           use beidou
     *   eGPS_BeiDou       use gps + beidou
     *   eGLONASS          use glonass
     *   eGPS_GLONASS      use gps + glonass
     *   eBeiDou_GLONASS   use beidou +glonass
     *   eGPS_BeiDou_GLONASS use gps + beidou + glonass
     */
    rtc.setGnss(rtc.eGPS_BeiDou_GLONASS);

    rtc.setCallback(callback);   // 读取并打印所有GNSS数据
}

void loop()
{
    parseSerialCommand();

    getCalibStatus();

    getInterruptStatus();

    getRTCData();

    delay(900);
}

/***********************************************************************************************
                    The parsing and implementation of the serial command
************************************************************************************************/

void parseSerialCommand(void)
{
    /**
     * Command format: cmd-value
     * cmd: command type
     * value: the set value corresponding to the command type, some commands can be empty
     */
    if (Serial.available()) {   // Detect whether there is an available serial command
        String cmd = Serial.readStringUntil('-');   // Read the specified termination character string, used to cut and identify the serial command. The same type won't repeat later.
        int value = Serial.parseInt();   // Parse character string and return int number

        if (cmd.equals("gnns")) {
            switch (value) {
            case 1:
                /*********************************************************************************
                 *                 读取所有GNSS数据
                **********************************************************************************/
                Serial.println("****************************************** getAllGNSS.ino");
                rtc.getAllGnss();
                break;
            case 2:
                /*********************************************************************************
                   *                 获取gnss简单数据
                  **********************************************************************************/
                Serial.println("****************************************** getGNSS.ino");
                getGNSS();
                break;
            case 3:
                rtc.enablePower();      // Enable gnss power
                break;
            case 4:
                rtc.disablePower();      // Disable GNSS, the data will not be refreshed after disabling  
                break;
            case 5:
                /**
                 * @brief Calibrate RTC immediately with GNSS
                 * @note This is a single calibration;
                 * @n If the GNSS module signal is weak, time calibration may encounter issues.
                 * @return None
                 */
                rtc.calibRTC();
                break;
            case 6:
                /**
                 * @brief The loop automatically performs GNSS timing based on the set interval
                 * @param hour Automatic calibration of the time interval. range: 0~255, unit: hour.
                 * @note When set to zero, automatic time calibration is disabled.
                 * @n Enabling it will trigger an immediate calibration.
                 * @n If the GNSS module signal is weak, time calibration may encounter issues.
                 * @return None
                 */
                rtc.calibRTC(1);
                break;

            default:
                break;
            }
            delay(3000);

        } else if (cmd.equals("rtc")) {
            switch (value) {
            case 1:
                /*********************************************************************************
                   *                 运行此例程，先设置内部时钟，然后循环获取时钟、温度和电压数据
                  **********************************************************************************/
                Serial.println("\n****************************************** getTime.ino");
                //Get internal temperature
                Serial.print(rtc.getTemperatureC());
                Serial.println(" C");
                //Get battery voltage
                Serial.print(rtc.getVoltage());
                Serial.println(" V");
                /*Enable 12-hour time format*/
                Serial.println(rtc.getAMorPM());
                Serial.println();
                // getRTCData();

                break;
            case 2:
                /*********************************************************************************
                 *                 运行这个例程来读写RTC模块中的RAM数据
                **********************************************************************************/
                Serial.println("\n****************************************** ramReadAndWrite.ino");
                testRAM();
                break;
            case 3:
                /*********************************************************************************
                   *                 运行这个例程控制引脚32K输出方波
                  **********************************************************************************/
                Serial.println("****************************************** control32k.ino");
                rtc.enable32k();
                break;
            case 4:
                rtc.disable32k();
                break;
            case 5:
                rtc.setTime(2022, 7, 27, 23, 59, 40);//Set default time
                break;
            case 6:
                rtc.setHourSystem(rtc.e12hours);//Set display format
                break;
            case 7:
                rtc.setHourSystem(rtc.e24hours);//Set display format
                break;
            case 8:
                //Countdown timer, schedule alarm and daily regular alarm can't be used at the same time
                Serial.println("Count down start");
                rtc.countDown(10);
                break;
            case 9:
                Serial.println("set Alarm 1 start");
                rtc.setAlarm(2022, 7, 28);//Set schedule alarm
                break;
            case 10:
                Serial.println("set Alarm 2 start");
                rtc.setAlarm(rtc.eEveryDay, 0, 0, 20);//Set daily regular alarm
                break;
            case 11:
                Serial.println("clear Alarm");
                rtc.clearAlarm();
                break;

            default:
                break;
            }

            // } else if (cmd.equals("set")) {
            delay(3000);

        } else {   // Unknown command type
            Serial.println("Help : \n \
      Currently available commands (format: cmd-value):\n \
        Set and open high-pass filter: e.g. hp-500\n \
        Get the address of the remote Bluetooth device: e.g. addr-\n \
      For more details about commands, please refer to the code comments of this demo.\n");
            delay(1000);
        }
        while (Serial.read() >= 0);   // Clear the remaining data in the serial port
    }
}

void testRAM(void)
{
    rtc.writeSRAM(0x2D, 0x11);//Address Range 0x2c~0x71
    delay(1000);
    uint8_t data = rtc.readSRAM(0x2D);
    Serial.print("write data:");
    Serial.println(data);
    delay(100);
    rtc.clearSRAM(0x2D);
    delay(100);
    data = rtc.readSRAM(0x2D);
    Serial.print("clear data:");
    Serial.println(data);
}

void getRTCData(void)
{
    DFRobot_GNSSAndRTC::sTimeData_t sTime;
    sTime = rtc.getRTCTime();
    Serial.print(sTime.year, DEC);//year
    Serial.print('/');
    Serial.print(sTime.month, DEC);//month
    Serial.print('/');
    Serial.print(sTime.day, DEC);//day
    Serial.print(" (");
    Serial.print(sTime.week);//week
    Serial.print(") ");
    Serial.print(sTime.hour, DEC);//hour
    Serial.print(':');
    Serial.print(sTime.minute, DEC);//minute
    Serial.print(':');
    Serial.print(sTime.second, DEC);//second
    Serial.println(' ');
}

void getGNSS(void)
{
    DFRobot_GNSSAndRTC::sTim_t utc = rtc.getUTC();
    DFRobot_GNSSAndRTC::sTim_t date = rtc.getDate();
    DFRobot_GNSSAndRTC::sLonLat_t lat = rtc.getLat();
    DFRobot_GNSSAndRTC::sLonLat_t lon = rtc.getLon();
    double high = rtc.getAlt();
    uint8_t starUserd = rtc.getNumSatUsed();
    double sog = rtc.getSog();
    double cog = rtc.getCog();

    Serial.println("");
    Serial.print(date.year);
    Serial.print("/");
    Serial.print(date.month);
    Serial.print("/");
    Serial.print(date.date);
    Serial.print("/");
    Serial.print(utc.hour);
    Serial.print(":");
    Serial.print(utc.minute);
    Serial.print(":");
    Serial.print(utc.second);
    Serial.println();
    Serial.println((char)lat.latDirection);
    Serial.println((char)lon.lonDirection);

    Serial.print("lat DDMM.MMMMM = ");
    Serial.println(lat.latitude, 5);
    Serial.print("lon DDDMM.MMMMM = ");
    Serial.println(lon.lonitude, 5);

    Serial.print("lat degree = ");
    Serial.println(lat.latitudeDegree, 6);
    Serial.print("lon degree = ");
    Serial.println(lon.lonitudeDegree, 6);

    Serial.print("star userd = ");
    Serial.println(starUserd);
    Serial.print("alt high = ");
    Serial.println(high);
    Serial.print("sog =  ");
    Serial.println(sog);
    Serial.print("cog = ");
    Serial.println(cog);
    Serial.print("gnss mode =  ");
    Serial.println(rtc.getGnssMode());
}

uint8_t underCalibCount = 0;
void getCalibStatus(void)
{
    /**
     * @brief Current clock calibration status
     * @param mode By default, it is set to true, indicating access to the calibration status only.
     * @n  If continuous calibration for one minute does not return a successful calibration,
     * @n  you can pass in false to manually terminate this calibration session.
     * @return uint8_t type, indicates current clock calibration status
     * @retval 0 Not calibrated
     * @retval 1 Calibration complete
     * @retval 2 Under calibration
     * @note Note: To avoid affecting subsequent calibration status,
     * @n    "Calibration completed Status (1)" is automatically zeroed after a successful read
     */
    uint8_t status = rtc.calibStatus();
    if (DFRobot_GNSSAndRTC::eCalibComplete == status) {
        underCalibCount = 0;
        Serial.println("Calibration success!");
    } else if (DFRobot_GNSSAndRTC::eUnderCalib == status) {
        underCalibCount += 1;
        if (60 <= underCalibCount) {   // If the calibration fails for a long time, manually terminate the calibration
            rtc.calibStatus(false);
            underCalibCount = 0;
            Serial.println("Calibration failed!");
            Serial.println("It may be due to weak satellite signals.");
            Serial.println("Please proceed to an open outdoor area for time synchronization.");
        }
    }
}

void getInterruptStatus(void)
{
    /*********************************************************************************
     *                 运行这个例程来实现倒计时功能
    **********************************************************************************/
    // Serial.println("****************************************** countDown.ino");
    if (alarmFlag == 1) {
        // rtc.countDown(10);
        // rtc.clearAlarm();
        alarmFlag = 0;
        Serial.println("Count down alarm clock is triggered.");
    }
}
