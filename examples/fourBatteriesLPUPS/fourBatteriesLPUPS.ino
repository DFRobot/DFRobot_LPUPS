/*!
 * @file  fourBatteriesLPUPS.ino
 * @brief LPUPS reports battery information to the computer via USB-HID
 * @details Reads battery information from UPS via I2C, and reports this information to the computer via USB-HID
 * @n HIDPowerDevice : https://github.com/abratchik/HIDPowerDevice
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.1
 * @date  2023-08-09
 * @url  https://github.com/DFRobot/DFRobot_LPUPS
 */
#include <DFRobot_LPUPS.h>
#include <HIDPowerDevice.h>
#include "upsDef.h"

DFRobot_LPUPS_I2C LPUPS(&Wire, /*I2CAddr*/ UPS_I2C_ADDRESS);

uint16_t iPreviousStatus = 0;   // Now and previous device status.
byte iRemaining = 0, iPrevRemaining = 100;
int iRes = 0;
uint16_t iPrevRunTimeToEmpty = 0;

int iIntTimer = 0; // Update interval counter


void setup(void)
{
  delay(5000);
  Serial.begin(115200);
  Serial.println("Serial Begin"); //Avoid serial port not working

  // Init the sensor
  while (NO_ERR != LPUPS.begin(FOUR_BATTERIES_UPS_PID)) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  Serial.println("Begin ok!");

  // Initialize UPS indicator LEDs
  pinMode(UPS_GREEN_LED, OUTPUT);   // Battery level indicator LED, green
  pinMode(UPS_RED_LED, OUTPUT);   // Battery level indicator LED, red
  pinMode(UPS_BLUE_LED, OUTPUT);   // Output refresh every 1 second, indicates Arduino cycle is running, blue

  /**
   * @fn setMaxChargeVoltage
   * @brief Set the maximum charging voltage
   * @param data Maximum charging voltage:
   * @n          Three batteries: 11100 ~ 12600 mV
   * @n          Four batteries: 14800 ~ 16800 mV
   * @return None
   */
   // maxChargeVoltage = 15600;
   // LPUPS.setMaxChargeVoltage(maxChargeVoltage);

   // Initialize HIDPowerDevice
  initPowerDevice();
}


void loop()
{
  /************ Get charge chip data and print ****************************/
  /**
   * Get chip data
   * regBuf - data buffer for storing data
   */
  LPUPS.getChipData(regBuf);
  printChargeData();

  /*********** Unit of measurement, measurement unit ****************************/
  /**
   * Battery voltage range: 12.3V ~ 16.8V, in order to keep the battery stable at extreme values:
   * Assuming the battery voltage range is 12.4V ~ 16.7V, corresponding to battery capacity 0 ~ 100.
   * Note: You can adjust the battery capacity more accurately by correcting the voltage mutation with dischargeCurrent if interested.
   */
  if (batteryVoltage > MIN_BATTERY_VOLTAGE) {
    iRemaining = (((float)batteryVoltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100;
  } else {
    Serial.println("The battery voltage is lower than normal !!!");   // Battery voltage lower than normal value.
  }

  if (100 < iRemaining) {
    iRemaining = 100;
  }

  // Please ensure to use the dedicated charger for LattePanda and connect it to the UPS (connect it to LP). 
  if (chargerStatus1.ac_stat) {   // check if there is charging current.
    bACPresent = true;
    if (64 < chargeCurrent) {   // Check if there is charging current. Due to precision issues, current less than 64 is considered as fully charged.
      bCharging = true;
    } else {
      bCharging = false;
    }
    bDischarging = false;
  } else {
    if (iPrevRemaining < iRemaining) {
      if (3 >= (iRemaining - iPrevRemaining))
        iRemaining = iPrevRemaining;
    }

    bACPresent = false;
    bCharging = false;
    if (dischargeCurrent) {   // Check if there is discharging current.
      bDischarging = true;
    } else {
      bDischarging = false;
    }
  }

  iRunTimeToEmpty = (float)iAvgTimeToEmpty * iRemaining / 100;

  // Adjust battery indicator LED based on the obtained battery capacity value iRemaining
  digitalWrite(UPS_GREEN_LED, LOW);   // Turn on green LED;
  digitalWrite(UPS_RED_LED, LOW);   // Turn on red LED;

  if (iRemaining <= 25) {   // Battery capacity <= 25%, turn on red LED
    digitalWrite(UPS_GREEN_LED, HIGH);   // Turn off green LED;
  } else if ((iRemaining > 25) && (iRemaining < 75)) {   // 25% < Battery capacity < 75%, both red and green LEDs are on

  } else if (iRemaining >= 75) {   // Battery capacity >= 75%, turn on green LED
    digitalWrite(UPS_RED_LED, HIGH);   // Turn off red LED;
  }

  // Refresh the values to be reported on USB-HID based on the obtained charge chip data
  flashReportedData();

  /************ Delay ***************************************/
  delay(1000);
  iIntTimer++;
  digitalWrite(UPS_BLUE_LED, LOW);   // 打开 蓝色 LED灯;
  delay(1000);
  iIntTimer++;
  digitalWrite(UPS_BLUE_LED, HIGH);   // 关掉 蓝色 LED灯;

  /************ 批量发送或中断 ***********************/
  if ((iPresentStatus != iPreviousStatus) || (iRemaining != iPrevRemaining) ||
    (iRunTimeToEmpty != iPrevRunTimeToEmpty) || (iIntTimer > MIN_UPDATE_INTERVAL)) {

    // 12 INPUT OR FEATURE(required by Windows)
    PowerDevice.sendReport(HID_PD_REMAININGCAPACITY, &iRemaining, sizeof(iRemaining));
    if (bDischarging) PowerDevice.sendReport(HID_PD_RUNTIMETOEMPTY, &iRunTimeToEmpty, sizeof(iRunTimeToEmpty));
    iRes = PowerDevice.sendReport(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));

    if (iRes < 0) {   // Reporting return value: less than 0 indicates communication loss with the host
      pinMode(UPS_BLUE_LED, INPUT);
    } else {
      pinMode(UPS_BLUE_LED, OUTPUT);
    }

    iIntTimer = 0; // Reset reporting interval timer
    iPreviousStatus = iPresentStatus; // Save new device status
    iPrevRemaining = iRemaining; // Save new battery remaining capacity
    iPrevRunTimeToEmpty = iRunTimeToEmpty; // Save new estimated battery runtime count

  }

  /************ Serial print reported battery level and operation result ******************/
  Serial.print("iRemaining = "); // Battery remaining capacity percentage
  Serial.println(iRemaining);
  Serial.print("iRunTimeToEmpty = "); // Estimated time to empty battery
  Serial.println(iRunTimeToEmpty);
  Serial.print("iRes = "); // Reporting return value, less than 0: indicates communication loss with host
  Serial.println(iRes);
  Serial.println();
}

