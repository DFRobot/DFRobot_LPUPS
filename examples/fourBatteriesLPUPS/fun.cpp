
#include <DFRobot_LPUPS.h>
#include <HIDPowerDevice.h>
#include "upsDef.h"

uint8_t regBuf[DATA_LEN_MAX] = { 0 };
DFRobot_LPUPS_I2C::sChargerStatus0_t chargerStatus0;
DFRobot_LPUPS_I2C::sChargerStatus1_t chargerStatus1;
DFRobot_LPUPS_I2C::sProchotStatus0_t prochotStatus0;
DFRobot_LPUPS_I2C::sProchotStatus1_t prochotStatus1;
uint16_t systemPower = 0, inputVoltage = 0;
uint16_t dischargeCurrent = 0, chargeCurrent = 0;
uint16_t CMPINVoltage = 0, inputCurrent = 0;
uint16_t batteryVoltage = 0, systemVoltage = 0, maxChargeVoltage = 0;
bool bCharging, bACPresent, bDischarging; // Whether charging, AC power present, discharging

char outputBuf[512]; // Print buffer

// String constants
const char STRING_DEVICE_CHEMISTRY[] PROGMEM = "Li-ion";   // Li-ion
const char STRING_OEM_VENDOR[] PROGMEM = "MyCoolUPS";
const char STRING_SERIAL[] PROGMEM = "UPS100";   // UPS100

const byte bDeviceChemistry = IDEVICECHEMISTRY;   // Index of a string descriptor containing the battery’s chemistry.
const byte bOEMVendor = IOEMVENDOR;

uint16_t iPresentStatus = 0;   // Now and previous device status.

byte bRechargable = 1;   // Rechargeable Battery (1)/Not Rechargeable Battery (0)
byte bCapacityMode = 2;   // In the data manual, "2" represents battery capacity in percentage.

// Physical parameters.
const uint16_t iConfigVoltage = MAX_BATTERY_VOLTAGE;   // Nominal value of the voltage.
uint16_t iVoltage = MAX_BATTERY_VOLTAGE;
uint16_t iRunTimeToEmpty = 0;
uint16_t iAvgTimeToFull = 7200;
uint16_t iAvgTimeToEmpty = 7200;   // 12
uint16_t iRemainTimeLimit = 600;   // 1
/* Writing this value immediately shuts down (i.e., turns off) the output
   for a period equal to the indicated number of seconds in
   DelayBeforeReboot, after which time the output is started. */
int16_t  iDelayBe4Reboot = -1;
/* Writing this value shuts down (i.e., turns off) either the output after
  the indicated number of seconds, or sooner if the batteries become depleted. */
int16_t  iDelayBe4ShutDown = -1;

byte iAudibleAlarmCtrl = 2; // 1 - Disabled, 2 - Enabled, 3 - Muted

// Parameters compliant with Advanced Configuration and Power Interface (ACPI).
const byte iDesignCapacity = 100;
byte iWarnCapacityLimit = 10; // warning at 10% 
byte iRemnCapacityLimit = 5; // low at 5% 
const byte bCapacityGranularity1 = 1; // Battery capacity granularity between low and warning.
const byte bCapacityGranularity2 = 1; // Battery capacity granularity between warning and full.
byte iFullChargeCapacity = 100;


void initPowerDevice(void)
{
  PowerDevice.begin();

  // 序列号是以特殊方式设置的，因为它形成了Arduino端口名称
  PowerDevice.setSerial(STRING_SERIAL);

  // 用于调试目的。
  PowerDevice.setOutput(Serial);

  // usb上报参数设置
  PowerDevice.setFeature(HID_PD_PRESENTSTATUS, &iPresentStatus, sizeof(iPresentStatus));

  PowerDevice.setFeature(HID_PD_RUNTIMETOEMPTY, &iRunTimeToEmpty, sizeof(iRunTimeToEmpty));
  PowerDevice.setFeature(HID_PD_AVERAGETIME2FULL, &iAvgTimeToFull, sizeof(iAvgTimeToFull));
  PowerDevice.setFeature(HID_PD_AVERAGETIME2EMPTY, &iAvgTimeToEmpty, sizeof(iAvgTimeToEmpty));
  PowerDevice.setFeature(HID_PD_REMAINTIMELIMIT, &iRemainTimeLimit, sizeof(iRemainTimeLimit));
  PowerDevice.setFeature(HID_PD_DELAYBE4REBOOT, &iDelayBe4Reboot, sizeof(iDelayBe4Reboot));
  PowerDevice.setFeature(HID_PD_DELAYBE4SHUTDOWN, &iDelayBe4ShutDown, sizeof(iDelayBe4ShutDown));

  PowerDevice.setFeature(HID_PD_RECHARGEABLE, &bRechargable, sizeof(bRechargable));
  PowerDevice.setFeature(HID_PD_CAPACITYMODE, &bCapacityMode, sizeof(bCapacityMode));
  PowerDevice.setFeature(HID_PD_CONFIGVOLTAGE, &iConfigVoltage, sizeof(iConfigVoltage));
  PowerDevice.setFeature(HID_PD_VOLTAGE, &iVoltage, sizeof(iVoltage));

  PowerDevice.setStringFeature(HID_PD_IDEVICECHEMISTRY, &bDeviceChemistry, STRING_DEVICE_CHEMISTRY);
  PowerDevice.setStringFeature(HID_PD_IOEMINFORMATION, &bOEMVendor, STRING_OEM_VENDOR);

  PowerDevice.setFeature(HID_PD_AUDIBLEALARMCTRL, &iAudibleAlarmCtrl, sizeof(iAudibleAlarmCtrl));

  PowerDevice.setFeature(HID_PD_DESIGNCAPACITY, &iDesignCapacity, sizeof(iDesignCapacity));
  PowerDevice.setFeature(HID_PD_FULLCHRGECAPACITY, &iFullChargeCapacity, sizeof(iFullChargeCapacity));
  PowerDevice.setFeature(HID_PD_REMAININGCAPACITY, &iRemaining, sizeof(iRemaining));
  PowerDevice.setFeature(HID_PD_WARNCAPACITYLIMIT, &iWarnCapacityLimit, sizeof(iWarnCapacityLimit));
  PowerDevice.setFeature(HID_PD_REMNCAPACITYLIMIT, &iRemnCapacityLimit, sizeof(iRemnCapacityLimit));
  PowerDevice.setFeature(HID_PD_CPCTYGRANULARITY1, &bCapacityGranularity1, sizeof(bCapacityGranularity1));
  PowerDevice.setFeature(HID_PD_CPCTYGRANULARITY2, &bCapacityGranularity2, sizeof(bCapacityGranularity2));
}

void printChargeData(void)
{
  /*************** CS32_I2C_CHARGER_STATUS_REG ~ CS32_I2C_PROCHOT_STATUS_REG ***************/
  memcpy(&chargerStatus0, &regBuf[CS32_I2C_CHARGER_STATUS_REG], sizeof(regBuf[CS32_I2C_CHARGER_STATUS_REG]));
  memcpy(&chargerStatus1, &regBuf[CS32_I2C_CHARGER_STATUS_REG + 1], sizeof(regBuf[CS32_I2C_CHARGER_STATUS_REG + 1]));
  memcpy(&prochotStatus0, &regBuf[CS32_I2C_PROCHOT_STATUS_REG], sizeof(regBuf[CS32_I2C_PROCHOT_STATUS_REG]));
  memcpy(&prochotStatus1, &regBuf[CS32_I2C_PROCHOT_STATUS_REG + 1], sizeof(regBuf[CS32_I2C_PROCHOT_STATUS_REG + 1]));
  memset(outputBuf, 0, sizeof(outputBuf));
  sprintf(outputBuf, "Charge status register 0 = %#x\r\n"
    "Charge status register 1 = %#x\r\n"
    "Prochot status register 0 = %#x\r\n"
    "Prochot status register 1 = %#x\r\n",
    regBuf[CS32_I2C_CHARGER_STATUS_REG], regBuf[CS32_I2C_CHARGER_STATUS_REG + 1], regBuf[CS32_I2C_PROCHOT_STATUS_REG], regBuf[CS32_I2C_PROCHOT_STATUS_REG + 1]);
  Serial.print(outputBuf);

  /*************** CS32_I2C_ADC_PSYS_REG ~ CS32_I2C_ADC_VSYS_REG ***************/
  // PSYS: Full range: 3.06 V, LSB: 12 mV
  systemPower = regBuf[CS32_I2C_ADC_PSYS_REG] * 12;
  // VBUS: Full range: 3.2 V - 19.52 V, LSB: 64 mV
  inputVoltage = 3200 + regBuf[CS32_I2C_ADC_VBUS_REG] * 64;
  if (3200 == inputVoltage) {
    inputVoltage = 0;
  }
  // IDCHG: Full range: 32.512 A, LSB: 256 mA
  dischargeCurrent = regBuf[CS32_I2C_ADC_IDCHG_REG] * 256;
  // ICHG: Full range 8.128 A, LSB: 64 mA
  chargeCurrent = regBuf[CS32_I2C_ADC_ICHG_REG] * 64;
  // CMPIN: Full range 3.06 V, LSB: 12 mV
  CMPINVoltage = regBuf[CS32_I2C_ADC_CMPIN_REG] * 12;
  // POR State - IIN: Full range: 12.75 A, LSB: 50 mA
  inputCurrent = regBuf[CS32_I2C_ADC_IIN_REG] * 50;
  // VBAT: Full range : 2.88 V - 19.2 V, LSB 64 mV
  batteryVoltage = 2880 + regBuf[CS32_I2C_ADC_VBAT_REG] * 64;
  if (2880 == batteryVoltage) {
    batteryVoltage = 0;
  }
  // VSYS: Full range: 2.88 V - 19.2 V, LSB: 64 mV
  systemVoltage = 2880 + regBuf[CS32_I2C_ADC_VSYS_REG] * 64;
  if (2880 == systemVoltage) {
    systemVoltage = 0;
  }
  // VSYS: Full range: 2.88 V - 19.2 V, LSB: 64 mV
  systemVoltage = 2880 + regBuf[CS32_I2C_ADC_VSYS_REG] * 64;
  if (2880 == systemVoltage) {
    systemVoltage = 0;
  }
  maxChargeVoltage = LPUPS_CONCAT_BYTES(regBuf[CS32_I2C_SET_VBAT_LIMIT_REG + 1], regBuf[CS32_I2C_SET_VBAT_LIMIT_REG]);
  memset(outputBuf, 0, sizeof(outputBuf));
  sprintf(outputBuf, "8-bit Digital Output of System Power = %u mV\r\n"
    "8-bit Digital Output of Input Voltage = %u mV\r\n"
    "8-bit digital output of battery discharge current = %u mA\r\n"
    "8-bit digital output of battery charge current = %u mA\r\n"
    "8-bit digital output of CMPIN voltage = %u mV\r\n"
    "8-bit digital output of input current = %u mA\r\n"
    "8-bit digital output of battery voltage = %u mV\r\n"
    "8-bit digital output of system voltage = %u mV\r\n"
    "The max charge voltage = %u mV\r\n",
    systemPower, inputVoltage, dischargeCurrent, chargeCurrent, CMPINVoltage, inputCurrent, batteryVoltage, systemVoltage, maxChargeVoltage);
  Serial.print(outputBuf);

}

void flashReportedData(void)
{
  // Charging status
  if (bCharging)
    bitSet(iPresentStatus, PRESENTSTATUS_CHARGING);
  else
    bitClear(iPresentStatus, PRESENTSTATUS_CHARGING);

  // AC power supply
  if (bACPresent)
    bitSet(iPresentStatus, PRESENTSTATUS_ACPRESENT);
  else
    bitClear(iPresentStatus, PRESENTSTATUS_ACPRESENT);

  // Fully charged
  if (iRemaining == iFullChargeCapacity)
    bitSet(iPresentStatus, PRESENTSTATUS_FULLCHARGE);
  else
    bitClear(iPresentStatus, PRESENTSTATUS_FULLCHARGE);

  // Discharging
  if (bDischarging) {   // Not charging
    bitSet(iPresentStatus, PRESENTSTATUS_DISCHARGING);
    // if(iRemaining < iRemnCapacityLimit) bitSet(iPresentStatus,PRESENTSTATUS_BELOWRCL);   // Below remaining capacity limit.

    // Exceeded runtime/capacity limit.
    if (iRunTimeToEmpty < iRemainTimeLimit)
      bitSet(iPresentStatus, PRESENTSTATUS_RTLEXPIRED);
    else
      bitClear(iPresentStatus, PRESENTSTATUS_RTLEXPIRED);

  } else {
    bitClear(iPresentStatus, PRESENTSTATUS_DISCHARGING);
    bitClear(iPresentStatus, PRESENTSTATUS_RTLEXPIRED);   // Clearing relevant flags during charging.
  }

  // Shutdown request.
  if (iDelayBe4ShutDown > 0) {
    bitSet(iPresentStatus, PRESENTSTATUS_SHUTDOWNREQ);
    Serial.println("shutdown requested");
  } else
    bitClear(iPresentStatus, PRESENTSTATUS_SHUTDOWNREQ);

  // Shutdown imminent.
  if ((iPresentStatus & (1 << PRESENTSTATUS_SHUTDOWNREQ)) ||
    (iPresentStatus & (1 << PRESENTSTATUS_RTLEXPIRED))) {
    bitSet(iPresentStatus, PRESENTSTATUS_SHUTDOWNIMNT);   // - Shutdown imminent.
    Serial.println("shutdown imminent");
  } else
    bitClear(iPresentStatus, PRESENTSTATUS_SHUTDOWNIMNT);

  bitSet(iPresentStatus, PRESENTSTATUS_BATTPRESENT);   // - Power BATT
}
