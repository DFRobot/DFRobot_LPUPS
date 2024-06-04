
#ifndef __LPUPS_DEF_H__
#define __LPUPS_DEF_H__

#include <Arduino.h>

/**
 * Battery voltage range: 9.2V ~ 12.6V, in order to keep the battery stable at extreme values:
 * Assuming the battery voltage range is 9.3V ~ 12.5V, corresponding to battery capacity 0 ~ 100.
 * Note: You can adjust the battery capacity more accurately by correcting the voltage mutation with dischargeCurrent if interested.
 */
#define MIN_BATTERY_VOLTAGE   9300   // Lower battery voltage limit
#define MAX_BATTERY_VOLTAGE   12500   // Upper battery voltage limit

#define UPS_GREEN_LED   9    // Battery level indicator LED, green
#define UPS_RED_LED     10   // Battery level indicator LED, red
#define UPS_BLUE_LED    13   // Output refresh every 1 second, indicates Arduino cycle is running, blue

#define MIN_UPDATE_INTERVAL   26 // Minimum update interval for USB-HID

#define DATA_LEN_MAX   0x24U
extern uint8_t regBuf[DATA_LEN_MAX];

extern DFRobot_LPUPS_I2C::sChargerStatus1_t chargerStatus1;

extern uint16_t dischargeCurrent, chargeCurrent;
extern uint16_t batteryVoltage, maxChargeVoltage;
extern byte iRemaining;
extern bool bCharging, bACPresent, bDischarging; // Whether charging, AC power present, discharging

extern uint16_t iRunTimeToEmpty, iAvgTimeToEmpty;   // 12

extern uint16_t iPresentStatus;   // Now and previous device status.

void initPowerDevice(void);
void printChargeData(void);
void flashReportedData(void);

#endif /* __LPUPS_DEF_H__ */
