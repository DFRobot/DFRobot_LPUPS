# DFRobot_LPUPS
* [中文版](./README_CN.md)

A UPS for LP, providing corresponding power supply for LP's equipment.

![产品实物图](./resources/images/LPUPS.png)


## Product Link (https://www.dfrobot.com/search-lpups.html)
    SKU: DFR0682


## Table of Contents

* [Summary](#summary)
* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)


## Summary

* Obtain battery information from LPUPS via I2C and report this information to LP via USB-HID.


## Installation

There two methods:

0. This library uses HID and depends on the HIDPowerDevice library (https://github.com/abratchik/HIDPowerDevice). To install and run this library, you must also install its dependencies.
1. To use this library, first download the library file, paste it into the \Arduino\libraries directory, then open the examples folder and run the demo in the folder.
2. Search the DFRobot_LPUPS library from the Arduino Software Library Manager and download it.


## Methods

```C++

  /**
   * @fn begin
   * @brief Init function
   * @return int type, indicates returning init status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

  /**
   * @fn getChipData
   * @brief Retrieve chip data.
   * @param regBuf register data
   * @return None
   */
  void getChipData(uint8_t * regBuf);

  /**
   * @fn setMaxChargeVoltage
   * @brief Set maximum charging voltage.
   * @param data Maximum charging voltage, 11100 ~ 12600 mV
   * @return None
   */
  void setMaxChargeVoltage(uint16_t data);

```


## Compatibility

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Leonardo   |      √       |              |             |


## History

- 2022/11/23 - Version 1.0.0 released.
- 2023/08/09 - Version 1.0.1 released.


## Credits

Written by qsjhyy(yihuan.huang@dfrobot.com), 2022. (Welcome to our [website](https://www.dfrobot.com/))

