# DFRobot_LPUPS
* [English Version](./README.md)

一款LP的UPS，为LP的电源设备提供对应驱动。

![产品实物图](./resources/images/LPUPS.png)


## 产品链接 (https://www.dfrobot.com.cn/search.php?keywords=lpups)
    SKU: DFR0682


## 目录

* [概述](#概述)
* [库安装](#库安装)
* [方法](#方法)
* [兼容性](#兼容性)
* [历史](#历史)
* [创作者](#创作者)


## 概述

* 通过I2C获取LPUPS的电池信息, 通过USB-HID将这些电池信息上报到LP。


## 库安装

这里有2种安装方法：

0. 这个库用使用HID, 依赖于HIDPowerDevice(https://github.com/abratchik/HIDPowerDevice)这个库, 安装运行此库必须安装他的依赖库。
1. 使用此库前，请首先下载库文件，将其粘贴到\Arduino\libraries目录中，然后打开examples文件夹并在该文件夹中运行演示。
2. 直接在Arduino软件库管理中搜索下载 DFRobot_LPUPS 库。


## 方法

```C++

  /**
   * @fn begin
   * @brief 初始化函数
   * @return int类型, 表示返回初始化的状态
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

  /**
   * @fn getChipData
   * @brief 获取芯片数据
   * @param regBuf 
   * @return None
   */
  void getChipData(uint8_t * regBuf);

  /**
   * @fn setMaxChargeVoltage
   * @brief 设置最大充电电压
   * @param data 最大充电电压, 11100 ~ 12600 mV
   * @return None
   */
  void setMaxChargeVoltage(uint16_t data);

```


## 兼容性

MCU                | Work Well    | Work Wrong   | Untested    | Remarks
------------------ | :----------: | :----------: | :---------: | :----:
Arduino Leonardo   |      √       |              |             |


## 历史

- 2022/11/23 - 1.0.0 版本


## 创作者

Written by qsjhyy(yihuan.huang@dfrobot.com), 2022. (Welcome to our [website](https://www.dfrobot.com/))

