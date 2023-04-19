/*!
 * @file  DFRobot_LPUPS.h
 * @brief  Define infrastructure of DFRobot_LPUPS class
 * @details  Define macros and functions related to LPUPS.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2022-11-23
 * @url  https://github.com/DFRobot/DFRobot_LPUPS
 */
#ifndef __DFRobot_LPUPS_H__
#define __DFRobot_LPUPS_H__

#include <Arduino.h>
#include <Wire.h>


#define ENABLE_DBG   //!< Open this macro and you can see the details of the program
#ifdef ENABLE_DBG
  #define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
  #define DBG(...)
#endif


#define UPS_I2C_ADDRESS   0X55
#define UPS_PID_VALUE     0X42AA   //!< PID: The PID (Product Identifier) for the module is DFR0682. The highest two bits of the PID are used to determine the category: 00 for SEN, 01 for DFR, 10 for TEL, and 11 for BOS. The remaining 14 bits are used as the num.

/* LPUPS register address */
#define CS32_I2C_CHARGER_STATUS_REG   0x00U   //!< 21/20h
#define CS32_I2C_PROCHOT_STATUS_REG   0x02U   //!< 23/22h

#define CS32_I2C_ADC_PSYS_REG         0x06U   //!< 26h, PSYS: Full range: 3.06 V, LSB 12 mV
#define CS32_I2C_ADC_VBUS_REG         0x07U   //!< 27h, VBUS: Full range: 3.2 V - 19.52 V, LSB 64 mV
#define CS32_I2C_ADC_IDCHG_REG        0x08U   //!< 28h, IDCHG: Full range: 32.512 A, LSB 256 mA
#define CS32_I2C_ADC_ICHG_REG         0x09U   //!< 29h, ICHG: Full range: 8.128 A, LSB 64 mA
#define CS32_I2C_ADC_CMPIN_REG        0x0AU   //!< 2Ah, CMPIN: Full range: 3.06 V, LSB 12 mV
#define CS32_I2C_ADC_IIN_REG          0x0BU   //!< 2Bh, POR status - IIN: Full range: 12.75 A, LSB 50 mA
#define CS32_I2C_ADC_VBAT_REG         0x0CU   //!< 2Ch, VBAT: Full range: 2.88 V - 19.2 V, LSB 64 mV
#define CS32_I2C_ADC_VSYS_REG         0x0DU   //!< 2Dh, VSYS: Full range: 2.88 V - 19.2 V, LSB 64 mV

#define CS32_I2C_PID_REG              0x10U
#define CS32_I2C_VID_REG              0x12U
#define CS32_I2C_VERSION_REG          0x14U

#define CS32_I2C_SET_VBAT_LIMIT_REG   0x16U

/* Convenience Macro */
#define LPUPS_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data


class DFRobot_LPUPS
{
public:
  #define NO_ERR             0    // No error
  #define ERR_DATA_BUS     (-1)   // Data bus error
  #define ERR_IC_VERSION   (-2)   // Chip version error

/************************* Sensor Status *******************************/
  /**
   * @struct sChargerStatus0_t
   * @brief charging status register 0
   * @note Register structure:
   * @n ----------------------------------------------------------------------------------------------
   * @n |   b7   |   b6    |   b5   |     b4      |      b3     |     b2     |    b1     |    b0     |
   * @n ----------------------------------------------------------------------------------------------
   * @n | F_ACOV | F_BATOC | F_ACOC | SYSOVP_STAT | F_SYS_SHORT | F_LATCHOFF | F_OTG_OVP | F_OTG_UVP |
   * @n ----------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   f_otg_uvp: 1; /**< 0b: charger is not in otg;  1b: charge is in otg
                              (0b: No fault; 1b: OTG UVP) */
    uint8_t   f_otg_ovp: 1; /**< 0b: charger is not in pre-charge;  1b: charger is in pre-charge
                              (0b: No fault; 1b: OTG OVP) */
    uint8_t   f_latchoff: 1; /**< 0b: charger is not in fast charge;  1b: charger is in fast charger
                              (0b: No fault; 1b:  Latch off (REG0x30[3])) */
    uint8_t   f_sys_short: 1; /**< The fault is latched until a clear from host by writing this bit to 0.
                                   0b: No fault <default at POR>
                                   1b: When SYS is lower than 2.4V, then 7 times restart tries are failed. */
    uint8_t   sysovp_stat: 1; /**< SYSOVP Status and Clear
    When the SYSOVP occurs, this bit is HIGH. During the SYSOVP, the converter is disabled.
    After the SYSOVP is removed, the user must write a 0 to this bit or unplug the adapter to clear the SYSOVP condition to enable the converter again.
    (0b: Not in SYSOVP <default at POR>; 1b: In SYSOVP. When SYSOVP is removed, write 0 to clear the SYSOVP latch.) */
    uint8_t   f_acoc: 1; /**< The faults are latched until a read from host.
                              (0b: No fault; 1b: ACOC) */
    uint8_t   f_batoc: 1; /**< The faults are latched until a read from host.
                               (0b: No fault; 1b: BATOC) */
    uint8_t   f_acov: 1; /**< The faults are latched until a read from host.
                              (0b: No fault; 1b: ACOV) */
  } __attribute__ ((packed)) sChargerStatus0_t;

  /**
   * @struct sChargerStatus1_t
   * @brief charging status register 1
   * @note Register structure:
   * @n --------------------------------------------------------------------------------------
   * @n |    b7   |    b6    |   b5   |    b4     |    b3     |    b2    |    b1    |   b0   |
   * @n --------------------------------------------------------------------------------------
   * @n | AC_STAT | ICO_DONE | IN_VAP | IN_VINDPM | IN_IINDPM | IN_FCHRG | IN_PCHRG | IN_OTG |
   * @n --------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   in_otg: 1; /**< 0b: charger is not in otg;  1b: charge is in otg */
    uint8_t   in_pchrg: 1; /**< 0b: charger is not in pre-charge;  1b: charger is in pre-charge */
    uint8_t   in_fchrg: 1; /**< 0b: charger is not in fast charge;  1b: charger is in fast charger */
    uint8_t   in_iindpm: 1; /**< 0b: charger is not in iindpm;  1b: charger is in iindpm */
    uint8_t   in_vindpm: 1; /**< 0b: charger is not in vindpm during forward mode, or voltage regulation during otg mode;
                                 1b: charger is in vindpm during forward mode, or voltage regulation during otg mode */
    uint8_t   in_vap: 1; /**< 0b: charger is not operated in vap mode;  1b: charger is operated in vap mode */
    uint8_t   ico_done: 1; /**< after the ico routine is successfully executed, the bit goes 1.
                                (0b: ico is not complete; 1b: ico is complete) */
    uint8_t   ac_stat: 1; /**< input source status, same as chrg_ok bit.
                               (0b: input not present; 1b: input is present) */
  } __attribute__ ((packed)) sChargerStatus1_t;

  /**
   * @struct sProchotStatus0_t
   * @brief status register 0
   * @note Register structure:
   * @n -------------------------------------------------------------------------------------------------
   * @n |   b7   |  b6    |   b5    |   b4   |   b3    |   b2   |         b1        |        b0         |
   * @n -------------------------------------------------------------------------------------------------
   * @n | S_VDPM | S_COMP | S_ICRIT | S_INOM | S_IDCHG | S_VSYS | S_BATTERY_REMOVAL | S_ADAPTER_REMOVAL |
   * @n -------------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   s_adapter_removal: 1; /**< 0b: Not triggered; 1b: Triggered */
    uint8_t   s_battery_removal: 1; /**< 0b: Not triggered; 1b: Triggered */
    uint8_t   s_vsys: 1; /**< 0b: Not triggered; 1b: Triggered */
    uint8_t   s_idchg: 1; /**< 0b: Not triggered; 1b: Triggered */
    uint8_t   s_inom: 1; /**< 0b: Not triggered; 1b: Triggered */
    uint8_t   s_icrit: 2; /**< 0b: Not triggered; 1b: Triggered */
    uint8_t   s_comp: 1; /**< 0b: Not triggered; 1b: Triggered */
    uint8_t   s_vdpm: 1; /**< 0b: Not triggered; 1b: Triggered */
  } __attribute__ ((packed)) sProchotStatus0_t;

  /**
   * @struct sProchotStatus1_t
   * @brief status register 1
   * @note Register structure:
   * @n -------------------------------------------------------------------------------------
   * @n |b7 |       b6        |  b5   |  b4   |       b3      |b2 |     b1     |     b0     |
   * @n -------------------------------------------------------------------------------------
   * @n | R | EN_PROCHOT_EXIT | PROCHOT_WIDTH | PROCHOT_CLEAR | R | S_VAP_FAIL | S_EXIT_VAP |
   * @n -------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   stat_exit_vap: 1; /**< When the charger is operated in VAP mode, it can exit VAP by either 
      being disabled through host, or there is any charger faults.
      0b: PROCHOT_EXIT_VAP is not active <default at POR>
      1b: PROCHOT_EXIT_VAP is active, PROCHOT pin is low until host writes this status bit to 0. */
    uint8_t   stat_vap_fail: 1; /**< This status bit reports a failure to load VBUS 7 consecutive times in VAP mode, 
      which indicates the battery voltage might be not high enough to enter VAP mode, 
      or the VAP loading current settings are too high.
      0b: Not is VAP failure <default at POR>
      1b: In VAP failure, the charger exits VAP mode, and latches off until the host writes this bit to 0. */
    uint8_t   reserved1: 1; /**< 0b: Reserved */
    uint8_t   prochot_clear: 1; /**< PROCHOT Pulse Clear. Clear PROCHOT pulse when 0x23[6] = 1.
      0b: Clear PROCHOT pulse and drive PROCHOT pin HIGH
      1b: Idle <default at POR> */
    uint8_t   prochot_width: 2; /**< PROCHOT Pulse Width Minimum PROCHOT pulse width when REG0x23[6] = 0.
      00b: 100 us; 01b: 1 ms; 10b: 10 ms <default at POR>; 11b: 5s */
    uint8_t   en_prochot_exit: 1; /**< PROCHOT Pulse Extension Enable. When pulse extension is enabled, 
      keep the PROCHOT pin voltage LOW until host writes REG0x23[3] = 0.
      0b: Disable pulse extension <default at POR>
      1b: Enable pulse extension */
    uint8_t   reserved: 1; /**< Reserved */
  } __attribute__ ((packed)) sProchotStatus1_t;

public:

/************************ Init ********************************/
  /**
   * @fn DFRobot_LPUPS
   * @brief Constructor
   * @return None
   */
  DFRobot_LPUPS(void);

  /**
   * @fn begin
   * @brief Init function
   * @return int type, indicates returning init status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

/************************** Config function ******************************/
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

protected:

/************************** Register read/write port ******************************/

  /**
   * @fn writeReg
   * @brief Write register function, design it as a pure virtual function, implement the function body through a derived class
   * @param reg  Register address 8bits
   * @param pBuf Storage and buffer for data to be written
   * @param size Length of data to be written
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size)=0;

  /**
   * @fn readReg
   * @brief Read register function, design it as a pure virtual function, implemented by a derived class
   * @param reg  Register address 8bits
   * @param pBuf Storage and buffer for data to be read
   * @param size Length of data to be read
   * @return Return the read length, returning 0 means reading failed
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size)=0;

private:
  // Private variables
};

/************************** Init and read/write of I2C and SPI interfaces ******************************/

class DFRobot_LPUPS_I2C:public DFRobot_LPUPS
{
public:
  /**
   * @fn DFRobot_LPUPS_I2C
   * @brief Constructor, set sensor I2C communication address according to SDO pin wiring
   * @param pWire Wire object is defined in Wire.h, so just use &Wire and the methods in Wire can be pointed to and used
   * @param i2cAddr The I2C address is 0x55.
   * @return None
   */
  DFRobot_LPUPS_I2C(TwoWire *pWire=&Wire, uint8_t i2cAddr=UPS_I2C_ADDRESS);

  /**
   * @fn begin
   * @brief Subclass init function
   * @return int type, indicates returning init status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

protected:
  /**
   * @fn writeReg
   * @brief Write register value through I2C bus
   * @param reg  Register address 8bits
   * @param pBuf Storage and buffer for data to be written
   * @param size Length of data to be written
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size);

  /**
   * @fn readReg
   * @brief Read register value through I2C bus
   * @param reg  Register address 8bits
   * @param pBuf Storage and buffer for data to be read
   * @param size Length of data to be read
   * @return Return the read length, returning 0 means reading failed
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size);

private:
  TwoWire *_pWire;   // Pointer to I2C communication method
  uint8_t _deviceAddr;   // Address of the device for I2C communication
};

#endif
