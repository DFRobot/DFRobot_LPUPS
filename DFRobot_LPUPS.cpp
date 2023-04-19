/*!
 * @file DFRobot_LPUPS.cpp
 * @brief  Define the infrastructure DFRobot_LPUPS class
 * @n This is a Digital Metal-Oxide Multi-Gas Sensor. It can be controlled by I2C and SPI port.
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-26
 * @url https://github.com/DFRobot/DFRobot_LPUPS
 */
#include "DFRobot_LPUPS.h"

DFRobot_LPUPS::DFRobot_LPUPS()
{
}

int DFRobot_LPUPS::begin(void)
{
  uint8_t pidBuf[2];
  if(0 == readReg(CS32_I2C_PID_REG, pidBuf, sizeof(pidBuf)))   // Judge whether the data bus is successful
  {
    DBG("ERR_DATA_BUS");
    return ERR_DATA_BUS;
  }

  DBG("real sensor id=");DBG(LPUPS_CONCAT_BYTES(pidBuf[1], pidBuf[0]));
  if(UPS_PID_VALUE != LPUPS_CONCAT_BYTES(pidBuf[1], pidBuf[0]))   // Judge whether the chip version matches
  {
    DBG("ERR_IC_VERSION");
    return ERR_IC_VERSION;
  }

  DBG("begin ok!");
  return NO_ERR;
}

/***************** Config function ******************************/

void DFRobot_LPUPS::getChipData(uint8_t * regBuf)
{
  readReg(CS32_I2C_CHARGER_STATUS_REG, regBuf, CS32_I2C_SET_VBAT_LIMIT_REG + 2);
  delay(20);
}

void DFRobot_LPUPS::setMaxChargeVoltage(uint16_t data)
{
  if (11100 > data)
    data = 11100;
  if (12600 < data)
    data = 12600;
  uint8_t dataBuf[2] = { 0 };
  dataBuf[0] = (uint8_t)(data & 0xFF);
  dataBuf[1] = (uint8_t)((data & 0xFF00) >> 8);
  writeReg(CS32_I2C_SET_VBAT_LIMIT_REG, dataBuf, 2);
  delay(20);
}

/***************** Init and read/write of I2C interfaces ******************************/

DFRobot_LPUPS_I2C::DFRobot_LPUPS_I2C(TwoWire *pWire, uint8_t i2cAddr)
{
  _deviceAddr = i2cAddr;
  _pWire = pWire;
}

int DFRobot_LPUPS_I2C::begin(void)
{
  _pWire->begin();   // Wire.h(I2C)library function initialize wire library
  return DFRobot_LPUPS::begin();   // Use the initialization function of the parent class
}

void DFRobot_LPUPS_I2C::writeReg(uint8_t reg, const void* pBuf, size_t size)
{
  if(pBuf == NULL) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);

  for(size_t i = 0; i < size; i++) {
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

size_t DFRobot_LPUPS_I2C::readReg(uint8_t reg, void* pBuf, size_t size)
{
  size_t count = 0;
  if(NULL == pBuf) {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t*)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire -> write(reg);
  if(0 != _pWire->endTransmission()) {   // Used Wire.endTransmission() to end a slave transmission started by beginTransmission() and arranged by write().
    DBG("endTransmission ERROR!!");
  } else {
    _pWire->requestFrom(_deviceAddr, (uint8_t)size);   // Master device requests size bytes from slave device, which can be accepted by master device with read() or available()
    
    while (_pWire->available()) {
      _pBuf[count++] = _pWire->read();   // Use read() to receive and put into buf
    }
    // _pWire->endTransmission();
  }
  return count;
}
