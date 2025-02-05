/*
  HIDPowerDevice.h

  Copyright (c) 2020, Aleksandr Bratchik
  Original code (pre-library): Copyright (c) 2020, Aleksandr Bratchik

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef HIDPOWERDEVICE_h
#define HIDPOWERDEVICE_h

#define _USING_HID



#include "HID.h"
#include "WString.h"


#if !defined(_USING_HID)

#warning "Using legacy HID core (non pluggable)"

#else

//================================================================================
//================================================================================

#define HID_PD_IPRODUCT              0x01 // FEATURE ONLY
#define HID_PD_SERIAL                0x02 // FEATURE ONLY
#define HID_PD_MANUFACTURER          0x03 // FEATURE ONLY
#define IDEVICECHEMISTRY             0x04
#define IOEMVENDOR                   0x05

#define HID_PD_RECHARGEABLE          0x06 // FEATURE ONLY
#define HID_PD_PRESENTSTATUS         0x07 // INPUT OR FEATURE(required by Windows)
#define HID_PD_REMAINTIMELIMIT       0x08
#define HID_PD_MANUFACTUREDATE       0x09
#define HID_PD_CONFIGVOLTAGE         0x0A // 10 FEATURE ONLY
#define HID_PD_VOLTAGE               0x0B // 11 INPUT (NA) OR FEATURE(implemented)
#define HID_PD_REMAININGCAPACITY     0x0C // 12 INPUT OR FEATURE(required by Windows)
#define HID_PD_RUNTIMETOEMPTY        0x0D 
#define HID_PD_FULLCHRGECAPACITY     0x0E // 14 INPUT OR FEATURE. Last Full Charge Capacity 
#define HID_PD_WARNCAPACITYLIMIT     0x0F
#define HID_PD_CPCTYGRANULARITY1     0x10
#define HID_PD_REMNCAPACITYLIMIT     0x11
#define HID_PD_DELAYBE4SHUTDOWN      0x12 // 18 FEATURE ONLY
#define HID_PD_DELAYBE4REBOOT        0x13
#define HID_PD_AUDIBLEALARMCTRL      0x14 // 20 FEATURE ONLY 
#define HID_PD_CURRENT               0x15 // 21 FEATURE ONLY
#define HID_PD_CAPACITYMODE          0x16
#define HID_PD_DESIGNCAPACITY        0x17
#define HID_PD_CPCTYGRANULARITY2     0x18
#define HID_PD_AVERAGETIME2FULL      0x1A
#define HID_PD_AVERAGECURRENT        0x1B
#define HID_PD_AVERAGETIME2EMPTY     0x1C

#define HID_PD_IDEVICECHEMISTRY      0x1F // Feature
#define HID_PD_IOEMINFORMATION       0x20 // Feature


// PresenStatus dynamic flags
#define PRESENTSTATUS_CHARGING       0x00
#define PRESENTSTATUS_DISCHARGING    0x01
#define PRESENTSTATUS_ACPRESENT      0x02
#define PRESENTSTATUS_BATTPRESENT    0x03
#define PRESENTSTATUS_BELOWRCL       0x04
#define PRESENTSTATUS_RTLEXPIRED     0x05
#define PRESENTSTATUS_NEEDREPLACE    0x06
#define PRESENTSTATUS_VOLTAGENR      0x07
#define PRESENTSTATUS_FULLCHARGE     0x08
#define PRESENTSTATUS_FULLDISCHARGE  0x09
#define PRESENTSTATUS_SHUTDOWNREQ    0x0A
#define PRESENTSTATUS_SHUTDOWNIMNT   0x0B
#define PRESENTSTATUS_COMMLOST       0x0C
#define PRESENTSTATUS_OVERLOAD       0x0D





class HIDPowerDevice_  {
    
private:
    
    const byte bProduct = IPRODUCT;
    const byte bManufacturer = IMANUFACTURER;
    const byte bSerial = ISERIAL;  
    
public:
  HIDPowerDevice_(void);
  void begin(void);
  
  void setOutput(Serial_&);
  
  void setSerial(const char*);
  
  
  void end(void);
  
  int sendDate(uint16_t id, uint16_t year, uint8_t month, uint8_t day);
  int sendReport(uint16_t id, const void* bval, int len);
  
  int setFeature(uint16_t id, const void* data, int len);
  
  int setStringFeature(uint8_t id, const uint8_t* index, const char* data);

};

extern HIDPowerDevice_ PowerDevice;


#endif
#endif

