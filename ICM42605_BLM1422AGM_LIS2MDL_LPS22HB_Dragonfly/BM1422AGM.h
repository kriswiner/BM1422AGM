/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The BM1422AGM is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef BM1422AGM_h
#define BM1422AGM_h

#include "Arduino.h"
#include <Wire.h>
#include "I2Cdev.h"

//Register map for BM1422AGMAGM
// https://www.rohm.com/documents/11413/4994461/BM1422AGMagmv-e.pdf

#define BM1422AGM_INFO                  0x0D
#define BM1422AGM_WHO_AM_I              0x0F  // should be 0x41
#define BM1422AGM_DATAX_L               0x10
#define BM1422AGM_DATAX_H               0x11
#define BM1422AGM_DATAY_L               0x12
#define BM1422AGM_DATAY_H               0x13
#define BM1422AGM_DATAZ_L               0x14
#define BM1422AGM_DATAZ_H               0x15
#define BM1422AGM_STATUS1               0x18
#define BM1422AGM_CNTL1                 0x1B
#define BM1422AGM_CNTL2                 0x1C
#define BM1422AGM_CNTL3                 0x1D
#define BM1422AGM_AVE_A                 0x40
#define BM1422AGM_CNTL4                 0x5C
#define BM1422AGM_TEMP_L                0x60
#define BM1422AGM_TEMP_H                0x61
#define BM1422AGM_OFFSET_X_L            0x6C
#define BM1422AGM_OFFSET_X_H            0x6D
#define BM1422AGM_OFFSET_Y_L            0x72
#define BM1422AGM_OFFSET_Y_H            0x73
#define BM1422AGM_OFFSET_Z_L            0x78
#define BM1422AGM_OFFSET_Z_H            0x79
#define BM1422AGM_FINEOUTOUTX_L         0x90
#define BM1422AGM_FINEOUTOUTX_H         0x91
#define BM1422AGM_FINEOUTOUTY_L         0x92
#define BM1422AGM_FINEOUTOUTY_H         0x93
#define BM1422AGM_FINEOUTOUTZ_L         0x94
#define BM1422AGM_FINEOUTOUTZ_H         0x95
#define BM1422AGM_GAIN_PARA_X_L         0x9C
#define BM1422AGM_GAIN_PARA_X_H         0x9D
#define BM1422AGM_GAIN_PARA_Y_L         0x9E
#define BM1422AGM_GAIN_PARA_Y_H         0x9F
#define BM1422AGM_GAIN_PARA_Z_L         0xA0
#define BM1422AGM_GAIN_PARA_Z_H         0xA1

#define BM1422AGM_ADDRESS               0x0E

#define BM1422AGM_MODR_10Hz    0x00
#define BM1422AGM_MODR_20Hz    0x02
#define BM1422AGM_MODR_100Hz   0x01
#define BM1422AGM_MODR_1000Hz  0x03


class BM1422AGM
{
  public:
  BM1422AGM(I2Cdev* i2c_bus);
  uint8_t getChipID();
  void init(uint8_t MODR);
  void sleep(uint8_t MODR);
  void wake(uint8_t MODR);
  void offsetBias(float * dest1, float * dest2);
  void reset();
  uint8_t status();
  void readData(int16_t * destination);
  uint16_t readTemperature();
  private:
  float _mRes;
  I2Cdev* _i2c_bus;
};

#endif
