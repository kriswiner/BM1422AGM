/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The BM1422AGM is a low power magnetometer, here used as 3 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "BM1422AGM.h"
#include "I2Cdev.h"

BM1422AGM::BM1422AGM(I2Cdev* i2c_bus)
{
  _i2c_bus = i2c_bus;
}


uint8_t BM1422AGM::getChipID()
{
  uint8_t c = _i2c_bus->readByte(BM1422AGM_ADDRESS, BM1422AGM_WHO_AM_I);
  return c;
}


void BM1422AGM::reset()
{
  // reset device
  _i2c_bus->writeByte(BM1422AGM_ADDRESS, BM1422AGM_CNTL1, 0x20); // Set bit 5 to 1 to reset BM1422AGM
  delay(1);
  _i2c_bus->writeByte(BM1422AGM_ADDRESS, BM1422AGM_CNTL1, 0x00); // Set bit 5 to 0 to finish reset sequence
  uint8_t temp[2] = {0x00, 0x00};
  _i2c_bus->writeBytes(BM1422AGM_ADDRESS, BM1422AGM_CNTL4, 2, &temp[0]); // Write 0x00, 0x00 to CNTL4 to finish reset sequence
  delay(10); // Wait for all registers to reset 
}

void BM1422AGM::init(uint8_t MODR)
{
 
 // enable active mode (bit 7), select 14-bit output (bit 6), continuous measurement mode (bit 1 = 0)
 _i2c_bus->writeByte(BM1422AGM_ADDRESS, BM1422AGM_CNTL1, 0x80 | 0x40 | MODR << 3);  

  uint8_t temp[2] = {0x00, 0x00};
  _i2c_bus->writeBytes(BM1422AGM_ADDRESS, BM1422AGM_CNTL4, 2, &temp[0]); // Write 0x00, 0x00 to CNTL4 to finish reset sequence

 // enable interrupt (bit 3), active high (bit 2)
 _i2c_bus->writeByte(BM1422AGM_ADDRESS, BM1422AGM_CNTL2, 0x08 | 0x04);  

 // start measurements
 _i2c_bus->writeByte(BM1422AGM_ADDRESS, BM1422AGM_CNTL3, 0x40);  
}


void BM1422AGM::sleep(uint8_t MODR)
{
 //  power down (bit 7), 14-bit data (bit 6), ODR (bits 3/4) 
 _i2c_bus->writeByte(BM1422AGM_ADDRESS, BM1422AGM_CNTL1,  0x40 | MODR << 3 );  
}


void BM1422AGM::wake(uint8_t MODR)
{
 //  power down (bit 7), 14-bit data (bit 6), ODR (bits 3/4) 
 _i2c_bus->writeByte(BM1422AGM_ADDRESS, BM1422AGM_CNTL1, 0x80 | 0x40 | MODR << 3);  
}


uint8_t BM1422AGM::status()
{
  // Read the status register of the altimeter  
  uint8_t temp = _i2c_bus->readByte(BM1422AGM_ADDRESS, BM1422AGM_STATUS1);   
  return temp;
}


void BM1422AGM::readData(int16_t * destination)
{
  uint8_t rawData[6];  // x/y/z mag register data stored here
  _i2c_bus->readBytes(BM1422AGM_ADDRESS,  BM1422AGM_DATAX_L, 6, &rawData[0]);  // Read the 6 raw data registers into data array

  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;            // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


uint16_t BM1422AGM::readTemperature()
{
  uint8_t rawData[2];  // x/y/z mag register data stored here
  _i2c_bus->readBytes(BM1422AGM_ADDRESS, BM1422AGM_TEMP_L, 2, &rawData[0]);  // Read the 2 raw data registers into data array
  uint16_t temp = ((uint16_t)rawData[1] << 8) | rawData[0] ;          // Turn the MSB and LSB into a signed 16-bit value
  return temp;   
}


void BM1422AGM::offsetBias(float * dest1, float * dest2)
{
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  float _mRes = 0.00042f; // 1/24 in the ROHM example for conversion to uT
  
  Serial.println("Calculate BM1422AGM mag offset bias: move all around to sample the complete response surface!");
  delay(4000);

  for (int ii = 0; ii < 2000; ii++)
  {
    readData(mag_temp);
       for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(12);
  }

     // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0] * _mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1] * _mRes;   
    dest1[2] = (float) mag_bias[2] * _mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("BM1422AGM Mag Calibration done!");
}


