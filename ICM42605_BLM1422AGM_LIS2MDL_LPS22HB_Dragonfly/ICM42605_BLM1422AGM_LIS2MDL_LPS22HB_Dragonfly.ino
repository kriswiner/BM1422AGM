#include "ICM42605.h"
#include "LIS2MDL.h"
#include "BM1422AGM.h"
#include "LPS22HB.h"
#include <RTC.h>
#include "I2Cdev.h"

#define I2C_BUS          Wire                           // Define the I2C bus (Wire instance) you wish to use

I2Cdev                   i2c_0(&I2C_BUS);               // Instantiate the I2Cdev object and point to the desired I2C bus

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed 26

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t sumCount1 = 0, sumCount2 = 0;             // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;    // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix 1 coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33;            // rotation matrix 2 coefficients for Euler angles and gravity components
float deltat1 = 0.0f, sum1 = 0.0f;          // integration interval for both filter schemes
float deltat2 = 0.0f, sum2 = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, firstUpdate1 = 0; // used to calculate integration interval
uint32_t lastUpdate2 = 0, firstUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                         // used to calculate integration interval
float lin_ax1, lin_ay1, lin_az1, lin_ax2, lin_ay2, lin_az2;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
uint32_t count = 0;
volatile bool sleepFlag = true;

//ICM42605 definitions
#define ICM42605_intPin1  9  // interrupt1 pin definitions, significant motion
#define ICM42605_intPin2 10  // interrupt2 pin definitions, data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_15_125DPS, GFS_31_25DPS, GFS_62_5DPS, GFS_125DPS, GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_1_5625Hz, AODR_3_125Hz, AODR_6_25Hz, AODR_12_5Hz, AODR_25Hz, AODR_50Hz, AODR_100Hz, AODR_200Hz, AODR_500Hz, AODR_1000Hz, AODR_2000Hz, AODR_4000Hz, AODR_8000Hz
      GODR_12_5Hz, GODR_25Hz, GODR_50Hz, GODR_100Hz, GODR_200Hz, GODR_500Hz, GODR_1000Hz, GODR_2000Hz, GODR_4000Hz, GODR_8000Hz
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_250DPS, AODR = AODR_1000Hz, GODR = GODR_1000Hz;

float aRes, gRes;               // scale resolutions per LSB for the accel and gyro sensor2
float accelBias[3] = {0., 0., 0.}, gyroBias[3] = {0., 0., 0.}; // offset biases for the accel and gyro
int16_t ICM42605Data[7];        // Stores the 16-bit signed sensor output
float   Gtemperature;           // Stores the real internal gyro temperature in degrees Celsius
float ax, ay, az, gx, gy, gz;  // variables to hold latest accel/gyro data values 
uint8_t ICM42605status;

bool newICM42605Data = false;
bool newICM42605Tap  = false;

ICM42605 ICM42605(&i2c_0); // instantiate ICM42605 class


//LIS2MDL definitions
#define LIS2MDL_intPin  11 // interrupt for magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: LIS2MDL_MODR_10Hz, LIS2MDL_MODR_20Hz, LIS2MDL_MODR_50 Hz and LIS2MDL_MODR_100Hz
*/ 
uint8_t MODR1 = LIS2MDL_MODR_100Hz;

float mRes1 = 0.0015f;           // mag sensitivity
float magBias1[3] = {0., 0., 0.}, magScale1[3]  = {0., 0., 0.}; // Bias corrections for magnetometer
int16_t LIS2MDLData[4];          // Stores the 16-bit signed sensor output
float Mtemperature1;             // Stores the real internal chip temperature in degrees Celsius
float mx1, my1, mz1;             // variables to hold latest mag data values 
uint8_t LIS2MDLstatus;

bool newLIS2MDLData = false;

LIS2MDL LIS2MDL(&i2c_0); // instantiate LIS2MDL class


//BM1422AGM definitions
#define BM1422AGM_intPin  8 // interrupt for BM1422AGM magnetometer data ready

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: BM1422AGM_MODR_10Hz, BM1422AGM_MODR_20Hz, BM1422AGM_MODR_100 Hz and BM1422AGM_MODR_1000Hz
*/ 
uint8_t MODR2 = BM1422AGM_MODR_100Hz;

float mRes2 = 0.00042f;            // BM1422AGM mag sensitivity
float magBias2[3] = {0., 0., 0.}, magScale2[3]  = {0., 0., 0.}; // Bias corrections for magnetometer
int16_t BM1422AGMData[4];          // Stores the 16-bit signed sensor output
float Mtemperature2;               // Stores the real internal chip temperature in degrees Celsius
float mx2, my2, mz2;               // variables to hold latest mag data values 
uint8_t BM1422AGMstatus; 

bool newBM1422AGMData = false;

BM1422AGM BM1422AGM(&i2c_0); // instantiate BM1422AGM class


// LPS22H definitions
uint8_t LPS22H_intPin = 12;

/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_25Hz;     // set pressure amd temperature output data rate
uint8_t LPS22Hstatus;
float Temperature, Pressure, altitude;

bool newLPS22HData = false;

LPS22H LPS22H(&i2c_0);


// RTC set time using STM32L4 native RTC class
/* Change these values to set the current initial time */
uint8_t seconds = 0;
uint8_t minutes = 15;
uint8_t hours = 17;

/* Change these values to set the current initial date */
uint8_t day = 4;
uint8_t month = 7;
uint8_t year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false; // for RTC alarm interrupt


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led off

  pinMode(LIS2MDL_intPin, INPUT);
  pinMode(BM1422AGM_intPin, INPUT);
//  pinMode(LPS22H_intPin, INPUT);
  pinMode(ICM42605_intPin1, INPUT);
  pinMode(ICM42605_intPin2, INPUT);
 
  Wire.begin(TWI_PINS_20_21); // set master mode 
  Wire.setClock(400000); // I2C frequency at 400 kHz  
  delay(1000);
 
  i2c_0.I2Cscan();

  // Read the Chip ID registers, this is a good test of communication
  Serial.println("ICM42605 accel/gyro...");
  byte c = ICM42605.getChipID();  // Read CHIP_ID register for ICM42605
  Serial.print("ICM42605 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x42, HEX);
  Serial.println(" ");
  delay(1000); 

  Serial.println("LIS2MDL mag...");
  byte d = LIS2MDL.getChipID();  // Read CHIP_ID register for LIS2MDL
  Serial.print("LIS2MDL "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
  Serial.println(" ");
  delay(1000); 

  Serial.println("LPS22HB barometer...");
  uint8_t e = LPS22H.getChipID();
  Serial.print("LPS25H "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
  Serial.println(" ");
  delay(1000); 
  
  Serial.println("BM1422AGM mag...");
  byte f = BM1422AGM.getChipID();  // Read CHIP_ID register for BM1422AGM
  Serial.print("BM1422AGM "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" I should be "); Serial.println(0x41, HEX);
  Serial.println(" ");
  delay(1000); 
  
  if(c == 0x42 && d == 0x40 && e == 0xB1 && f == 0x41) // check if all I2C sensors have acknowledged
  {
   Serial.println("ICM42605 and LIS2MDL and LPS22HB and BM1422AGM are online..."); Serial.println(" ");
   
   digitalWrite(myLed, LOW);

   LIS2MDL.reset(); // software reset LIS2MDL to default registers
   
   LIS2MDL.init(MODR1);

   LIS2MDL.selfTest();

   LIS2MDL.offsetBias(magBias1, magScale1);
   Serial.println("LIS2MDL mag biases (mG)"); Serial.println(1000.0f * magBias1[0]); Serial.println(1000.0f * magBias1[1]); Serial.println(1000.0f * magBias1[2]); 
   Serial.println("LIS2MDL mag scale factor"); Serial.println(magScale1[0]); Serial.println(magScale1[1]); Serial.println(magScale1[2]); 
   delay(2000); // add delay to see results before serial spew of data

   BM1422AGM.reset(); // software reset LIS2MDL to default registers
   
   BM1422AGM.init(MODR2);

   BM1422AGM.offsetBias(magBias2, magScale2);
   Serial.println("BM1422AGM mag biases (mG)"); Serial.println(1000.0f * magBias2[0]); Serial.println(1000.0f * magBias2[1]); Serial.println(1000.0f * magBias2[2]); 
   Serial.println("BM1422AGM mag scale factor"); Serial.println(magScale2[0]); Serial.println(magScale2[1]); Serial.println(magScale2[2]); 
   delay(2000); // add delay to see results before serial spew of data

   LPS22H.Init(PODR);  // Initialize LPS22H altimeter
   delay(1000);

   ICM42605.reset();  // software reset ICM42605 to default registers

   // get sensor resolutions, only need to do this once
   aRes = ICM42605.getAres(Ascale);
   gRes = ICM42605.getGres(Gscale);

   ICM42605.init(Ascale, Gscale, AODR, GODR);

//   ICM42605.selfTest();

   ICM42605.offsetBias(accelBias, gyroBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println("gyro biases (dps)"); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
   delay(1000); 

   digitalWrite(myLed, HIGH);
   
  }
  else 
  {
  if(c != 0x42) Serial.println(" ICM42605 not functioning!");
  if(d != 0x40) Serial.println(" LIS2MDL not functioning!");    
  if(e != 0xB1) Serial.println(" LPS22HB not functioning!");   
  if(e != 0x41) Serial.println(" BM1422AGM not functioning!");   

  while(1){};
  }

 // Set the time
  SetDefaultRTC();
  
  // Set up the RTC alarm interrupt //
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(ICM42605_intPin1 , myinthandler1, RISING);  // define interrupt for intPin1 output of ICM42605
  attachInterrupt(LIS2MDL_intPin   , myinthandler2, RISING);  // define interrupt for intPin  output of LIS2MDL
//  attachInterrupt(LPS22H_intPin    , myinthandler3, RISING);  // define interrupt for intPin  output of LPS22HB
  attachInterrupt(BM1422AGM_intPin , myinthandler4, RISING);  // define interrupt for intPin  output of BM1422AGM

  LIS2MDL.readData(LIS2MDLData);  // read data register to clear interrupt before main loop
  ICM42605.status();
  BM1422AGM.readData(BM1422AGMData); 
}

// End of setup //

void loop() {

   // If intPin goes high, either all data registers have new data
   if(newICM42605Data == true) {   // On interrupt, read data
      newICM42605Data = false;     // reset newData flag

     ICM42605status = ICM42605.status();   // INT1 cleared on status read

     ICM42605.readData(ICM42605Data); 
   
   // Now we'll calculate the accleration value into actual g's
     ax = (float)ICM42605Data[1]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
     ay = (float)ICM42605Data[2]*aRes - accelBias[1];   
     az = (float)ICM42605Data[3]*aRes - accelBias[2];  

   // Calculate the gyro value into actual degrees per second
     gx = (float)ICM42605Data[4]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
     gy = (float)ICM42605Data[5]*gRes - gyroBias[1];  
     gz = (float)ICM42605Data[6]*gRes - gyroBias[2]; 

    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now1 = micros();
    deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate1 = Now1;

    sum1 += deltat1; // sum for averaging filter update rate
    sumCount1++;

    MadgwickQuaternionUpdate1(-ay, -ax, az, gy*pi/180.0f, gx*pi/180.0f, -gz*pi/180.0f,  -my1,  mx1, -mz1);
    }

    for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    Now2 = micros();
    deltat2 = ((Now2 - lastUpdate2)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate2 = Now2;

    sum2 += deltat2; // sum for averaging filter update rate
    sumCount2++;

    MadgwickQuaternionUpdate2(-ay, -ax, az, gy*pi/180.0f, gx*pi/180.0f, -gz*pi/180.0f,  my2,  mx2, -mz2);
    }
  }

   // If intPin goes high, either all data registers have new data
   if(newLIS2MDLData == true) {   // On interrupt, read data
      newLIS2MDLData = false;     // reset newData flag

     LIS2MDLstatus = LIS2MDL.status();
     
     if(LIS2MDLstatus & 0x08) // if all axes have new data ready
     {
      LIS2MDL.readData(LIS2MDLData);  
   
   // Now we'll calculate the accleration value into actual G's
     mx1 = (float)LIS2MDLData[0]*mRes1 - magBias1[0];  // get actual G value 
     my1 = (float)LIS2MDLData[1]*mRes1 - magBias1[1];   
     mz1 = (float)LIS2MDLData[2]*mRes1 - magBias1[2]; 
     mx1 *= magScale1[0];
     my1 *= magScale1[1];
     mz1 *= magScale1[2];  
     }
   }

   // If intPin goes high, either all data registers have new data
   if(newBM1422AGMData == true) {   // On interrupt, read data
      newBM1422AGMData = false;     // reset newData flag

     BM1422AGMstatus = BM1422AGM.status();
     
     if(BM1422AGMstatus & 0x40) // if all axes have new data ready
     {

     BM1422AGM.readData(BM1422AGMData);  
   
   // Now we'll calculate the accleration value into actual G's
     mx2 = (float)BM1422AGMData[0]*mRes2 - magBias2[0];  // get actual G value 
     my2 = (float)BM1422AGMData[1]*mRes2 - magBias2[1];   
     mz2 = (float)BM1422AGMData[2]*mRes2 - magBias2[2]; 
     mx2 *= magScale2[0];
     my2 *= magScale2[1];
     mz2 *= magScale2[2];  
     }
   }

    
   // end sensor interrupt handling

    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved and the MPU9250 is awake
       alarmFlag = false;

    // Read RTC
   if(SerialDebug)
    {
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }
    
    if(SerialDebug) {
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    Serial.print("gx = "); Serial.print( gx, 2); 
    Serial.print(" gy = "); Serial.print( gy, 2); 
    Serial.print(" gz = "); Serial.print( gz, 2); Serial.println(" deg/s");
    Serial.print("mx1 = "); Serial.print((int)1000*mx1);  
    Serial.print(" my1 = "); Serial.print((int)1000*my1); 
    Serial.print(" mz1 = "); Serial.print((int)1000*mz1); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    Serial.println(" ");

    Serial.print("mx2 = "); Serial.print((int)1000*mx2);  
    Serial.print(" my2 = "); Serial.print((int)1000*my2); 
    Serial.print(" mz2 = "); Serial.print((int)1000*mz2); Serial.println(" mG");

    Serial.print("Q0 = "); Serial.print(Q[0]);
    Serial.print(" Qx = "); Serial.print(Q[1]); 
    Serial.print(" Qy = "); Serial.print(Q[2]); 
    Serial.print(" Qz = "); Serial.println(Q[3]); 
    Serial.println(" ");
    }

    // get pressure and temperature from the LPS22HB
    LPS22Hstatus = LPS22H.status();  // poll since only sampling at 1 Hz

    if(LPS22Hstatus & 0x01) { // if new pressure data available
    Pressure = (float) LPS22H.readAltimeterPressure()/4096.0f;
    Temperature = (float) LPS22H.readAltimeterTemperature()/100.0f; 
    
    altitude = 145366.45f*(1.0f - pow((Pressure/1013.25f), 0.190284f)); 

      if(SerialDebug) {
      Serial.print("Altimeter temperature = "); Serial.print( Temperature, 2); Serial.println(" C"); // temperature in degrees Celsius  
      Serial.print("Altimeter temperature = "); Serial.print(9.0f*Temperature/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
      Serial.print("Altimeter pressure = "); Serial.print(Pressure, 2);  Serial.println(" mbar");// pressure in millibar
      Serial.print("Altitude = "); Serial.print(altitude, 2); Serial.println(" feet");
      }
    }

    Gtemperature = ((float) ICM42605Data[0]) / 132.48f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro temperature is ");  Serial.print(Gtemperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    LIS2MDLData[3] = LIS2MDL.readTemperature();
    Mtemperature1 = ((float) LIS2MDLData[3]) / 8.0f + 25.0f; // Mag chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("LIS2MDL Mag temperature is ");  Serial.print(Mtemperature1, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    BM1422AGMData[3] = BM1422AGM.readTemperature();
    Mtemperature2 = ((float) BM1422AGMData[3]) / 100.0f - 40.0f; // Mag chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("BM1422AGM Mag temperature is ");  Serial.print(Mtemperature2, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch1 = -asinf(a32);
    roll1  = atan2f(a31, a33);
    yaw1   = atan2f(a12, a22);
    pitch1 *= 180.0f / pi;
    yaw1   *= 180.0f / pi; 
    yaw1   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw1 < 0) yaw1   += 360.0f; // Ensure yaw stays between 0 and 360
    roll1  *= 180.0f / pi;
    lin_ax1 = ax + a31;
    lin_ay1 = ay + a32;
    lin_az1 = az - a33;

    if(SerialDebug) {
    Serial.print("Yaw1, Pitch1, Roll1: ");
    Serial.print(yaw1, 2);
    Serial.print(", ");
    Serial.print(pitch1, 2);
    Serial.print(", ");
    Serial.println(roll1, 2);

    Serial.print("Grav_x1, Grav_y1, Grav_z1: ");
    Serial.print(-a31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-a32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(a33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax1, Lin_ay1, Lin_az1: ");
    Serial.print(lin_ax1*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay1*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az1*1000.0f, 2);  Serial.println(" mg");

    Serial.print("rate1 = "); Serial.print((float)sumCount1/sum1, 2); Serial.println(" Hz");
    }
 
    A12 =   2.0f * (Q[1] * Q[2] + Q[0] * Q[3]);
    A22 =   Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
    A31 =   2.0f * (Q[0] * Q[1] + Q[2] * Q[3]);
    A32 =   2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
    A33 =   Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
    pitch2 = -asinf(A32);
    roll2  = atan2f(A31, A33);
    yaw2   = atan2f(A12, A22);
    pitch2 *= 180.0f / pi;
    yaw2   *= 180.0f / pi; 
    yaw2   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw2 < 0) yaw2   += 360.0f; // Ensure yaw stays between 0 and 360
    roll2 *= 180.0f / pi;
    lin_ax2 = ax + A31;
    lin_ay2 = ay + A32;
    lin_az2 = az - A33;
    
    if(SerialDebug) {
    Serial.print("Yaw2, Pitch2, Roll2: ");
    Serial.print(yaw2, 2);
    Serial.print(", ");
    Serial.print(pitch2, 2);
    Serial.print(", ");
    Serial.println(roll2, 2);

    Serial.print("Grav_x2, Grav_y2, Grav_z2: ");
    Serial.print(-A31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-A32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(A33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax2, Lin_ay2, Lin_az2: ");
    Serial.print(lin_ax2*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay2*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az2*1000.0f, 2);  Serial.println(" mg");
    
    Serial.print("rate2 = "); Serial.print((float)sumCount2/sum2, 2); Serial.println(" Hz");
    }

    sumCount1 = 0;
    sum1 = 0;      
    sumCount2 = 0;
    sum2 = 0;    

//     Serial.print(millis()/1000);Serial.print(",");
//     Serial.print(yaw, 2); Serial.print(","); Serial.print(pitch, 2); Serial.print(","); Serial.print(roll, 2); Serial.print(","); Serial.println(Pressure, 2);

    digitalWrite(myLed, LOW); delay(1); digitalWrite(myLed, HIGH);   
    }
}

// End of main loop //


void myinthandler1()
{
  newICM42605Data = true;
}

void myinthandler2()
{
  newLIS2MDLData = true;
}

void myinthandler3()
{
  newLPS22HData = true;
}

void myinthandler4()
{
  newBM1422AGMData = true;
}

void alarmMatch()
{
  alarmFlag = true;
}

void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
  

