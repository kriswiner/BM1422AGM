# BM1422AGM
Arduino sketch to compare Rohm's BH1422AGM 3-axis magnetometer with ST's LIS2MDL 3-axis magnetimeter. Both are paired with TDK's ICM42605 3-axis accel/gyro and there is an ST LPS22HB barometer on the board just for fun. Either of the mags would be part of a standard 10 DoF motion sensor suite for use in, say, UAV navigation.

Both the LIS2MDL and BM1422AGM use magnetoresistive magnetic elements for magnetic field detecvtion as opposed to the standard Hall sensor. The former are much less susceptible to temperature effects and generally have ultra-low jitter and drift. The further advantage of the BM1422AGM is that it 1) can run at 1000 Hz and 2) has a field resolution of 0.42 mGuass (0.042 uT). This compares to the LIS2MDL with maximum sample rate of 100 Hz and resolution 1.5 mGuass (0.15 uT).

Do these differences offer any significant improvement in the accuracy of absolute orientation estimation, in particular, heading accuracy? We designed this breakout board to find out:

![breakout board](https://user-images.githubusercontent.com/6698410/61560919-7d576400-aa22-11e9-9299-d4272fe04fb5.jpg)

Testing is currently underway.

