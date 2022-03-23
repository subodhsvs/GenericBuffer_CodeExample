# Simple application test for MotionFX library

### Menu:

*  -a            Auto-activate all available channels
*  -c <n>        Do n conversions
*  -l <n>        Set buffer length to n samples
*  -x <dT>       Test MotionFX with delta time dT in ms
*  -m <ms>       Mag sample time in ms (default 10)
*  -x <dT>       Test MotionFX with delta time dT in ms
*  -o <ms>       Show MotionFX output with rate (default 1000 ms)
*  -g <ms>       Select Motion FX algo (0 - 6x, 1 = 9x, default 0)
*  -s            Show sensor data output
*  -w <n>        Set delay between reads in us (event-less mode)

### Example to test on LSM6DSOX with magnetometer (LIS2MDL):

>iio_test_sensors -x 20 -a -c -1 -o 80 -g 1 -m 20 lsm6dsox_accel lsm6dsox_gyro lsm6dsox_magn

Output data:

* q = quaternion
* gb = gyro bias
* mc = magnetometer calibration offset (*only for 9x algo*)
* a = accelerometer data (g)
* g = gyroscope data (dps)
* mt = magnetometer timestamp (*only for 9x algo*)
* m = magnetometer data (uT/50) (* only for 9x algo*)

this is an example of output (9x):

>q(0.036307,-0.004179,0.998250,0.046489), gb(-0.014503,-0.277005,0.015991), mc(0.000000 0.000000 0.000000), a(0.074211,0.002012,0.996641), g(0.376949,-0.201624,0.026299), mt(19840534), m(-0.000012,-0.000609,-0.000579)
