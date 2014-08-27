ardunav
=======

ArduNav is an Open Source AHRS Development platform for testing Dead Reckoning and AHRS Algorithms designed specifically for college students
 
An extremely low-cost, simple and yet accurate tool to create your own AHRS and run your sensor-fusion algorithms in Arduino Due using the following sensors - MPU6050 (accelerometer and gyroscopes), HMC 5883L (magnetometer) and NEO-6M GPS module. The ardunav also includes a HC-05 Bluetooth module to transmit data over Bluetooth to a nearby ground station. 

The project aims at building an AHRS for under $100 including the Arduino board, sensors and required hardware. 

The Firmware utilizes Eigen 3 Library for Matrix Computations and Kalman Filtering.

[![ScreenShot](http://scottadkinsworkout.org/wp-content/uploads/2012/04/scott-adkins-workout-training-plan.jpg)](https://www.youtube.com/watch?v=txDWA1ZpQMc&feature=youtu.be)

ACKNOWLEDGEMENTS:

I would like to thank Jeff Rowberg for sharing the Arduino source code for MPU6050 and HMC5883L using the I2CDev Library, Mikal Hart for building the TinyGPS library and Hazim Bitar for sharing the code on programming the HC-05 bluetooth module.

I would like to thank Prof. Demoz-Gebre Egziabher, Department of Aerospace Engineering, University of Minnesota and Mr. Brian Taylor, Director, UAV Lab, University of Minnesota for teaching me the course AEM 5333, Design to Flight:UAV which culminated in the development of Ardunav.

I would finally like to thank my faculty advisor, Prof. Perry Li, Department of Mechanical Engineering, University of Minnesota for guiding me in the field of vision guided inertial navigation.
