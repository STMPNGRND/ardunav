/*
The MIT License (MIT)

Copyright (c) 2014 Chandra Mangipudi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

The code contains the following libraries created by Jeff Rowberg - MPU6050, HMC5883L and I2CDev,
Mikal Hart - TinyGPS.

Further the author acknowledges the support of Mr. Hazim Bitar for HC05 programming and 
arduino user -randomvibe for helping integrate Eigen library with Arduino
*/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <TinyGPS.h>
#include <string.h>
#include <Eigen313.h>
#include <Dense>
#include <LU>

using namespace Eigen;

#define PI 3.142

int MPU9150_I2C_ADDRESS = 0x68;
int time_now=0,time_prev=0,t_print=0;

MPU6050 accelgyro(0x68);
HMC5883L mag;

#define epsilon_acc 0.01
#define eul_tol 0.05
#define g 9.81
#define PI 3.142
#define scale_factor_acc 1/16384*9.81
#define scale_factor_gyro 1/131*PI/180

// --------------------------------------------- GPS Configuration ------------------------------------------------

uint8_t gps_config_change[63]={// Rate to 250 ms
                          0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFA, 0x00, 0x01, 0x00, 0x01, 0x00,
                          0x10, 0x96,
                          // Baud Change
                          0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 
                          0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00,
                          0x00, 0x00, 0xBF, 0x78,
                           
                           // Save Config
                          0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 
                          0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB}; 
TinyGPS gps;

float flat,flon,falt,course,gnd_vel;
unsigned long int age;
char gps_buffer[50];

// --------------------------------------------- Accelerometer Parameters ------------------------------------------------

#define kx 1.00122474
#define ky 1.003180317
#define kz 1.006887

#define bx -0.0407
#define by -0.0551
#define bz 0.0982

double dt=0.025;
unsigned long tnow=0,tprev=0;

// --------------------------------------------- EKF Variables ------------------------------------------------


// Variables to Collect Sensor Data
int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

// Variables for AHRS
double acc_check;
double phi, theta, psi, phi_acc, theta_acc, psi_magno;

// Matrices for AHRS determination
VectorXd euler(3),grav(3),acc(3),y_eul(3);
MatrixXd dcm(3, 3), T_gyro(3, 3), T_magno(3, 3), F_gyro(3, 3), eye33(3, 3), temp33(3, 3);
MatrixXd H_gyro(3, 3), Q_gyro(3, 3), P_gyro(3, 3), R_gyro(3, 3), K_gyro(3, 3), phi_gyro(3, 3);

VectorXd gyro_bias(3), gyro(3), cmps(3), omega(3), m(3),mag_bias(3),acc_bias(3);
MatrixXd acc_scale(3,3);


void init_euler_EKF()
{
	euler.setZero(3);
	grav << 0, 0, 9.81;
	eye33.setIdentity(3, 3);
	H_gyro = eye33;
	P_gyro = eye33; P_gyro *= 10;
	R_gyro = eye33; R_gyro *= 1;R_gyro(3,3)=0.05;
	Q_gyro = eye33; Q_gyro *= 0.5;
	gyro_bias.setZero(3);
	mag_bias<<-87.934429781140720,-180.4859983546638,88.433885777505620;
        acc_scale<<kx,0,0,0,ky,0,0,0,kz;
        acc_bias<<bx,by,bz;

}

static void get_T_gyro()
{
	phi = euler(0), theta = euler(1), psi = euler(2);

	T_gyro << 1, sin(phi)*tan(theta), cos(phi)*tan(theta),
			  0, cos(phi), -sin(phi),
			  0, sin(phi) * 1 / cos(theta), cos(phi) * 1 / cos(theta);

}
static void get_dcm()
{
	phi = euler(0), theta = euler(1), psi = euler(2);

	dcm << cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi),
		   cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi),
		   -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);

}
static void get_discrete_F_gyro()
{
	F_gyro << tan(euler[1])*(cos(euler[0])* omega[1] - sin(euler[0])* omega[2]), 1 / cos(euler[1]) * 1 / cos(euler[1])*(sin(euler[0])*omega[1] + cos(euler[0])* omega[2]), 0,
		      -(omega[1] * cos(euler[0]) + omega[2] * sin(euler[0])), 0, 0,
		      1 / cos(euler[1])*(omega[1] * cos(euler[0]) - omega[2] * sin(euler[0])), 1 / cos(euler[1])*tan(euler[1])*(omega[1] * sin(euler[0]) + omega[2] * cos(euler[0])), 0;
}
static void get_T_magno()
{
	phi = euler(0), theta = euler(1), psi = euler(2);
	T_magno << cos(theta), sin(theta)*sin(phi), sin(theta)*cos(phi),
			   0, cos(phi), -sin(phi),
			   -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi);
}

void get_bias_est()
{
  int bias_count=0,bias_count_max=1000;
  while(bias_count<bias_count_max)
  {
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    acc[0]=((double)ay)*scale_factor_acc;acc[1]=((double)ax)*scale_factor_acc;acc[2]=-((double)az)*scale_factor_acc;
    gyro[0]=((double)gy)*scale_factor_gyro;gyro[1]=((double)gx)*scale_factor_gyro;gyro[2]=-((double)gz)*scale_factor_gyro;
  
    gyro_bias(0)=(gyro_bias(0)*bias_count+gyro(0))/(bias_count+1); 
    gyro_bias(1)=(gyro_bias(1)*bias_count+gyro(1))/(bias_count+1);
    gyro_bias(2)=(gyro_bias(2)*bias_count+gyro(2))/(bias_count+1);  
    bias_count++;
  }
}
void setup() {
    delay(1000);
    Wire.begin();
    pinMode(22,OUTPUT);
    pinMode(25,OUTPUT);
    pinMode(26,OUTPUT);
    pinMode(30,OUTPUT);
    
    pinMode(46,OUTPUT);
    pinMode(50,OUTPUT);
    
    digitalWrite(22,HIGH);
    digitalWrite(25,HIGH);
    digitalWrite(26,HIGH);
    digitalWrite(30,HIGH);
    
    digitalWrite(46,LOW);
    digitalWrite(50,LOW);
 
    Serial.begin(115200);
    Serial2.begin(115200);
    
    Serial.println("Enter 'a' to start");
    
    while(1)
    {
      char c=Serial2.read();
      char c1=Serial.read();
      if(c=='a'||c1=='a')
        break;
    }
    
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    
    Serial.println("Testing device connections--- MPU6050 ");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
   
    mag.initialize();
     
    Serial.println("Testing device connections---HMC5883L");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
    
    Serial1.begin(38400);
    Serial1.write(gps_config_change,sizeof(gps_config_change));
    Serial1.end();
    Serial1.begin(115200);
   
    Serial.println("Initializing Matrices");
    init_euler_EKF();
    Serial.println("Initialized Matrices Successfully");
    
    Serial.println("Starting Gyro Bias Estimation");
    get_bias_est();
    Serial.println("Completed Gyro Bias Estimation");
    Serial.println("Starting AHRS ");
    tprev=millis();
    t_print=millis();
    theta_acc = asin(acc[0] / acc.norm());
    phi_acc = asin(-acc[1] / acc.norm() / cos(theta_acc));
    get_T_magno();
    m = T_magno*cmps;
    psi_magno = -atan2(m(1), m(0));
    euler(0)=phi_acc;
    euler(1)=theta_acc;
    euler(2)=psi_magno;
}

void loop() {
  
     tnow=millis();
     dt=((double)(tnow-tprev))/1000.0f;
     tprev=tnow;
     
     //----------------------------------- Get Acc, Mag and Gyro Scaled Data ------------------------------------------------
     
     accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
     mag.getHeading(&mx, &my, &mz);
     acc[0]=((double)ay)*scale_factor_acc;acc[1]=((double)ax)*scale_factor_acc;acc[2]=-((double)az)*scale_factor_acc;
     gyro[0]=((double)gy)*scale_factor_gyro;gyro[1]=((double)gx)*scale_factor_gyro;gyro[2]=-((double)gz)*scale_factor_gyro;
     
     // Remove Scale factor and Bias from Sensor Data
     gyro-=gyro_bias;
     
     acc=acc_scale*(acc-acc_bias);
//     acc[0]=kx*(acc[0]-bx);
//     acc[1]=ky*(acc[1]-by);
//     acc[2]=kz*(acc[2]-bz);

     cmps[0]=-(((double)mx)-mag_bias[0]);
     cmps[1]= (((double)my)-mag_bias[1]);
     cmps[2]=-(((double)mz)-mag_bias[2]);
     
     
     
        //------------------------------------Attitude Time Update--------------------------------------------------------------
 
	get_T_gyro();
	omega = T_gyro*gyro;
	euler += dt*omega;
	get_discrete_F_gyro();
	phi_gyro = (eye33 + F_gyro*dt);
	P_gyro = phi_gyro*P_gyro*phi_gyro.transpose() + Q_gyro;
	
	
	//------------------------------------Attitude Measurement Update--------------------------------------------------------------
	
	
	double acc_check = abs(acc.norm() / g - 1);
	if (acc_check<epsilon_acc)
	{
		theta_acc = asin(acc[0] / acc.norm());
		phi_acc = asin(-acc[1] / acc.norm() / cos(theta_acc));
	}
	else
	{
		phi_acc = euler[0];
		theta_acc = euler[1];
	}
	
	get_T_magno();
	m = T_magno*cmps;
	psi_magno = -atan2(m(1), m(0));
	y_eul << phi_acc, theta_acc, psi_magno;
	
	temp33 = H_gyro*P_gyro*H_gyro.transpose() + R_gyro;
	K_gyro = P_gyro*H_gyro*temp33.inverse();
	euler += K_gyro*(y_eul - H_gyro*euler);
	P_gyro = (eye33 - K_gyro*H_gyro)*P_gyro;
	
	//------------------------------------Print Data in 500ms Intervals--------------------------------------------------------------
        
     if(millis()-t_print>500)
     {
       
       Serial.print("dt=");Serial.println(dt,4);
       Serial.print("phi=");Serial.println(euler(0)*180/PI,4);
       Serial.print("theta=");Serial.println(euler(1)*180/PI,4);
       Serial.print("psi=");Serial.println(euler(2)*180/PI,4);
     
       Serial.print("lat=");Serial.println(flat,8);
       Serial.print("lon=");Serial.println(flon,8);
       Serial.print("alt=");Serial.println(falt,8);
       Serial.print("course");Serial.println(course,8);
       Serial.print("speed=");Serial.println(gnd_vel,8);
       
       Serial2.print("dt=");Serial2.println(dt,4);
       Serial2.print("phi=");Serial2.println(euler(0)*180/PI,4);
       Serial2.print("theta=");Serial2.println(euler(1)*180/PI,4);
       Serial2.print("psi=");Serial2.println(euler(2)*180/PI,4);
     
       Serial2.print("lat=");Serial2.println(flat,8);
       Serial2.print("lon=");Serial2.println(flon,8);
       Serial2.print("alt=");Serial2.println(falt,8);
       Serial2.print("course");Serial2.println(course,8);
       Serial2.print("speed=");Serial2.println(gnd_vel,8);
    
       t_print=millis(); 
     }
     
     
     //------------------------------------GPS Measurement Update--------------------------------------------------------------

     long time1=millis();
     while (Serial1.available()>0)
     {
      char ch=Serial1.read();
      int count=0;
      gps.encode(ch);
      
      if(millis()-time1>10 || ch=='\n')
        break;
     }
     gps.f_get_position(&flat, &flon, &age);
     gnd_vel=gps.f_speed_mps();
     falt=gps.f_altitude();
     course=gps.f_course();

     //------------------------------------Check for End of Program--------------------------------------------------------------

      if(Serial.available()>0)
      { 
        char c=Serial.read();
        if(c=='b')
          {
            Wire.endTransmission();
            exit(0);
          }
      }
      
      if(Serial2.available()>0)
       {
          char ch=Serial2.read();
          if(ch=='b')
          {
            Wire.endTransmission();
            exit(0);
          }
          
       }

}
