// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors. 
// The effect of temperature has not been taken into account so I can't promise that it will work if you 
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

/* ==========  LICENSE  ==================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 =========================================================
 */

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=1024;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=32;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax, ay, az, gx, gy, gz;
int16_t leftaxoff, leftayoff, leftazoff, leftgxoff, leftgyoff, leftgzoff;
int16_t rightaxoff, rightayoff, rightazoff, rightgxoff, rightgyoff, rightgzoff;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  pinMode(50, OUTPUT);
  digitalWrite(50, LOW);
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
  //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.

  // initialize serial communication
  Serial.begin(115200);

  // initialize device
  accelgyro.initialize();


  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 first connection successful" : "MPU6050 first connection failed");

  double sumax = 0; double sumay = 0; double sumaz = 0; double sumgx = 0; double sumgy = 0; double sumgz = 0;
 for (int i = 0; i < 1000; i++) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumax += ax * 1.0 / 32768; sumay += ay * 1.0 / 32768; sumaz += az * 1.0 / 32768;
        sumax += gx * 1.0 / 32768; sumax += gy * 1.0 / 32768; sumax += gz * 1.0 / 32768;
   }
//   Serial.println("TOTAL:");
//   Serial.println(prevax);
   leftaxoff = sumax/1000.0*32768;
   leftayoff = sumay/1000.0*32768;
   leftazoff = sumaz/1000.0*32768;
   leftgxoff = sumgx/1000.0*32768;
   leftgyoff = sumgy/1000.0*32768;
   leftgzoff = sumgz/1000.0*32768;

  pinMode(50, OUTPUT);
  digitalWrite(50, HIGH);
  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);

  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 second connection successful" : "MPU6050 second connection failed");

   sumax = 0; sumay = 0; sumaz = 0; sumgx = 0; sumgy = 0; sumgz = 0;
   for (int i = 0; i < 1000; i++) {
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumax += ax * 1.0 / 32768; sumay += ay * 1.0 / 32768; sumaz += az * 1.0 / 32768;
        sumax += gx * 1.0 / 32768; sumax += gy * 1.0 / 32768; sumax += gz * 1.0 / 32768;
   }
   rightaxoff = sumax/1000.0*32768;
   rightayoff = sumay/1000.0*32768;
   rightazoff = sumaz/1000.0*32768;
   rightgxoff = sumgx/1000.0*32768;
   rightgyoff = sumgy/1000.0*32768;
   rightgzoff = sumgz/1000.0*32768;
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  pinMode(50, OUTPUT);
  digitalWrite(50, LOW);
  pinMode(53, OUTPUT);
  digitalWrite(53, HIGH);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= leftaxoff; ay -= leftayoff; az -= leftazoff;
  gx -=leftgxoff; gy -=leftgyoff; gz -= leftgzoff;
  Serial.print("left,");
  Serial.print(ax); Serial.print(",");Serial.print(ay); Serial.print(",");Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");Serial.print(gy);Serial.print(",");Serial.println(gz);
  
  pinMode(50, OUTPUT);
  digitalWrite(50, HIGH);
  pinMode(53, OUTPUT);
  digitalWrite(53, LOW);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  ax -= rightaxoff; ay -= rightayoff; az -= rightazoff;
  gx -=rightgxoff; gy -=rightgyoff; gz -= rightgzoff;
  Serial.print("left,");
  Serial.print(ax); Serial.print(",");Serial.print(ay); Serial.print(",");Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");Serial.print(gy);Serial.print(",");Serial.println(gz);
  
//  Serial.println(printThis);
//  if (state == 0) {
//    pinMode(50, OUTPUT);
//  digitalWrite(50, LOW);
//  pinMode(53, OUTPUT);
//  digitalWrite(53, HIGH);
//  
//  leftaxoff = prevax/1000;
//  leftayoff = prevay/1000;
//  leftazoff = prevaz/1000;
//  leftgxoff = prevgx/1000;
//  leftgyoff = prevgy/1000;
//  leftgyoff = prevgy/1000;
//  pinMode(50, OUTPUT);
//  digitalWrite(50, HIGH);
//  pinMode(53, OUTPUT);
//  digitalWrite(53, LOW);
//  prevax = 0; prevay = 0; prevaz = 0; prevgx = 0; prevgy = 0; prevgz = 0;
//  for (int i = 0; i < 1000; i++) {
//      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//      prevax += ax; prevay +=ay; prevaz += az;
//      prevgx += gx; prevgy +=gy; prevgz += gz;
//  }
//  rightaxoff = prevax/1000;
//  rightayoff = prevay/1000;
//  rightazoff = prevaz/1000;
//  rightgxoff = prevgx/1000;
//  rightgyoff = prevgy/1000;
//  rightgyoff = prevgy/1000;
//    state++;
//  } else {
//  pinMode(50, OUTPUT);
//  digitalWrite(50, LOW);
//  pinMode(53, OUTPUT);
//  digitalWrite(53, HIGH);
//  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  ax = ax - leftaxoff;
//  ay = ay - leftayoff;
//  az = az - leftazoff;
//  gx = gx - leftgxoff;
//  gy = gy - leftgyoff;
//  gz = gz - leftgzoff;

//  Serial.println("leftaxoff: " + leftazoff);
//  Serial.println("leftayoff: " + leftayoff);
//  Serial.println("leftazoff: " + leftazoff);
//  Serial.println("leftgxoff: " + leftgxoff);
//  Serial.println("leftgyoff: " + leftgyoff);
//  Serial.println("leftgzoff: " + leftgzoff);
//  delay(30);
//  pinMode(50, OUTPUT);
//  digitalWrite(50, HIGH);
//  pinMode(53, OUTPUT);
//  digitalWrite(53, LOW);
//  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  ax = ax - rightaxoff;
//  ay = ay - rightayoff;
//  az = az - rightazoff;
//  gx = gx - rightgxoff;
//  gy = gy - rightgyoff;
//  gz = gz - rightgzoff;
//  Serial.print("right,");
//  Serial.print(ax); Serial.print(",");Serial.print(ay); Serial.print(",");Serial.print(az); Serial.print(",");
//  Serial.print(gx); Serial.print(",");Serial.print(gy);Serial.print(",");Serial.println(gz);
//  delay(30);
//  }
}
