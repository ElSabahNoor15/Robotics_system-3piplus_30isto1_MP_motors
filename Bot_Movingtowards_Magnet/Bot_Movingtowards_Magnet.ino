#include <Wire.h>
#include <LIS3MDL.h>
#include "encoders.h"

# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

# define FWD LOW
# define REV HIGH

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 32;
const int WHEEL_CIRCUMFERENCE = 100.48;
const float DISTANCE_BET_WHEELS = 90.0;

LIS3MDL mag;


void setup() {
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);

  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);

  setupEncoder0();
  setupEncoder1();
  Wire.begin();
  Serial.begin(9600);
  Serial.println("***RESET***");
  delay(1000);

  if (!mag.init()) {
    while (1) {
      Serial.println("Failed to detect and initialize magnetometer!");
      delay(1000);
    }
  }

  mag.enableDefault();
}

// Exponential Moving Average parameters
float alpha = 0.8;  // Adjust this value based on the desired smoothing effect
float previousX = 0.0;
float previousY = 0.0;

void setMotorPower(float left_pwm, float right_pwm) {
  if (left_pwm > 0) {
    digitalWrite(L_DIR_PIN, FWD);
    analogWrite(L_PWM_PIN, left_pwm);
  } else {
    analogWrite(L_PWM_PIN, -left_pwm);
    digitalWrite(L_DIR_PIN, REV);
  }

  if (right_pwm > 0) {
    digitalWrite(R_DIR_PIN, FWD);
    analogWrite(R_PWM_PIN, right_pwm);
  } else {
    digitalWrite(R_DIR_PIN, REV);
    analogWrite(R_PWM_PIN, -right_pwm);
  }
}

float calibrateAndScale(float rawReading, float offset, float range, float scale) {
  return scale * (rawReading - offset);
}

// Function to apply Exponential Moving Average filter
float exponentialMovingAverage(float currentReading, float previousValue, float alpha) {
  return (alpha * currentReading) + ((1 - alpha) * previousValue);
}


void rotateRightBot(){
  
  count_e0 = 0.00;
  count_e1 = 0.00;
  
  // count needed for 360 is 1012 encoder count
  // therefore 2.36 counts for 1 degree
  
  float countsNeeded = 2.361*3;

  while (count_e1 < countsNeeded) {
    setMotorPower(20, -24);
   }
   setMotorPower(0, 0);
  delay(400);
}

void rotateLeftBot(){
  
  count_e0 = 0.00;
  count_e1 = 0.00;
  
  // count needed for 360 is 1012 encoder count
  // therefore 2.36 counts for 1 degree
  
  float countsNeeded = 2.361*3;

  while (count_e0 < countsNeeded) {
    setMotorPower(-19, 23);
   }
   setMotorPower(0, 0);
  delay(500);
}
  
void loop() {
  
  float minX = 0, minY = 0;
  float maxX = 0, maxY = 0;
  
  mag.read();

  float x = mag.m.x;
  float y = mag.m.y;

  minX = min(minX, x);
  minY = min(minY, y);
  maxX = max(maxX, x);
  maxY = max(maxY, y);

  float rangeX = maxX - minX;
  float rangeY = maxY - minY;
  
  float offsetX = minX + (rangeX/2.0);
  float offsetY = minY + (rangeY/2.0);

  float avgRange = (rangeX + rangeY) / 2.0;
  float scaleX = avgRange / rangeX;
  float scaleY = avgRange / rangeY;

  float calibratedX = calibrateAndScale(x, offsetX, rangeX, scaleX);
  float calibratedY = calibrateAndScale(y, offsetY, rangeY, scaleY);

  float calibrated_heading = atan2(calibratedY,calibratedX);

  float filteredX = exponentialMovingAverage(calibratedX, previousX, alpha);
  float filteredY = exponentialMovingAverage(calibratedY, previousY, alpha);
    
  previousX = filteredX;
  previousY = filteredY;

  Serial.print("X: ");
  Serial.print(filteredX);
  Serial.print(", Y: ");
  Serial.println(filteredY);
  if(filteredX > -3000 or filteredX < -5000 and abs(filteredY) > 5000){
    setMotorPower(0, 0);
    delay(3000);
    }
  else{
    rotateRightBot();
    }
  delay(200);
}
