#include "encoders.h"
# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

// Set initial direction (HIGH/LOW)
// for the direction pins.
# define FWD LOW
# define REV HIGH

# define EMIT_PIN 11           // IR emitter pin
# define LS_LEFT_PIN 12        // Line sensor DN1 pin
# define LS_MIDLEFT_PIN 18     // Line sensor DN2 pin
# define LS_MIDDLE_PIN 20      // Line sensor DN3 pin
# define LS_MIDRIGHT_PIN 21    // Line sensor DN4 pin
# define LS_RIGHT_PIN 22       // Line sensor DN5 pin

# define MAX_SAMPLES 10
unsigned long results[ MAX_SAMPLES ]; // An array of MAX_SAMPLES length
int ls_pins[5] = {LS_LEFT_PIN,
                  LS_MIDLEFT_PIN,
                  LS_MIDDLE_PIN,
                  LS_MIDRIGHT_PIN,
                  LS_RIGHT_PIN };
enum RobotState {
  idle,
  movingForward,
  followingLine,
  turningToOrigin,
  comingToOrigin
};

RobotState currentState = idle;  // Initialize the robot's state

//demographics of the robot
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 29.86F;
const float WHEEL_DIAMETER = 32;
const int WHEEL_CIRCUMFERENCE = 100.48;
const float DISTANCE_BET_WHEELS = 90.0;
//coordinates and angle
float avgDistance;
float currentX = 0.0;
float currentY = 0.0;
float deltaTheta; 

//Function of reading line sensor
float readLineSensor( int i ) {
    unsigned long elapsed_time=0;
    if(i>=0 && i<5){
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH);
      
      pinMode(ls_pins[i], OUTPUT);
      digitalWrite(ls_pins[i], HIGH );
      delayMicroseconds(10);
      pinMode(ls_pins[i], INPUT);

      unsigned long start_time = micros();

      while(digitalRead(ls_pins[i] ) == HIGH ) {
        //Do nothing here(waiting)
      }

      unsigned long end_time=micros();

      pinMode(EMIT_PIN, INPUT);

      elapsed_time = end_time - start_time;
    }

    return (float)elapsed_time;
}

//functions for motor                                
void setMotorPower( float left_pwm, float right_pwm ) {

  if(left_pwm>0){
    digitalWrite(L_DIR_PIN, FWD);
    analogWrite( L_PWM_PIN, left_pwm );
    }
  else{
    analogWrite( L_PWM_PIN, -left_pwm );
    digitalWrite(L_DIR_PIN, REV);
    }
  if(right_pwm>0){
    digitalWrite(R_DIR_PIN, FWD);
    analogWrite( R_PWM_PIN, right_pwm );
    }
   else{
    digitalWrite(R_DIR_PIN, REV);
    analogWrite( R_PWM_PIN, -right_pwm );
    }
}

void rotate(){
  
  setMotorPower(-17,23);
  }

// Function to update the (x, y) coordinates based on encoder data
void updateCoordinates() {
  // Calculate the distance traveled by both wheels
  float leftDistance = ((-count_e1) * WHEEL_CIRCUMFERENCE) / (CLICKS_PER_ROTATION * GEAR_RATIO);
  float rightDistance = ((-count_e0) * WHEEL_CIRCUMFERENCE) / (CLICKS_PER_ROTATION * GEAR_RATIO);

  // Update the robot's orientation (theta)
  deltaTheta = (leftDistance - rightDistance) / DISTANCE_BET_WHEELS;
//  // Calculate the average distance
//  if(leftDistance<0){leftDistance = -leftDistance;}
//  else{return leftDistance;}
//  if(rightDistance<0){rightDistance = -rightDistance;}
//  else{return rightDistance;}
  avgDistance =  leftDistance + rightDistance/ 2.0;
  
  // Update the (x, y) coordinates based on the distance and orientation
  currentX = avgDistance * cos(deltaTheta);
  currentY = avgDistance * sin(deltaTheta);
//  Serial.print("Average Dist: ");
//  Serial.print(avgDistance);
//  Serial.print(" and ");
//  Serial.print(leftDistance);
//  Serial.print(", ");
//  Serial.print(rightDistance);
//  Serial.print("angle: ");
//  Serial.print(deltaTheta);
//  Serial.print(", positions: ( ");
//  Serial.print(currentX);
//  Serial.print(", ");
//  Serial.print(currentY);
//  Serial.print("), Distance travelled: ");
//  Serial.println(avgDistance);
}

//First task to touch the line
void driveForward() {
  // Reset the encoder counts to zero
  float lineTouchingDistance = 220; // millimeters
  count_e0 = 0;
  count_e1 = 0;

  // Calculate the number of encoder counts needed to travel the target distance
  float countsNeeded = (lineTouchingDistance / WHEEL_CIRCUMFERENCE) * CLICKS_PER_ROTATION * GEAR_RATIO;
  //float countsNeeded = 660;
  // Drive forward until the target distance is reached
  while (-count_e0 < countsNeeded || -count_e1 < countsNeeded) {
    setMotorPower(17, 23);
  }

  // Stop the motors
  setMotorPower(0, 0);
  updateCoordinates();
}


//Initialising the bot using Setup
void setup() {

  // Set some initial pin modes and states
  
  pinMode( EMIT_PIN, INPUT );         // Set EMIT as an input (off)
  pinMode( LS_LEFT_PIN, INPUT );      // Set line sensor pin to input
  pinMode( LS_MIDLEFT_PIN, INPUT );
  pinMode( LS_MIDDLE_PIN, INPUT );
  pinMode( LS_MIDRIGHT_PIN, INPUT );
  pinMode( LS_RIGHT_PIN, INPUT );

  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  
  
  // Set initial power values for the PWM pins

  analogWrite(L_PWM_PIN, 0);
  analogWrite(R_PWM_PIN, 0);

  //initiating encoders
  setupEncoder0();
  setupEncoder1();
  // Start Serial, wait to connect, print a debug message.
  Serial.begin(9600);
  delay(2000);
  Serial.println("***RESET***");

  updateCoordinates();

}


//Weight management and Line following

float weightedMeasurement(){
  float r1, r3;
  r1 = readLineSensor(1);
  r3 = readLineSensor(3);
  float sum = r1 + r3;

  r1 = r1 / sum;
  r3 = r3 / sum;
    
  r1 = r1 * 2.0;
  r3 = r3 * 2.0;
  float w = r3 - r1;
  
  return w;
}

boolean OnLine(){
  if(readLineSensor(1)>=1000 || readLineSensor(2)>=1300 || readLineSensor(3)>=1000){return true;}
  else{return false;}
  
}
boolean OnLineButTakeLeft(){

  if(readLineSensor(1)>=1100 || readLineSensor(2)>=1100){return true;}
  else{return false;}
  
}
boolean OnLineButTakeRight(){

  if(readLineSensor(3)>=1100 || readLineSensor(2)>=1101){return true;}
  else{return false;}
  
}

void linefollowing(){
 
  //unsigned long bot_moving_start_time = millis();
  int BiasPWM = 22;
  int MaxTurnPWM = 37;
  int leftMinPWM = 17;
  int rightMinPWM = 23;

  
  if(OnLine()== true){
 
    float W = weightedMeasurement();
    float LeftPWM = BiasPWM +(W*MaxTurnPWM);
    float RightPWM = BiasPWM - (W*MaxTurnPWM);
    setMotorPower(LeftPWM, RightPWM);
    updateCoordinates();
    }
  else if(OnLineButTakeLeft() == true){
    setMotorPower(-leftMinPWM,(rightMinPWM));
    updateCoordinates();
    }   //anti-clockwise
  else if(OnLineButTakeRight() == true){
    setMotorPower(leftMinPWM,-(rightMinPWM));
    updateCoordinates();
    }  //clockwise
  else if(readLineSensor(2)<720 && avgDistance>2000){
    setMotorPower(0,0);    
    updateCoordinates();
    delay(5000);   
    return;
    }
  else{
    rotate();
    }
}


//Rotating toward the origin
void turnToOrigin() {
  // Calculate the angle (theta) to point the robot toward the origin
  updateCoordinates();
  float targetTheta = atan2(currentY, currentX);
  Serial.print("Target angle: ");
  Serial.println(targetTheta);
  // Calculate the change in angle needed
  //float deltaTheta = (targetTheta - currentTheta);

  // Set a rotation speed (you can adjust this)
  //float rotationSpeed = 50; // Adjust as needed

  // Determine the direction of the turn (clockwise or counterclockwise)
  int turnDirection = (targetTheta > 0) ? 1 : -1;

  // Perform the turn
  if(targetTheta>0){
    while (targetTheta > 0.05) { // Adjust the threshold for precision
    setMotorPower(17,-23);
    updateCoordinates(); // Update orientation based on encoder data
    targetTheta -= targetTheta;
    }
    Serial.print(", final angle left: ");
    Serial.println(deltaTheta);
    }
  else{
    while (targetTheta < -0.05) { // Adjust the threshold for precision
  
    setMotorPower(-17,23);
    updateCoordinates(); // Update orientation based on encoder data
    targetTheta -= targetTheta;
    }
  
    Serial.print(", final angle left: ");
    Serial.println(deltaTheta);
  }

  // Stop the motors
  setMotorPower(0, 0); 
}

//Last Task of the Robot
void backToOrigin(){
  float targetDistance = sqrt(currentX * currentX + currentY * currentY); // millimeters
  count_e0 = 0;
  count_e1 = 0;

  // Calculate the number of encoder counts needed to travel the target distance
  float countsNeeded = (targetDistance / WHEEL_CIRCUMFERENCE) * CLICKS_PER_ROTATION * GEAR_RATIO;
  
  // Drive forward until the target distance is reached
  while (-count_e0 < countsNeeded || -count_e1 < countsNeeded) {
    setMotorPower(17, 23);
  }

  // Stop the motors
  setMotorPower(0, 0);
  //updateCoordinates();
}


//Section includes working of the robot using different states
void loop(){
  
  switch (currentState) {
    case idle:
      updateCoordinates();
      currentState = movingForward;
      break;

    case movingForward:
      driveForward();
      updateCoordinates();
      currentState = followingLine;
      break;

    case followingLine:
      linefollowing();
      // Assuming the robot needs to return to its origin (0,0) after covering a certain distance
      if(readLineSensor(2)<720 && avgDistance>2000){
        currentState = turningToOrigin;
        }
      break;

//    case turningToOrigin:
//
//      turnToOrigin();
//      if(readLineSensor(2)<730 && avgDistance>1550){
//        currentState = comingToOrigin;
//        }
//          
//      break;
//    case comingToOrigin:
//
//      backToOrigin();
//      delay(2000);
//      break;
//    // Stop the motors after reaching the origin
  
  while(1){setMotorPower(0, 0);}
  }
  
}

//void loop(){
//
//  //updateCoordinates();
//  turnToOrigin();
//  }
