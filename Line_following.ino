# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

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
  
  // Set initial direction (HIGH/LOW)
  // for the direction pins.
  
  // Set initial power values for the PWM pins

  analogWrite(L_PWM_PIN, 0);
  analogWrite(L_PWM_PIN, 0);
                                      // Start Serial, wait to connect, print a debug message.
  Serial.begin(9600);
  delay(2000);
  Serial.println("***RESET***");

} // End of setup()

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

      while(digitalRead(ls_pins[i] ) ==HIGH ) {
        //Do nothing here(waiting)
      }

      unsigned long end_time=micros();

      pinMode(EMIT_PIN, INPUT);

      elapsed_time = end_time - start_time;
    }

    return (float)elapsed_time;
}

float weightedMeasurement(){
  float r1, r3;
  r1 = readLineSensor(1);
  r3 = readLineSensor(3);

//  Serial.print(r1);
//  Serial.print(",");
//  Serial.print(r3);
//  Serial.print("\n");
  
  
  float sum = r1 + r3;
//  Serial.print(sum);
//  Serial.print("\n");
//  
  
  r1 = r1 / sum;
  r3 = r3 / sum;
  
//  Serial.print(r1);
//  Serial.print(",");
//  Serial.print(r3);
//  Serial.print("\n");
  
  
  r1 = r1 * 2.0;
  r3 = r3 * 2.0;
//  Serial.print(r1);
//  Serial.print(",");
//  Serial.print(r3);
//  Serial.print("\n");
  float w = r3 - r1;

//  Serial.print("W=");
//  Serial.print(w);
//  Serial.print("\n");
  
  return w;

}
boolean OnLine(){

  if(readLineSensor(1)>=950 || readLineSensor(2)>=1500 || readLineSensor(3)>=950){return true;}
  else{return false;}
  
  } 

void loop() {

  int BiasPWM = 20;
  int MaxTurnPWM = 40;

  for (int j=1;j<4;j++){
      unsigned long results = readLineSensor(j);
      //delay(200);
      Serial.print(results);
      if(j==3){Serial.println("");}
      else{Serial.print(",");}
    } 
  if(OnLine()== true){
//    
    float W = weightedMeasurement();
//    Serial.println(W,6);
    float LeftPWM = BiasPWM +(W*MaxTurnPWM);
    float RightPWM = BiasPWM - (W*MaxTurnPWM);
    setMotorPower(LeftPWM, RightPWM);
    }
  else{
    setMotorPower(0.0,0.0);
    }
//  delay(200);
  }
 // End of loop()
