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
  delay(1500);
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

void loop() {

//  setMotorPower(17,20); //minimum value with straight
//  setMotorPower(-17,20);  // anti clockwise
  setMotorPower(17,-20);    // clockwise

}
