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
  pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
  pinMode( LS_LEFT_PIN, INPUT );     // Set line sensor pin to input
  pinMode( LS_MIDLEFT_PIN, INPUT );
  pinMode( LS_MIDDLE_PIN, INPUT );
  pinMode( LS_MIDRIGHT_PIN, INPUT );
  pinMode( LS_RIGHT_PIN, INPUT );
  // Start Serial, wait to connect, print a debug message.
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");

} // End of setup()

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

void loop() {

  // Collect MAX_SAMPLES readings.
  //for( int i = 0; i < MAX_SAMPLES; i++ ) {              //Where there are 5 sensors
    for (int j=0;j<5;j++){
      unsigned long results = readLineSensor(j);
      //delay(200);
      Serial.print(results);
      if(j==4){Serial.println("");}
      else{Serial.print(",");}
    }    
    //delay(100);
  //}
  delay(1000);

} // End of loop()
