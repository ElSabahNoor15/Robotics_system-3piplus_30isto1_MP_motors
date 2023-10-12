# define EMIT_PIN 11    // Documentation says 11.
# define LS_MIDDLE_PIN 20    // Complete for DN1 pin


# define MAX_SAMPLES 10
# define LS_LEFT_PIN 20   // Complete for DN3 pin

# define MAX_SAMPLES 10
float results[ MAX_SAMPLES ]; // An array of MAX_SAMPLES length

void setup() {

  // Set some initial pin modes and states
  pinMode( EMIT_PIN, INPUT ); // Set EMIT as an input (off)
  pinMode( LS_LEFT_PIN, INPUT );     // Set line sensor pin to input

  // Start Serial, wait to connect, print a debug message.
  Serial.begin(9600);
  delay(1500);
  Serial.println("***RESET***");

} // End of setup()


void loop() {
   
  for(int i=0;i<MAX_SAMPLES;i++){
  pinMode( EMIT_PIN, OUTPUT );
  digitalWrite( EMIT_PIN, HIGH );
  pinMode( LS_LEFT_PIN, OUTPUT );
  digitalWrite( LS_LEFT_PIN, HIGH );
  delayMicroseconds( 10 );
  pinMode( LS_LEFT_PIN, INPUT);

  unsigned long start_time = micros();

  while( digitalRead(LS_LEFT_PIN) == HIGH ) {
      // Do nothing here (waiting).
  }

  unsigned long end_time = micros();

  pinMode( EMIT_PIN, INPUT );

  unsigned long elapsed_time = end_time - start_time;
  
  results[i] = (float)elapsed_time;
  delay(200);

  }
  
  Serial.println("Results: ");
  for( int i = 0; i < MAX_SAMPLES; i++ ) {
        Serial.println( results[i] );
      }
  delay(100);
 }
