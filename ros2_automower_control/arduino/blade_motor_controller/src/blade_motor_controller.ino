#include <Servo.h>

Servo ESC;     // create servo object to control the ESC
#define ESC1 10
#define HALLA_1_PIN 1
#define HALLB_1_PIN 2
#define HALLC_1_PIN 3
#define ZERO_VELOCITY 1488
#define OUTPUT_INTERVAL_MS 500

volatile int hallA = 0;
volatile int hallB = 0;
volatile int hallC = 0;
volatile int wheelState = 0;
int wheelStates[] = {4, 6, 2, 3, 1, 5, 4}; // possible hall sensor combinations in order;
int wheelStateIndices[] = {0, 4, 2, 3, 0, 5, 1}; // indices into wheelStates based on AND-ed hall states
int wheelStateIndex = 0;
int clicks = 0; // clicks since last output
unsigned long currentMillis = 0; //current timestamp
int comm = 1488; //command for motor1
bool bounce = false;

struct command{
  char type = 'c';
  int servoCommand1; // first servo
};
command input;

struct feedback{
  char type = 'f';
  int clicks_1;
  unsigned long timestamp_millis; //note: this overflows at 70 minutes
};
feedback output;

void setup() {
  //setup interrupts for motor 1 hall sensors
  attachInterrupt(digitalPinToInterrupt(HALLA_1_PIN), pin_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLB_1_PIN), pin_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLC_1_PIN), pin_ISR, CHANGE);
  
  
  Serial.begin(9600);
  delay(10000);
  Serial.println("sstarting up..");
  ESC.attach(ESC1,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC.writeMicroseconds(1488);
  delay(500);
  ESC.writeMicroseconds(1820);
  delay(1500);
  arm(&ESC);
//  ESC.writeMicroseconds(1300);
//  delay(10000);
//  ESC.writeMicroseconds(2000);
//  delay(10000);
//  ESC.writeMicroseconds(1400);
//  delay(5000);


  Serial.print("sArmed\n");

  //initial position 1
  hallA = digitalRead(HALLA_1_PIN);
  hallB = digitalRead(HALLB_1_PIN);
  hallC = digitalRead(HALLC_1_PIN);
  hallA = hallA << 2;
  hallB = hallB << 1;
  wheelState = hallA | hallB | hallC;
  wheelStateIndex = wheelStateIndices[wheelState];

}

void loop() {
  
  if(millis() - currentMillis >= 500){
    //send the current feedback
    output.clicks_1 = clicks;
    output.timestamp_millis = millis();
    byte* output_ptr = (byte*) &output;
    for (byte i = 0; i < sizeof(feedback); i++) Serial.write(*output_ptr++);
    
    currentMillis = millis();
    String left = String(clicks);
    clicks = 0;
  };
  if(Serial.available() > 0){
    //read command
    byte* input_ptr = (byte*) &input;
    Serial.readBytes((uint8_t*)input_ptr, sizeof(command));

    //acknowledge
    input_ptr = (byte*) &input;
    for (byte i = 0; i < sizeof(command); i++) Serial.write(*input_ptr++);
    
  };
  ESC.writeMicroseconds(input.servoCommand1);    // Send the signal to the ESC

}

void arm(Servo* esc){
  esc->writeMicroseconds(ZERO_VELOCITY); //zero accel - midpoint
  delay(50);
  esc->writeMicroseconds(ZERO_VELOCITY + 200);
  delay(50);
  esc->writeMicroseconds(ZERO_VELOCITY);
}


void pin_ISR() {
  hallA = digitalRead(HALLA_1_PIN);
  hallB = digitalRead(HALLB_1_PIN);
  hallC = digitalRead(HALLC_1_PIN);
  hallA = hallA << 2;
  hallB = hallB << 1;
  int finalState = hallA | hallB | hallC;
  int stateIndex = wheelStateIndices[finalState];
  int direction = stateIndex - wheelStateIndex;
  int next = (wheelStateIndex + 1) % 7;
  if(stateIndex == next){
    clicks++;
  } else {
    clicks--;
  };
  //clicks = clicks + direction;
  wheelStateIndex = stateIndex;
  wheelState = finalState;

}
