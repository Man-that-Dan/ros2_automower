#include <Servo.h>

Servo ESC;     // create servo object to control the ESC
Servo ESC_2;
#define ESC1 9
#define ESC2 10
#define HALLA_1_PIN 2
#define HALLB_1_PIN 3
#define HALLC_1_PIN 18
#define HALLA_2_PIN 19
#define HALLB_2_PIN 20
#define HALLC_2_PIN 21
#define ZERO_VELOCITY 1488
#define OUTPUT_INTERVAL_MS 500

volatile int hallA = 0;
volatile int hallB = 0;
volatile int hallC = 0;
volatile int hallA_2 = 0;
volatile int hallB_2 = 0;
volatile int hallC_2 = 0;
volatile int wheelState = 0;
volatile int wheelState_2 = 0;
int wheelStates[] = {4, 6, 2, 3, 1, 5, 4}; // possible hall sensor combinations in order;
int wheelStateIndices[] = {0, 4, 2, 3, 0, 5, 1}; // indices into wheelStates based on AND-ed hall states
int wheelStateIndex = 0;
int wheelStateIndex_2 = 0;
int clicks = 0; // clicks since last output
int clicks_2 = 0; // clicks for motor 2
unsigned long currentMillis = 0; //current timestamp
int comm = 1488; //command for motor1
int comm_2 = 1488; //comand for motor2
bool bounce = false;

enum command_type
{
  COMMAND = 'c',
  FEEDBACK = 'e',
  GAINS = 'p'
};

struct command{
  int servoCommand1; // first servo
  int servoCommand2; // second servo 
};

command input;
pid_values gains;

char request_type = 's';

struct feedback{
  int clicks_1;
  int clicks_2;
  unsigned long timestamp_millis; //note: this overflows at 70 minutes
};
feedback output;

void setup() {
  //setup interrupts for motor 1 hall sensors
  attachInterrupt(digitalPinToInterrupt(HALLA_1_PIN), pin_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLB_1_PIN), pin_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLC_1_PIN), pin_ISR, CHANGE);

  //interrupts for motor 2 hall sensors
  attachInterrupt(digitalPinToInterrupt(HALLA_2_PIN), pin_ISR_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLB_2_PIN), pin_ISR_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLC_2_PIN), pin_ISR_2, CHANGE);
  
  Serial.begin(9600);
  ESC.attach(ESC1,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC_2.attach(ESC2,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  arm(&ESC);
  arm(&ESC_2);

  //initial position 1
  hallA = digitalRead(HALLA_1_PIN);
  hallB = digitalRead(HALLB_1_PIN);
  hallC = digitalRead(HALLC_1_PIN);
  hallA = hallA << 2;
  hallB = hallB << 1;
  wheelState = hallA | hallB | hallC;
  wheelStateIndex = wheelStateIndices[wheelState];

  hallA_2 = digitalRead(HALLA_2_PIN);
  hallB_2 = digitalRead(HALLB_2_PIN);
  hallC_2 = digitalRead(HALLC_2_PIN);
  hallA_2 = hallA_2 << 2;
  hallB_2 = hallB_2 << 1;
  wheelState_2 = hallA_2 | hallB_2 | hallC_2;
  wheelStateIndex_2 = wheelStateIndices[wheelState_2];
}

void loop() {
  if(millis() - currentMillis >= OUTPUT_INTERVAL_MS){
    //send the current feedback
    output.clicks_1 = clicks;
    output.clicks_2 = clicks_2;
    output.timestamp_millis = millis();
    byte* output_ptr = (byte*) &output;
    for (byte i = 0; i < sizeof(feedback); i++) Serial.write(*output_ptr++);
    //Serial.print("\n");
    currentMillis = millis();
    clicks = 0;
    clicks_2 = 0;
  };
  if(Serial.available() > 0){
    //read command
    Serial.readBytes((byte*) &request_type, sizeof(request_type));
    switch (request_type)
    {
    case command_type::COMMAND:
      byte* input_ptr = (byte*) &input;
      Serial.readBytes((byte*)input_ptr, sizeof(command));
      //TODO DO PID HERE ESC.writeMicroseconds(input.servoCommand1);    // Send the signal to the ESC
      ESC_2.writeMicroseconds(input.servoCommand2);    // Send the signal to the ESC
      break;
    
    default:
      break;
    }
    if(!bounce){
      byte* input_ptr = (byte*) &input;
      Serial.readBytes((byte*)input_ptr, sizeof(command));
      Serial.write(input.type);
      Serial.write(0);
      Serial.write(0);
      Serial.println(String("sGot the poop command: ") + String(input.servoCommand1) + String(" and ") + String(input.servoCommand2));
      bounce = true;
    } else {
      //Serial.parseInt();
      bounce = false;
    }
    //acknowledge
    byte* input_ptr = (byte*) &input;
    for (byte i = 0; i < sizeof(command); i++) Serial.write(*input_ptr++);
    Serial.println(String("sGot the command: ") + String(input.servoCommand1) + String(" and ") + String(input.servoCommand2));
    input_ptr = (byte*) &input;
    for (byte i = 0; i < sizeof(command); i++) Serial.write(*input_ptr++);
    Serial.println(String("sGot the command: ") + String(input.servoCommand1) + String(" and ") + String(input.servoCommand2));
    input_ptr = (byte*) &input;
    for (byte i = 0; i < sizeof(command); i++) Serial.write(*input_ptr++);
    Serial.println(String("sGot the command: ") + String(input.servoCommand1) + String(" and ") + String(input.servoCommand2));
    input_ptr = (byte*) &input;
    for (byte i = 0; i < sizeof(command); i++) Serial.write(*input_ptr++);
    Serial.println(String("sGot the command: ") + String(input.servoCommand1) + String(" and ") + String(input.servoCommand2));
  };
  ESC.writeMicroseconds(input.servoCommand1);    // Send the signal to the ESC
  ESC_2.writeMicroseconds(input.servoCommand2);    // Send the signal to the ESC
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

void pin_ISR_2() {
  hallA_2 = digitalRead(HALLA_2_PIN);
  hallB_2 = digitalRead(HALLB_2_PIN);
  hallC_2 = digitalRead(HALLC_2_PIN);
  hallA_2 = hallA_2 << 2;
  hallB_2 = hallB_2 << 1;
  int finalState = hallA_2 | hallB_2 | hallC_2;
  int stateIndex = wheelStateIndices[finalState];
  int direction = stateIndex - wheelStateIndex;
  int next = (wheelStateIndex_2 + 1) % 7;
  if(stateIndex == next){
    clicks_2++;
  } else {
    clicks_2--;
  };
  //clicks = clicks + direction;
  wheelStateIndex_2 = stateIndex;
  wheelState_2 = finalState;

}
