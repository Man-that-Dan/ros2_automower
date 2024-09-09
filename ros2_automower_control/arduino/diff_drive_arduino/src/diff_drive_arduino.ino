#include <Servo.h>

#include "pid_function.h"
#include "command_types.h"
Servo ESC;  // create servo object to control the ESC
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
#define BAUDRATE 9600
#define PID_RATE 30  // hz
/* Safety stop if commands stop being received */
#define AUTO_STOP_INTERVAL 2000
const int PID_INTERVAL = 1000 / PID_RATE;
const int next_pid_time = PID_INTERVAL;
long last_motor_command_received = AUTO_STOP_INTERVAL;
volatile int hallA = 0;
volatile int hallB = 0;
volatile int hallC = 0;
volatile int hallA_2 = 0;
volatile int hallB_2 = 0;
volatile int hallC_2 = 0;
volatile int wheelState = 0;
volatile int wheelState_2 = 0;
int wheelStates[] = {4, 6, 2, 3, 1, 5, 4};  // possible hall sensor combinations in order;
int wheelStateIndices[] = {0, 4, 2, 3,
                           0, 5, 1};  // indices into wheelStates based on AND-ed hall states
int wheelStateIndex = 0;
int wheelStateIndex_2 = 0;
int clicks = 0;                   // clicks since last output
int clicks_2 = 0;                 // clicks for motor 2
unsigned long currentMillis = 0;  //current timestamp
int comm = 1488;                  //command for motor1
int comm_2 = 1488;                //comand for motor2
bool bounce = false;

struct command
{
  int servoCommand1;  // first servo
  int servoCommand2;  // second servo
};

command input;

char request_type = 's';

struct feedback
{
  int clicks_1;
  int clicks_2;
  unsigned long timestamp_millis;  //note: this overflows at 70 minutes
};
feedback output;

int readLeftEncoder() { return clicks; };
int readRightEncoder() { return clicks_2; };
/* Set the motor speeds accordingly */
void setMotorSpeeds(long left, long right)
{
  ESC.writeMicroseconds(left);
  ESC_2.writeMicroseconds(right);
};

void resetTicks()
{
  clicks = 0;
  clicks_2 = 0;
  leftPID.PrevEnc = 0;
  leftPID.Encoder = 0;
  rightPID.PrevEnc = 0;
  rightPID.Encoder = 0;
}

void setup()
{
  //setup interrupts for motor 1 hall sensors
  attachInterrupt(digitalPinToInterrupt(HALLA_1_PIN), pin_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLB_1_PIN), pin_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLC_1_PIN), pin_ISR, CHANGE);

  //interrupts for motor 2 hall sensors
  attachInterrupt(digitalPinToInterrupt(HALLA_2_PIN), pin_ISR_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLB_2_PIN), pin_ISR_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALLC_2_PIN), pin_ISR_2, CHANGE);

  Serial.begin(9600);
  ESC.attach(ESC1, 1000, 2000);    // (pin, min pulse width, max pulse width in microseconds)
  ESC_2.attach(ESC2, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
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

void loop()
{
  if (Serial.available() > 0) {
    //read command
    Serial.readBytes((byte *)&request_type, sizeof(request_type));
    switch (request_type) {
      case command_type::COMMAND:
        last_motor_command_received = millis();

        byte * input_ptr = (byte *)&input;
        Serial.readBytes((byte *)input_ptr, sizeof(command));
        if (input.servoCommand1 == 0 && input.servoCommand2 == 0) {
          resetPID();
          moving = 0;
          setMotorSpeeds(ZERO_VELOCITY_PWM, ZERO_VELOCITY_PWM);
        } else {
          leftPID.TargetTicksPerFrame = input.servoCommand1;
          rightPID.TargetTicksPerFrame = input.servoCommand2;
          moving = 1;
        }
        Serial.write(command_type::COMMAND);
        for (byte i = 0; i < sizeof(command); i++) Serial.write(*input_ptr++);
        Serial.write('\n');
        break;
      case command_type::FEEDBACK:
        output.clicks_1 = leftPID.Encoder;
        output.clicks_2 = rightPID.Encoder;
        output.timestamp_millis = millis();
        Serial.write(command_type::FEEDBACK);
        byte * output_ptr = (byte *)&output;
        for (byte i = 0; i < sizeof(feedback); i++) Serial.write(*output_ptr++);
        Serial.write('\n');
      case command_type::GAINS:
        byte * gains_ptr = (byte *)&pid_values;
        Serial.readBytes((byte *)gains_ptr, sizeof(pid_values));
        Serial.write(command_type::GAINS);
        for (byte i = 0; i < sizeof(pid_values); i++) Serial.write(*gains_ptr++);
        Serial.write('\n');
      case command_type::RESET_TICKS:
        resetTicks();
        Serial.write(command_type::RESET_TICKS);
        Serial.write('\n');
      default:
        Serial.write('?');
        Serial.write('\n');
        break;
    };
    if (millis() > next_pid_time) {
      updatePID();
      next_pid_time += PID_INTERVAL;
    };
    if ((millis() - last_motor_command_received) > AUTO_STOP_INTERVAL) {
      moving = 0;
      resetPID();
      setMotorSpeeds(ZERO_VELOCITY_PWM, ZERO_VELOCITY_PWM);
    };
  }

  void arm(Servo * esc)
  {
    esc->writeMicroseconds(ZERO_VELOCITY);  //zero accel - midpoint
    delay(50);
    esc->writeMicroseconds(ZERO_VELOCITY + 200);
    delay(50);
    esc->writeMicroseconds(ZERO_VELOCITY);
  }

  void pin_ISR()
  {
    hallA = digitalRead(HALLA_1_PIN);
    hallB = digitalRead(HALLB_1_PIN);
    hallC = digitalRead(HALLC_1_PIN);
    hallA = hallA << 2;
    hallB = hallB << 1;
    int finalState = hallA | hallB | hallC;
    int stateIndex = wheelStateIndices[finalState];
    int direction = stateIndex - wheelStateIndex;
    int next = (wheelStateIndex + 1) % 7;
    if (stateIndex == next) {
      clicks++;
    } else {
      clicks--;
    };
    wheelStateIndex = stateIndex;
    wheelState = finalState;
  }

  void pin_ISR_2()
  {
    hallA_2 = digitalRead(HALLA_2_PIN);
    hallB_2 = digitalRead(HALLB_2_PIN);
    hallC_2 = digitalRead(HALLC_2_PIN);
    hallA_2 = hallA_2 << 2;
    hallB_2 = hallB_2 << 1;
    int finalState = hallA_2 | hallB_2 | hallC_2;
    int stateIndex = wheelStateIndices[finalState];
    int direction = stateIndex - wheelStateIndex;
    int next = (wheelStateIndex_2 + 1) % 7;
    if (stateIndex == next) {
      clicks_2++;
    } else {
      clicks_2--;
    };
    //clicks = clicks + direction;
    wheelStateIndex_2 = stateIndex;
    wheelState_2 = finalState;
  }
