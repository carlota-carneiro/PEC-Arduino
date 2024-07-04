#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Define states
enum State { 
  START,
  SYSTEM_AJUSTE_YAW_PITCH,
  YAW_DIRECTION_CHECK,
  PITCH_DIRECTION_CHECK,
  YAW_DIRECTION_BREAK_CHECK,
  PITCH_DIRECTION_BREAK_CHECK
};

enum MagnetState {
  OFF,
  ON
};

// Current state
State currentState = START;
State previousState = START; // To track state changes
volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
const int ENCODER_RESOLUTION = 1200; // Change this to your encoder's resolution

// Pin configurations
#define ANEMOMETER_SPEED_PIN A0
#define ANEMOMETER_DIRECTION_PIN A1
#define YAW_ENCODER_ENCA 2
#define YAW_ENCODER_ENCB 3
#define PITCH_ENCODER_ENCA 4
#define PITCH_ENCODER_ENCB 5
#define IN_YAW_A 9
#define IN_YAW_B 8
#define IN_PITCH_A 11
#define IN_PITCH_B 12
#define MOTOR_YAW_PIN 7  // Make sure this is a PWM pin
#define MOTOR_PITCH_PIN 13  // Make sure this is a PWM pin
#define MAGNET_PIN_A 22
#define MAGNET_PIN_B 24

// Motor speed definitions
#define LOW_SPEED 80  // Assuming 0-255 for analogWrite
#define MEDIUM_SPEED 160

#define ALLOWANCE_THRESHOLD 30 // Allows for º Error

// Placeholder values for anemometer readings
int anemometerSpeedValues[10] = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
int anemometerDirectionValues[10] = {20, 45, 90, 135, 180, 225, 270, 315, 360, 45};
int loopCounter = 0;
int currentIndex = 0;

//checks if a value is within the target ± the allowance (30 degrees in this case). It also handles edge cases for wrap-around at 0 and 360 degrees.
#define WITHIN_ALLOWANCE(value, target) (((value) >= (target) - (ALLOWANCE_THRESHOLD) && (value) <= (target) + (ALLOWANCE_THRESHOLD)) || ((target) - (ALLOWANCE_THRESHOLD) < 0 && (value) >= 360 + (target) - (ALLOWANCE_THRESHOLD)) || ((target) + (ALLOWANCE_THRESHOLD) > 360 && (value) <= (target) + (ALLOWANCE_THRESHOLD) - 360))

float readAnemometerSpeed() {
  // Return a value from the list iteratively
  return anemometerSpeedValues[currentIndex];
}

int readAnemometerDirection() {
  // Return a value from the list iteratively
  return anemometerDirectionValues[currentIndex];
}

int getYawEncoder() {
  int pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // Convert position to degrees
  float degrees = (pos % ENCODER_RESOLUTION) * 360.0 / ENCODER_RESOLUTION;
  if (degrees < 0) {
    degrees += 360.0;
  }
  return degrees;
}

int getPitchEncoder() {
  // Placeholder: read analog value and convert to direction
  return readAnemometerDirection() + 5;  // For testing purposes, return always true value.
}

int getPitchBreakEncoder() {
  return readAnemometerDirection() + 20;  // For testing purposes, return always true value.
}

void setMotorYawSpeed(int speed) {
  // Set motor speed using PWM
  if (speed > 0) {
    digitalWrite(IN_YAW_A, HIGH);
    digitalWrite(IN_YAW_B, LOW);
    analogWrite(MOTOR_YAW_PIN, speed);
  } else {
    digitalWrite(IN_YAW_A, LOW);
    digitalWrite(IN_YAW_B, LOW);
    analogWrite(MOTOR_YAW_PIN, 0);
  }
}

void setMotorPitchSpeed(int speed) {
  // Set motor speed using PWM
  if (speed > 0) {
    digitalWrite(IN_PITCH_A, HIGH);
    digitalWrite(IN_PITCH_B, LOW);
    analogWrite(MOTOR_PITCH_PIN, speed);
  } else {
    digitalWrite(IN_PITCH_A, LOW);
    digitalWrite(IN_PITCH_B, LOW);
    analogWrite(MOTOR_PITCH_PIN, 0);
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Initialize pin modes
  pinMode(ANEMOMETER_SPEED_PIN, INPUT);
  pinMode(ANEMOMETER_DIRECTION_PIN, INPUT);
  pinMode(YAW_ENCODER_ENCA, INPUT);
  pinMode(YAW_ENCODER_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(YAW_ENCODER_ENCA), readYawEncoder, RISING);
  pinMode(PITCH_ENCODER_ENCA, INPUT);
  pinMode(PITCH_ENCODER_ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(PITCH_ENCODER_ENCA), readPitchEncoder, RISING);
  pinMode(MOTOR_YAW_PIN, OUTPUT);
  pinMode(MOTOR_PITCH_PIN, OUTPUT);
  pinMode(IN_YAW_A, OUTPUT);
  pinMode(IN_YAW_B, OUTPUT);
  pinMode(IN_PITCH_A, OUTPUT);
  pinMode(IN_PITCH_B, OUTPUT);
  pinMode(MAGNET_PIN_A, OUTPUT);
  pinMode(MAGNET_PIN_B, OUTPUT);

  Serial.println("Setup complete");
}

void printStateTransition(State oldState, State newState) {
  Serial.print("State Transition: ");
  switch (oldState) {
    case START:
      Serial.print("START");
      break;
    case SYSTEM_AJUSTE_YAW_PITCH:
      Serial.print("SYSTEM_AJUSTE_YAW_PITCH");
      break;
    case YAW_DIRECTION_CHECK:
      Serial.print("YAW_DIRECTION_CHECK");
      break;
    case PITCH_DIRECTION_CHECK:
      Serial.print("PITCH_DIRECTION_CHECK");
      break;
    case YAW_DIRECTION_BREAK_CHECK:
      Serial.print("YAW_DIRECTION_BREAK_CHECK");
      break;
    case PITCH_DIRECTION_BREAK_CHECK:
      Serial.print("PITCH_DIRECTION_BREAK_CHECK");
      break;
  }
  Serial.print(" -> ");
  switch (newState) {
    case START:
      Serial.println("START");
      break;
    case SYSTEM_AJUSTE_YAW_PITCH:
      Serial.println("SYSTEM_AJUSTE_YAW_PITCH");
      break;
    case YAW_DIRECTION_CHECK:
      Serial.println("YAW_DIRECTION_CHECK");
      break;
    case PITCH_DIRECTION_CHECK:
      Serial.println("PITCH_DIRECTION_CHECK");
      break;
    case YAW_DIRECTION_BREAK_CHECK:
      Serial.println("YAW_DIRECTION_BREAK_CHECK");
      break;
    case PITCH_DIRECTION_BREAK_CHECK:
      Serial.println("PITCH_DIRECTION_BREAK_CHECK");
      break;
  }
}

void printGeneralConditions(float windSpeed, int windDirection, int yawDirection, int pitchDirection) {
  String stateString;

    switch (currentState) {
      case START:
        stateString = "START";
        break;
      case SYSTEM_AJUSTE_YAW_PITCH:
        stateString = "SYSTEM_AJUSTE_YAW_PITCH";
        break;
      case YAW_DIRECTION_CHECK:
        stateString = "YAW_DIRECTION_CHECK";
        break;
      case PITCH_DIRECTION_CHECK:
        stateString = "PITCH_DIRECTION_CHECK";
        break;
      case YAW_DIRECTION_BREAK_CHECK:
        stateString = "YAW_DIRECTION_BREAK_CHECK";
        break;
      case PITCH_DIRECTION_BREAK_CHECK:
        stateString = "PITCH_DIRECTION_BREAK_CHECK";
        break;
    }

  Serial.println("+-------------------+------------+-------------+-------------+-------------+");
  Serial.println("|      State        | Wind Speed | Wind Dir (°)| Yaw Dir (°) | Pitch Dir (°)|");
  Serial.println("+-------------------+------------+-------------+-------------+-------------+");
  Serial.print("| ");
  Serial.print(stateString);
  Serial.print(" | ");
  Serial.print(windSpeed);
  Serial.print("        | ");
  Serial.print(windDirection);
  Serial.print("          | ");
  Serial.print(yawDirection);
  Serial.print("          | ");
  Serial.print("*");
  Serial.println("          |");
  Serial.println("+-------------------+------------+-------------+-------------+-------------+");
}

void changeMagnetState(MagnetState state){
  // Set motor speed using PWM
  if (state == ON) {
    digitalWrite(MAGNET_PIN_A, HIGH);
    digitalWrite(MAGNET_PIN_B, LOW);
  } else {
    digitalWrite(MAGNET_PIN_A, LOW);
    digitalWrite(MAGNET_PIN_B, LOW);
  }
}

void loop() {
  // Variables to hold sensor readings
  float windSpeed;
  int windDirection, yawDirection, pitchDirection;
  int motorSpeed = LOW_SPEED;
  
  // Iterate through the list every 200 loops
  if (loopCounter >= 200) {
    currentIndex = (currentIndex + 1) % 10;
    loopCounter = 0;

    // Read sensor values
    windSpeed = readAnemometerSpeed();
    windDirection = readAnemometerDirection();
    yawDirection = getYawEncoder();
    pitchDirection = getPitchEncoder();

    // Print general conditions every 100 iterations
    printGeneralConditions(windSpeed, windDirection, yawDirection, pitchDirection);
  } else {
    loopCounter++;
  }

  // Print state change, if status changed.
  if (currentState != previousState) {
    printStateTransition(previousState, currentState);
    previousState = currentState;
  }

  // Activate Magnet (Aux Breaking System)
  if (windSpeed >= 2 && windSpeed < 12) {
    changeMagnetState(OFF);
  }
  else{
    changeMagnetState(ON);
    //We need to increase "Speed" when magnet is turned on, because the magnet draws alot of power. 
    motorSpeed = MEDIUM_SPEED;
  }

  //State Machine
  switch (currentState) {
    case START:
      windSpeed = readAnemometerSpeed();
      if (windSpeed >= 2 && windSpeed < 12) {
        currentState = YAW_DIRECTION_CHECK;
      } else {
        currentState = YAW_DIRECTION_BREAK_CHECK;
      }
      break;

    case YAW_DIRECTION_CHECK:
      windDirection = readAnemometerDirection();
      yawDirection = getYawEncoder();
      if (!WITHIN_ALLOWANCE(yawDirection, windDirection)) {
        setMotorYawSpeed(motorSpeed); // Adjust yaw at low speed
        currentState = YAW_DIRECTION_CHECK;
      } else {
        setMotorYawSpeed(0);
        currentState = PITCH_DIRECTION_CHECK;
      }
      break;

    case YAW_DIRECTION_BREAK_CHECK:
      windDirection = readAnemometerDirection();
      yawDirection = getYawEncoder();
      if (!WITHIN_ALLOWANCE(yawDirection, windDirection)) {
        setMotorYawSpeed(motorSpeed); // Adjust yaw at low speed
        currentState = YAW_DIRECTION_BREAK_CHECK;
      } else {
        setMotorYawSpeed(0);
        currentState = PITCH_DIRECTION_BREAK_CHECK;
      }
      break;

    case PITCH_DIRECTION_CHECK:
      windDirection = readAnemometerDirection();
      pitchDirection = getPitchEncoder();
      if (!WITHIN_ALLOWANCE(pitchDirection, windDirection + 5)) {
        setMotorPitchSpeed(motorSpeed); // Adjust pitch at low speed
        currentState = PITCH_DIRECTION_CHECK;
      } else {
        setMotorPitchSpeed(0);
        currentState = START;
      }
      break;

    case PITCH_DIRECTION_BREAK_CHECK:
      windDirection = readAnemometerDirection();
      pitchDirection = getPitchBreakEncoder();
      if (!WITHIN_ALLOWANCE(pitchDirection, windDirection + 20)) {
        setMotorPitchSpeed(motorSpeed); // Adjust pitch at low speed
        currentState = PITCH_DIRECTION_BREAK_CHECK;
      } else {
        setMotorPitchSpeed(0);
        currentState = START;
      }
      break;

    default:
      currentState = START;
      break;
  }

  // Add delay for readability in serial output
  delay(50);
}

void readYawEncoder() {
  int b = digitalRead(YAW_ENCODER_ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void readPitchEncoder() {
  int b = digitalRead(PITCH_ENCODER_ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
