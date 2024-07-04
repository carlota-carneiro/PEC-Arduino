#include <Arduino.h>

// Define states
enum State { 
  START,
  SYSTEM_AJUSTE_YAW_PITCH,
  YAW_DIRECTION_CHECK,
  PITCH_DIRECTION_CHECK,
  YAW_DIRECTION_BREAK_CHECK,
  PITCH_DIRECTION_BREAK_CHECK
};

// Current state
State currentState = START;
State previousState = START; // To track state changes

// Pin configurations
#define ANEMOMETER_SPEED_PIN A0
#define ANEMOMETER_DIRECTION_PIN A1
#define YAW_ENCODER_PIN 2
#define PITCH_ENCODER_PIN 3
#define IN_YAW_A 9
#define IN_YAW_B 8
#define IN_PITCH_A 11
#define IN_PITCH_B 12
#define MOTOR_YAW_PIN 7  // Make sure this is a PWM pin
#define MOTOR_PITCH_PIN 13  // Make sure this is a PWM pin

// Motor speed definitions
#define LOW_SPEED 50  // Assuming 0-255 for analogWrite

// Placeholder values for anemometer readings
int anemometerSpeedValues[10] = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
int anemometerDirectionValues[10] = {0, 45, 90, 135, 180, 225, 270, 315, 360, 45};
int loopCounter = 0;
int currentIndex = 0;

float readAnemometerSpeed() {
  // Return a value from the list iteratively
  return anemometerSpeedValues[currentIndex];
}

int readAnemometerDirection() {
  // Return a value from the list iteratively
  return anemometerDirectionValues[currentIndex];
}

int readYawEncoder() {
  // Placeholder: read analog value and convert to direction
  return random(0, 360);  // For testing purposes, return a random value
}

int readPitchEncoder() {
  // Placeholder: read analog value and convert to direction
  return readAnemometerDirection() + 5;  // For testing purposes, return always true value.
}

int readPitchBreakEncoder() {
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
  pinMode(YAW_ENCODER_PIN, INPUT);
  pinMode(PITCH_ENCODER_PIN, INPUT);
  pinMode(MOTOR_YAW_PIN, OUTPUT);
  pinMode(MOTOR_PITCH_PIN, OUTPUT);
  pinMode(IN_YAW_A, OUTPUT);
  pinMode(IN_YAW_B, OUTPUT);
  pinMode(IN_PITCH_A, OUTPUT);
  pinMode(IN_PITCH_B, OUTPUT);

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

void printGeneralConditions(String state, float windSpeed, int windDirection, int yawDirection, int pitchDirection) {
  Serial.println("+-------------------+------------+-------------+-------------+-------------+");
  Serial.println("|      State        | Wind Speed | Wind Dir (°)| Yaw Dir (°) | Pitch Dir (°)|");
  Serial.println("+-------------------+------------+-------------+-------------+-------------+");
  Serial.print("| ");
  Serial.print(state);
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

void loop() {
  // Variables to hold sensor readings
  float windSpeed;
  int windDirection, yawDirection, pitchDirection;
  
  // Iterate through the list every 100 loops
  if (loopCounter >= 100) {
    currentIndex = (currentIndex + 1) % 10;
    loopCounter = 0;

    // Read sensor values
    windSpeed = readAnemometerSpeed();
    windDirection = readAnemometerDirection();
    yawDirection = readYawEncoder();
    pitchDirection = readPitchEncoder();

    // Print general conditions every 100 iterations
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
    printGeneralConditions(stateString, windSpeed, windDirection, yawDirection, pitchDirection);
  } else {
    loopCounter++;
  }

  // Print state change
  if (currentState != previousState) {
    printStateTransition(previousState, currentState);
    previousState = currentState;
  }

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
      yawDirection = readYawEncoder();
      if (yawDirection != windDirection) {
        setMotorYawSpeed(LOW_SPEED); // Adjust yaw at low speed
        currentState = YAW_DIRECTION_CHECK;
      } else {
        setMotorYawSpeed(0);
        currentState = PITCH_DIRECTION_CHECK;
      }
      break;

    case YAW_DIRECTION_BREAK_CHECK:
      windDirection = readAnemometerDirection();
      yawDirection = readYawEncoder();
      if (yawDirection != windDirection) {
        setMotorYawSpeed(LOW_SPEED); // Adjust yaw at low speed
        currentState = YAW_DIRECTION_BREAK_CHECK;
      } else {
        setMotorYawSpeed(0);
        currentState = PITCH_DIRECTION_BREAK_CHECK;
      }
      break;

    case PITCH_DIRECTION_CHECK:
      windDirection = readAnemometerDirection();
      pitchDirection = readPitchEncoder();
      if (pitchDirection != (windDirection + 5)) {
        setMotorPitchSpeed(LOW_SPEED); // Adjust pitch at low speed
        currentState = PITCH_DIRECTION_CHECK;
      } else {
        setMotorPitchSpeed(0);
        currentState = START;
      }
      break;

    case PITCH_DIRECTION_BREAK_CHECK:
      windDirection = readAnemometerDirection();
      pitchDirection = readPitchBreakEncoder();
      if (pitchDirection != (windDirection + 20)) {
        setMotorPitchSpeed(LOW_SPEED); // Adjust pitch at low speed
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
  delay(100);
}
