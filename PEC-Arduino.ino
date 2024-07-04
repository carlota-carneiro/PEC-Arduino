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

// Pin configurations
#define ANEMOMETER_SPEED_PIN A0
#define ANEMOMETER_DIRECTION_PIN A1
#define YAW_ENCODER_PIN A2
#define PITCH_ENCODER_PIN A3
#define IN_YAW_A 3
#define IN_YAW_B 4
#define IN_PITCH_A 3
#define IN_PITCH_B 4
#define MOTOR_YAW_PIN 2
#define MOTOR_PITCH_PIN 2

// Motor speed definitions
#define LOW_SPEED 128  // Assuming 0-255 for analogWrite

float readAnemometerSpeed() {
  // Placeholder: read analog value and convert to wind speed
  //int sensorValue = analogRead(ANEMOMETER_SPEED_PIN);
  //return sensorValue * (5.0 / 1023.0); // Example conversion
  return 5;
}

int readAnemometerDirection() {
  // Placeholder: read analog value and convert to direction
  //int sensorValue = analogRead(ANEMOMETER_DIRECTION_PIN);
  //return sensorValue * (360.0 / 1023.0); // Example conversion
  return 5;
}

int readYawEncoder() {
  // Placeholder: read analog value and convert to direction
  //int sensorValue = analogRead(YAW_ENCODER_PIN);
  //return sensorValue * (360.0 / 1023.0); // Example conversion
  return 5;
}

int readPitchEncoder() {
  // Placeholder: read analog value and convert to direction
  //int sensorValue = analogRead(PITCH_ENCODER_PIN);
  //return sensorValue * (360.0 / 1023.0) + 5; // Example conversion
  return 10;
}

void setMotorYawSpeed(int speed) {
  // Set motor speed using PWM
  digitalWrite(IN_YAW_A, HIGH);
  digitalWrite(IN_YAW_B, LOW);
  analogWrite(MOTOR_YAW_PIN, speed);
}

void setMotorPitchSpeed(int speed) {
  // Set motor speed using PWM
  digitalWrite(IN_PITCH_A, HIGH);
  digitalWrite(IN_PITCH_B, LOW);
  analogWrite(MOTOR_PITCH_PIN, speed);
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
}

void loop() {
  // Variables to hold sensor readings
  float windSpeed;
  int windDirection, yawDirection, pitchDirection;
  
  switch (currentState) {
    case START:
      Serial.println("State: START");
      windSpeed = readAnemometerSpeed();
      if (windSpeed >= 2 && windSpeed < 12) {
        currentState = YAW_DIRECTION_CHECK;
      } else {
        currentState = YAW_DIRECTION_BREAK_CHECK;
      }
      break;

    case YAW_DIRECTION_CHECK:
      Serial.println("State: YAW_DIRECTION_CHECK");
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
      Serial.println("State: YAW_DIRECTION_BREAK_CHECK");
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
      Serial.println("State: PITCH_DIRECTION_CHECK");
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
      Serial.println("State: PITCH_DIRECTION_BREAK_CHECK");
      windDirection = readAnemometerDirection();
      pitchDirection = readPitchEncoder();
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
  delay(1000);
}

