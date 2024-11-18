#include <Wire.h>
#include <MPU6050.h>
#include <Arduino.h>

// Motor A pins
int pwmA = 3;
int in1A = 4;
int in2A = 5;

// Motor B pins
int pwmB = 9;
int in1B = 8;
int in2B = 7;

// Standb pin
int stbyPin = 6;

// Bluetooth LED pin
int ledPin = 13; // Connect an LED to pin 13

// PID constants
double Kp = 25.0; // Reduced Proportional gain
double Ki = 1.5; // Reduced Integral gain
double Kd = 3.5;

// Setpoint and PID variables
double Setpoint = 0.0;
double Input, Output;
double last_error = 0.0;
double I = 0.0;

// MPU6050 variables
MPU6050 mpu;
double angleX = 0.0;
double dt = 0.01;
double alpha = 0.95;

// Variables to hold motor speed and direction
int motorSpeed = 0;
const int maxAngle = 45;
const int maxMotorSpeed = 150;

int MPUOffsets[6] = {-1920, -332, 1680, 90, 22, -22};

void setup() {
  Serial.begin(9600);  // For debugging via USB
  Serial1.begin(1200); // Initialize Serial1 for Bluetooth communication
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  pinMode(pwmA, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);
  pinMode(stbyPin, OUTPUT);

  pinMode(ledPin, OUTPUT); // Set LED pin as output
  digitalWrite(ledPin, LOW); // Start with LED off
  
  digitalWrite(stbyPin, HIGH); // Disable standby mode
}

void loop() {
  // Bluetooth control for LED
  if (Serial1.available() > 0) {
    char command = Serial1.read(); // Read command from Bluetooth
    if (command == '1') {
      digitalWrite(ledPin, HIGH); // Turn LED on
    } else if (command == '0') {
      digitalWrite(ledPin, LOW); // Turn LED off
    }
  }

  // Variables to hold sensor data
  int16_t ax, ay, az, gx, gy, gz;
  
  // Read accelerometer and gyroscope data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert gyroscope raw data to degrees/s
  double gyroX = gx / 131.0;

  // Calculate the angle from the accelerometer
  double accelAngleX = atan2(ay, az) * 180 / PI;

  // Integrate the gyroscope data to get the angle
  angleX += gyroX * dt;

  // Apply the complementary filter
  angleX = alpha * angleX + (1 - alpha) * accelAngleX;

  // Use the estimated angle as Input for PID control
  Input = angleX;

  // PID calculations
  double error = Setpoint - Input;
  double P = Kp * error;
  I += Ki * error;

  // Implement anti-windup for integral term
  I = constrain(I, -maxMotorSpeed, maxMotorSpeed);

  double D = Kd * (error - last_error);
  Output = P + I + D;

  motorSpeed = constrain(abs(Output), 0, maxMotorSpeed);

  if (angleX > maxAngle || angleX < -maxAngle) {
    stopMotors();
    return;
  }

  if (motorSpeed < 10) {
    stopMotors();
  } else if (angleX < 0) {
    moveForward(motorSpeed);
  } else {
    moveBackward(motorSpeed);
  }

  last_error = error;

  Serial.print("Angle: ");
  Serial.print(angleX);
  Serial.print(" Output: ");
  Serial.println(Output);
}

void moveForward(int speed) {
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  analogWrite(pwmA, speed);

  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  analogWrite(pwmB, speed);
}

void moveBackward(int speed) {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  analogWrite(pwmA, speed);

  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  analogWrite(pwmB, speed);
}

void stopMotors() {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  analogWrite(pwmA, 0);

  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  analogWrite(pwmB, 0);
}
