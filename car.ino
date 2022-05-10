#include "QTRSensors.h"
#include <Arduino.h>

#define ENCODER_DT 50.e3

// Pins
#define ENC_LEFT 12
#define ENC_RIGHT 13

#define LEFT_NSLP 31
#define RIGHT_NSLP 11

#define LEFT_DIR 29
#define RIGHT_DIR 30

#define LEFT_PWM 40
#define RIGHT_PWM 39

#define BUMP_0 24
#define LED_RF 41

// Variables
int16_t cur_left_pwm = 0;
int16_t cur_right_pwm = 0;

unsigned long left_encoder_count = 0;
unsigned long right_encoder_count = 0;

QTRSensors IR;

int16_t sensor_values[8] = {0};
uint16_t minimum_values[8] = {689, 589, 736, 643, 735, 712, 782, 805};
uint16_t maximum_values[8] = {1811, 1688, 1764, 1420, 1448, 1788, 1718, 1695};
float weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4}; // TODO tune
float weighted_values[8] = {0};

unsigned long last_encoder_timestamp;

unsigned long last_left_encoder_count;
unsigned long last_right_encoder_count;

float left_vel = 0.0;
float right_vel = 0.0;

float error;
float last_error;

float turn_kp = 0.175; // TODO tune
float turn_kd = 3.5;   // TODO tune
float speed = 125.0;   // TODO tune

// Left encoder interrupt
void inc_encoder_count_left()
{
  left_encoder_count += cur_left_pwm >= 0 ? 1 : -1;
}

// Right encoder interrupt
void inc_encoder_count_right()
{
  right_encoder_count += cur_right_pwm >= 0 ? 1 : -1;
}

// Set left pwm output to cur_right_pwm
void set_left_pwm()
{
  digitalWrite(LEFT_NSLP, HIGH);
  digitalWrite(LEFT_DIR, cur_left_pwm < 0);
  analogWrite(LEFT_PWM, abs(cur_left_pwm));
}

// Set right pwm output to cur_right_pwm
void set_right_pwm()
{
  digitalWrite(RIGHT_NSLP, HIGH);
  digitalWrite(RIGHT_DIR, cur_right_pwm < 0);
  analogWrite(RIGHT_PWM, abs(cur_right_pwm));
}

// Stop left motor
void stop_left(bool brake)
{
  cur_left_pwm = 0.0;
  digitalWrite(LEFT_PWM, 0);
  digitalWrite(LEFT_NSLP, brake);
}

// Stop right motor
void stop_right(bool brake)
{
  cur_right_pwm = 0.0;
  digitalWrite(RIGHT_PWM, 0);
  digitalWrite(RIGHT_NSLP, brake);
}

// Get the offset error using the IR sensors
float get_error()
{
  float error = 0;
  for (int i = 0; i < 8; i++)
  {
    Serial.print(sensor_values[i]);
    Serial.print("\t");
    sensor_values[i] -= minimum_values[i];

    weighted_values[i] = (sensor_values[i]) * 1000.0 / maximum_values[i] * weights[i];
    error += weighted_values[i];
  }
  Serial.println();
  error /= 8;

  return error;
}

void setup()
{
  pinMode(ENC_LEFT, INPUT);
  pinMode(ENC_RIGHT, INPUT);

  pinMode(LEFT_NSLP, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);

  pinMode(BUMP_0, INPUT_PULLUP);

  pinMode(LED_RF, OUTPUT);

  attachInterrupt(ENC_LEFT, inc_encoder_count_left, FALLING);
  attachInterrupt(ENC_RIGHT, inc_encoder_count_right, FALLING);

  digitalWrite(LEFT_NSLP, HIGH);
  digitalWrite(RIGHT_NSLP, HIGH);

  last_encoder_timestamp = micros();

  IR.setSensorPins((const uint8_t[]){65, 48, 64, 47, 52, 68, 53, 69}, 8);
  IR.setEmitterPins(45, 61);
  IR.setTimeout(2500);

  Serial.begin(19200);
  // delay(2000);
}

void loop()
{
  unsigned long cur_timestamp = micros();

  IR.read((uint16_t *)sensor_values);

  // Calculate turn PID output
  float error = (get_error() + last_error) / 2;
  float turn = error * turn_kp + (error - last_error) * turn_kd;
  last_error = error;

  // Calculate/set pwm signals
  cur_left_pwm = speed - turn;
  cur_right_pwm = speed + turn;
  // cur_left_pwm = cur_right_pwm = 0;
  
  set_left_pwm();
  set_right_pwm();

  // Calculate encoder velocities
  if (cur_timestamp - last_encoder_timestamp > ENCODER_DT)
  {
    left_vel = (left_encoder_count - last_left_encoder_count) / ENCODER_DT;
    right_vel = (right_encoder_count - right_encoder_count) / ENCODER_DT;

    last_left_encoder_count = left_encoder_count;
    last_right_encoder_count = right_encoder_count;

    last_encoder_timestamp = cur_timestamp;
  }
}
