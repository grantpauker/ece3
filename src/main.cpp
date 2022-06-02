#include "QTRSensors.h"
#include <Arduino.h>

// Pins
#define ENC_LEFT 12
#define ENC_RIGHT 13

#define LEFT_NSLP 31
#define RIGHT_NSLP 11

#define LEFT_DIR 29
#define RIGHT_DIR 30

#define LEFT_PWM 40
#define RIGHT_PWM 39

#define LED_FR 41
#define LED_FL 51
#define LED_BL 57
#define LED_BR 58

// Enums
enum State
{
  FOLLOW,
  TURN,
  STOP
};

enum Route
{
  FORWARD,
  BACKWARD
};

// Constants
#define INIT_DELAY 2e3
#define TIME_GOAL 15e3

#define SLOW_SPEED 105
#define FAST_SPEED 225
#define TURN_SPEED 200

#define TURN_KP 0.175
#define TURN_KD 3.0

#define BLACK_THRESHOLD 1650
#define TURN_DIST 360

#define FORWARD_FAST_START 1300
#define FORWARD_FAST_END 4600
#define BACKWARD_FAST_START 1200
#define BACKWARD_FAST_END 4750

uint16_t minimum_values[8] = {689, 589, 736, 643, 735, 712, 782, 805};
uint16_t maximum_values[8] = {1811, 1688, 1764, 1420, 1448, 1788, 1718, 1695};
float weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

// Global variables
QTRSensors IR;

int16_t sensor_values[8] = {0};
float weighted_values[8] = {0};

int32_t left_encoder_count = 0;
int32_t right_encoder_count = 0;

int16_t cur_left_pwm = 0;
int16_t cur_right_pwm = 0;

float error;
float last_error;

float speed;

State cur_state = FOLLOW;
Route route = FORWARD;

uint16_t black_count = 0;

int32_t start_time = 0;
int32_t end_time = 0;

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

// Set left pwm output to cur_left_pwm
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

// Get the offset error using the IR sensors
float get_error()
{
  float error = 0;
  for (size_t i = 0; i < 8; i++)
  {
    weighted_values[i] = (sensor_values[i] - minimum_values[i]) * 1000.0 / maximum_values[i] * weights[i];
    error += weighted_values[i];
  }
  error /= 8;
  return error;
}

// Checks if the car is on the black line
bool is_on_black()
{
  int32_t sum = 0;
  for (size_t i = 0; i < 8; i++)
  {
    sum += sensor_values[i];
  }
  sum /= 8;
  return sum > BLACK_THRESHOLD;
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

  pinMode(LED_FL, OUTPUT);
  pinMode(LED_FR, OUTPUT);
  pinMode(LED_BL, OUTPUT);
  pinMode(LED_BR, OUTPUT);

  attachInterrupt(ENC_LEFT, inc_encoder_count_left, FALLING);
  attachInterrupt(ENC_RIGHT, inc_encoder_count_right, FALLING);

  digitalWrite(LEFT_NSLP, HIGH);
  digitalWrite(RIGHT_NSLP, HIGH);

  IR.setSensorPins((const uint8_t[]){65, 48, 64, 47, 52, 68, 53, 69}, 8);
  IR.setEmitterPins(45, 61);
  IR.setTimeout(2500);

  Serial.begin(9600);
  delay(INIT_DELAY);
  start_time = millis();
}

void loop()
{
  IR.read((uint16_t *)sensor_values);

  int32_t distance = (left_encoder_count + right_encoder_count) / 2;
  if (route == FORWARD)
  {
    if (FORWARD_FAST_START < distance && distance < FORWARD_FAST_END)
    {
      speed = FAST_SPEED;
    }
    else
    {
      speed = SLOW_SPEED;
    }
  }
  else if (route == BACKWARD)
  {
    if (BACKWARD_FAST_START < distance && distance < BACKWARD_FAST_END)
    {
      speed = FAST_SPEED;
    }
    else
    {
      speed = SLOW_SPEED;
    }
  }

  if (cur_state == FOLLOW)
  {
    // Calculate turn PID output
    float error = (get_error() + last_error) / 2;
    float turn = error * TURN_KP + (error - last_error) * TURN_KD;
    last_error = error;

    cur_left_pwm = speed - turn;
    cur_right_pwm = speed + turn;
  }
  else if (cur_state == TURN)
  {
    cur_left_pwm = TURN_SPEED;
    cur_right_pwm = -TURN_SPEED;
  }
  else
  {
    cur_left_pwm = 0;
    cur_right_pwm = 0;
  }

  set_left_pwm();
  set_right_pwm();

  if (is_on_black())
  {
    black_count++;
  }
  else
  {
    black_count = 0;
  }

  if (cur_state == FOLLOW && black_count > 1)
  {
    left_encoder_count = 0;
    right_encoder_count = 0;
    cur_state = TURN;
    if (route == FORWARD)
    {
      cur_state = TURN;
    }
    else
    {
      cur_state = STOP;
      end_time = millis();
    }
  }
  else if (cur_state == TURN && left_encoder_count > TURN_DIST)
  {
    cur_state = FOLLOW;
    route = route == FORWARD ? BACKWARD : FORWARD;
  }

  bool is_under_time = (cur_state == STOP) && (end_time - start_time) < TIME_GOAL;
  digitalWrite(LED_FR, is_under_time);
}
