#include <ECE3.h> 

#define ENCODER_DT 50.e3



#define LEFT_NSLP 31
#define RIGHT_NSLP 11

#define LEFT_DIR 29
#define RIGHT_DIR 30

#define LEFT_PWM 40
#define RIGHT_PWM 39

#define BUMP_0 24
#define LED_RF 41

unsigned long last_encoder_timestamp;

unsigned long left_encoder_count;
unsigned long right_encoder_count;
unsigned long last_left_encoder_count;
unsigned long last_right_encoder_count;

long left_encoder_velocity;
long right_encoder_velocity;

void set_left_pwm(int speed){
  digitalWrite(LEFT_NSLP,HIGH);
  digitalWrite(LEFT_DIR, speed < 0);
  analogWrite(LEFT_PWM, abs(speed));
}

void set_right_pwm(int speed){
  digitalWrite(RIGHT_NSLP,HIGH);
  digitalWrite(RIGHT_DIR, speed < 0);
  analogWrite(RIGHT_PWM, abs(speed));
}

void stop_left(bool brake){
  digitalWrite(LEFT_PWM, 0);
  digitalWrite(LEFT_NSLP, brake);
}

void stop_right(bool brake){
  digitalWrite(RIGHT_PWM, 0);
  digitalWrite(RIGHT_NSLP, brake);
}


void setup() {
  pinMode(LEFT_NSLP,OUTPUT);
  pinMode(LEFT_DIR,OUTPUT);
  pinMode(LEFT_PWM ,OUTPUT);
  pinMode(BUMP_0,INPUT_PULLUP);
  pinMode(LED_RF, OUTPUT);

  digitalWrite(LEFT_NSLP,HIGH);
  digitalWrite(RIGHT_NSLP,HIGH);
  
  last_encoder_timestamp = micros();

  ECE3_Init();
  resetEncoderCount_left();

  Serial.begin(19200);
  
  delay(2000);
}

uint16_t sensor_values[8] = {0};
uint16_t minimum_values[8] = {0};
float  weighted_values[8] = {0};

float error;
float last_error;

float kp = 100;
float kd = 20;
float speed = 75;
float weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

float get_error(){
  ECE3_read_IR(sensor_values);
  float error = 0;
  for(int i = 0; i < 8; i++){
    sensor_values[i] -= minimum_values[i];

    weighted_values[i] = (sensor_values[i]/1000.)*weights[i];
    error += weighted_values[i];
    Serial.print(weighted_values[i]);
    Serial.print(", ");
  }
   Serial.print(error);
  Serial.println();
  error /= 8;

  return error;
}

void loop() {
  unsigned long cur_timestamp = micros();
  
  float error = get_error();
  float output = error * kp + (error - last_error ) * kd;
  last_error = error;
  
  set_left_pwm(speed - output);
  set_right_pwm(speed + output);

  if(cur_timestamp - last_encoder_timestamp > ENCODER_DT) { 
    left_encoder_count = getEncoderCount_left();
    right_encoder_count = getEncoderCount_right();
    left_encoder_velocity = left_encoder_count - last_left_encoder_count;
    right_encoder_velocity = right_encoder_count - last_right_encoder_count;
    last_left_encoder_count = left_encoder_count;
    last_right_encoder_count = right_encoder_count;
    last_encoder_timestamp = cur_timestamp;
  }
}
