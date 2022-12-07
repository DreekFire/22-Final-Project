#include <ESP32Encoder.h>

//Setting various globals
int MAX_RPM = 435; // no-load RPM at 12VDC
int MAX_TORQUE = 18.7; // stall torque at 12VDC, kg * cm
int TICKS_PER_REV = 384.5;

float DEG2RAD = 3.1415 / 180;

//Motor speeds
float motorVal[3] = {0.0,0.0,0.0};

ESP32Encoder encs[3];

// pin config
struct motor_pin_config {
  int IN_1;
  int IN_2;
  int PWM;
  int CLK;
  int DT;
};

// Setting pins for each motor
// pin(0) = IN_1, pin(1) = IN_2
// pin(2) = PWM, pin(3) = CLK, pin(4) = DT
// Output on motor from L to R is DT, CLK, GND, V
struct motor_pin_config motor_cfg[3] = {
  {25, 4, 26, 15, 32},
  {17, 16, 21, 14, 33},
  {18, 5, 19, 34, 39},
};

struct motor {
  long position;
  float filtered_vel;
  unsigned long last_speed_check;
};

struct motor motors[3] = {
  {0, 0.0, 0},
  {0, 0.0, 0},
  {0, 0.0, 0},
};


void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(motor_cfg[i].IN_1, OUTPUT);
    pinMode(motor_cfg[i].IN_2, OUTPUT);
    pinMode(motor_cfg[i].PWM, OUTPUT);
    pinMode(motor_cfg[i].DT, INPUT);
    pinMode(motor_cfg[i].CLK, INPUT);
    ledcAttachPin(motor_cfg[i].PWM, i);
    ledcSetup(i, 1000, 8);
    encs[i].attachFullQuad(motor_cfg[i].DT, motor_cfg[i].CLK);
    encs[i].setCount(0);
  }
}

int motor = 0;
float volt = 0.0;
int up = 1;
float incr = 0.005;

int i = 0;

void loop() {
  get_speed(motor);
  delay(1);

  if (i >= 100) {
    if (up) {
      if (abs(get_speed(motor)) > 0.1 || volt > 1) {
        up = 0;
        Serial.println("Motor " + String(motor) + " min start voltage: " + String(volt));
      }
      volt += incr;
    } else {
      if (abs(get_speed(motor)) <= 0.1 || volt <= 0) {
        up = 1;
        set_voltage(motor, 0);
        Serial.println("Motor " + String(motor) + " max stop voltage: " + String(volt));
        motor = (motor + 1) % 3;
        volt = 0;
      }
      volt -= incr;
    }
    set_voltage(motor, volt);
    // Serial.println("Volts: " + String(volt) + ", Speed: " + String(get_speed(motor)));
  }
}

// Method to set motor speed
// motor_id: 0 to 2
// voltage: -1.0 to 1.0
void set_voltage(int motor_id, float voltage) {
  int pwm = (int) (abs(voltage) * 255);
  bool in1 = voltage >= 0;
  bool in2 = voltage <= 0;
  digitalWrite(motor_cfg[motor_id].IN_1, in1);
  digitalWrite(motor_cfg[motor_id].IN_2, in2);
  ledcWrite(motor_id, pwm);
}

float get_speed(int motor_id) {
  unsigned long t = millis();
  unsigned long dt = t - motors[motor_id].last_speed_check;
  if (dt < 10) {
    return motors[motor_id].filtered_vel / TICKS_PER_REV * 60;
  }
  long last_pos = motors[motor_id].position;
  long new_pos = encs[motor_id].getCount();
  float raw_vel = (1000.0f * (new_pos - last_pos)) / dt;
  float filtered_vel = 0.4 * raw_vel + 0.6 * motors[motor_id].filtered_vel;
  motors[motor_id].filtered_vel = filtered_vel;
  motors[motor_id].last_speed_check = t;
  motors[motor_id].position = new_pos;
  return filtered_vel / TICKS_PER_REV * 60;
}

