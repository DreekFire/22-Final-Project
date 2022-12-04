#include "MPU9250.h"
#include <ESP32Encoder.h>

int MAX_RPM = 435; // no-load RPM at 12VDC
int MAX_TORQUE = 18.7; // stall torque at 12VDC, kg * cm
int TICKS_PER_REV = 384.5;

MPU9250 mpu;
ESP32Encoder encs[3];

// pin config
struct motor_pin_config {
  int IN_1;
  int IN_2;
  int PWM;
  int CLK;
  int DT;
};

struct motor_pin_config motor_cfg[3] = {
  {25, 4, 26, 15, 32},
  {17, 16, 21, 14, 33},
  {18, 5, 19, 13, 12},
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
  Wire.begin();
  delay(2000);

  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }

  // calibrate anytime you want to
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  /*Serial.println("Mag calibration will start in 5sec.");
  Serial.println("Please Wave device in a figure eight until done.");
  delay(5000);
  mpu.calibrateMag();*/

  print_calibration();
  mpu.verbose(false);
  
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

void loop() {
  static uint32_t prev_ms = millis();
  if (millis() > prev_ms + 25) {
    print_speeds();
    prev_ms = millis();
  }
  set_voltage(0, 0.15);
  set_voltage(1, 0.15);
  set_voltage(2, 0.15);
}

void print_roll_pitch_yaw() {
  Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(mpu.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(", ");
  Serial.println(mpu.getRoll(), 2);
}

void print_calibration() {
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  /*Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();*/
}

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

// motor_id: 0 to 2
// torque: -MAX_TORQUE to MAX_TORQUE
void set_torque(int motor_id, float torque) {
  float voltage = torque / MAX_TORQUE + get_speed(motor_id) / MAX_RPM;
  set_voltage(motor_id, voltage);
}

// motor_id: 0 to 2
// returns speed in RPM
float get_speed(int motor_id) {
  unsigned long t = millis();
  unsigned long dt = t - motors[motor_id].last_speed_check;
  if (dt < 10) {
    return motors[motor_id].filtered_vel / TICKS_PER_REV * 60;
  }
  long last_pos = motors[motor_id].position;
  long new_pos = encs[motor_id].getCount();
  float raw_vel = (1000.0f * (new_pos - last_pos)) / dt;
  float filtered_vel = 0.1 * raw_vel + 0.9 * motors[motor_id].filtered_vel;
  motors[motor_id].filtered_vel = filtered_vel;
  motors[motor_id].last_speed_check = t;
  motors[motor_id].position = new_pos;
  return filtered_vel / TICKS_PER_REV * 60;
}

void print_speeds() {
  Serial.println("< motor speeds >");
  Serial.print(get_speed(0));
  Serial.print(", ");
  Serial.print(get_speed(1));
  Serial.print(", ");
  Serial.print(get_speed(2));
  Serial.println();
}