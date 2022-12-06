#include "MPU9250.h"
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

//Setting various globals
int MAX_RPM = 435; // no-load RPM at 12VDC
int MAX_TORQUE = 18.7; // stall torque at 12VDC, kg * cm
int TICKS_PER_REV = 384.5;

//Bluetooth input/output vars
bool motorToggle = false;
bool writeToggle = false;
String inputPacket = "";
String outputPacket = "";

//Motor speeds
float motorVal[3] = {0.0,0.0,0.0};

//Trying to enable BT
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Importing certain vars.
MPU9250 mpu;
ESP32Encoder encs[3];
BluetoothSerial SerialBT;

// Incrimental int to ensure that printing only happens every 256th loop
int printCount = 0;

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
  SerialBT.begin("BallBot"); //Bluetooth device name
  delay(2000);

  // Trying to setup IMU on address 0x68
  if (!mpu.setup(0x68)) {  
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }

  // Calibrating IMU in stationary.
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
  
  // Looping through Motor pints and setting proper pins. Output is for writing speed values
  // input is for getting encoder values
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
  // Checking status of IMU, setting data as needed
  if (mpu.update()) {
  static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      //print_roll_pitch_yaw();
      prev_ms = millis();
    }

  }
  // Checking if it is time to print, if not we let BT have the serial for recieving commands
  if (printCount >= 256){
    print_speeds();
    printCount = 0;
    // Send routine output packet
    outputPacket = String(mpu.getYaw()) +"," + String(mpu.getPitch()) + "," + String(mpu.getRoll());

  }
  // Recieve BT commands
  else{
    //Writing to BT out
    if (outputPacket != "") {
      SerialBT.flush();
      SerialBT.println(outputPacket);
    }
    //Recieving to BT in
    if (SerialBT.available()) {
      inputPacket = SerialBT.readStringUntil('\n');
      Serial.println(inputPacket);
      inputPacket.trim();
    }
    //Read char e for toggle motors
    if (inputPacket == "e"){
      motorToggle = !motorToggle;
      Serial.print("TOGGLE");
    }
    //Clearing packets
    inputPacket = "";
    outputPacket = "";
  }

  //This line needed to ensure that the IMU data is up to date
  motorVal[0] = get_speed(0);
  motorVal[1] = get_speed(1);
  motorVal[2] = get_speed(2);
  
  printCount++;

  // Motor speed set to different voltages, toggled via variable
  if (motorToggle){
    set_voltage(0, 0.15);
    set_voltage(1, 0.15);
    set_voltage(2, 0.15);
  }
  else{
    set_voltage(0, 0);
    set_voltage(1, 0);
    set_voltage(2, 0);
  }

}

//Method to print out IMU data
void print_roll_pitch_yaw() {
  Serial.print("Yaw, Pitch, Roll: ");
  Serial.print(mpu.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(", ");
  Serial.println(mpu.getRoll(), 2);
}

//Method to calibrate IMU
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

// motor_id: 0 to 2
// torque: -MAX_TORQUE to MAX_TORQUE
void set_torque(int motor_id, float torque) {
  float voltage = torque / MAX_TORQUE + get_speed(motor_id) / MAX_RPM;
  set_voltage(motor_id, voltage);
}

// Method to read encoders
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

//Method to print each motor speed
void print_speeds() {
  Serial.println("< motor speeds >");
  Serial.print(motorVal[0]);
  Serial.print(", ");
  Serial.print(motorVal[1]);
  Serial.print(", ");
  Serial.print(motorVal[2]);
  Serial.println();
}