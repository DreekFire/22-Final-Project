#include "MPU9250.h"
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

//Trying to enable BT
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Setting various globals
#define COS_30 0.866
#define COS_60 0.5

#define ROT_SEC_TO_M_S 0.003665 //(rot/min) * 1/60(min/sec) * 2pi (rad/rot * 0.035)m/rad;

static float KV = 0.00085; // 1 / no-load RPM at 12VDC - adjusted
static float KM = 1 / 1.8326; // stall torque at 12VDC, Nm
const int TICKS_PER_REV = 384.5;
const float MIN_VOLTAGE = 0.005;
const float BUMP_VOLTAGE = 0.03;

const float DEG2RAD = 3.1415 / 180;


// Run Mode:
enum Mode{RUN, STOP};
enum Mode mode = STOP;

//Bluetooth input/output vars
bool motorToggle = false;
bool writeToggle = false;
String inputPacket = "";
String outputPacket = "";

// PID constants and stats
static float Kp = 0.0; // 0.11
static float Ki = 0;   // 0
static float Kd = 0.0; // 0.0005

static float I1 = 0;
static float I2 = 0;

static float x_tilt_target = 0;
static float y_tilt_target = 0;

static float TORQUE_LIMIT = 0.8;

float e1 = 0, e2 = 0, eZ = 0, d1 = 0, d2 = 0, dZ = 0, res1 = 0, res2 = 0, resZ = 0;


// Vel PID
static float KpV = 0;
static float KiV = 0;
static float KdV = 0;

static float vel_est_decay = 0.9;

static float vel_x_est = 0;
static float vel_y_est = 0;
static float IVx = 0;
static float IVy = 0;
static float last_vel_x = 0;
static float last_vel_y = 0;
static float last_vel_time = 0;

static float x_vel_target = 0;
static float y_vel_target = 0;
const float MAX_TARGET_VEL = 0.3;

static int loop_count = 0;

const float MAX_TILT_SETPOINT = 5;

float vel_ex = 0, vel_ey = 0, vel_dx = 0, vel_dy = 0;

// Time Tracking
// static int tot_loop_time = 0;
// static int loops = 0;
static long tprev = 0;
static long tcurr = 0;


static float exp_decay = 0.99;

// Incrimental int to ensure that printing only happens every 256th loop
int BTCount = 0;

//Importing certain vars.
MPU9250 mpu;
ESP32Encoder encs[3];
BluetoothSerial SerialBT;
float pidVal[3] = {0.0,0.0,0.0};

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
  Wire.begin();
  SerialBT.begin("BallBot"); //Bluetooth device name
  MPU9250Setting setting;
  setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
  setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
  setting.mag_output_bits = MAG_OUTPUT_BITS::M14BITS;
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_1000HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_99HZ;

  // Trying to setup IMU on address 0x68
  if (!mpu.setup(0x68, setting)) {  
      while (1) {
          // Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }

  // Calibrating IMU in stationary.
  //Serial.println("Accel Gyro calibration will start in 5sec.");
  // Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  mpu.setFilterIterations(10);

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

  // Serial.println("Robot is Ready #001");
}

void loop() {
  mpu.update();
  get_speed(0);
  get_speed(1);
  get_speed(2);

  // Serial.print("RPMS:");
  // Serial.print(get_speed(0));
  // Serial.print(",");
  // Serial.print(get_speed(1));
  // Serial.print(",");
  // Serial.print(get_speed(2));
  // Serial.println(";");

  tcurr = millis();

  if (mode == RUN) {
    if (tcurr - tprev > 10) { // Angle Feedback every 10 ms
      // tot_loop_time += tcurr - tprev;
      // loops ++;
      tprev = tcurr;

      // PID Feedback:
      // X axis
      e1 = mpu.getEulerY() - x_tilt_target;
      d1 = mpu.getGyroY();

      e2 = -mpu.getEulerX() - y_tilt_target;
      d2 = -mpu.getGyroX();

      res1 = -Kp * e1 - Kd * d1 - Ki * I1;
      res2 = -Kp * e2 - Kd * d2 - Ki * I2;

      I1 = I1 * exp_decay + e1;
      I2 = I2 * exp_decay + e2;

      set_torque(0, clamp( res1, TORQUE_LIMIT));
      set_torque(1, clamp(-res1 * COS_60 + res2 * COS_30, TORQUE_LIMIT));
      set_torque(2, clamp(-res1 * COS_60 - res2 * COS_30, TORQUE_LIMIT));
      

      // Setting velocity estimation for the velocity PID
      vel_x_est = vel_est_decay * vel_x_est + (1 - vel_est_decay) * ( 0 * get_speed(0) + COS_30 * get_speed(1) - COS_30 * get_speed(2)) * ROT_SEC_TO_M_S;
      vel_y_est = vel_est_decay * vel_y_est + (1 - vel_est_decay) * (-1 * get_speed(0) + COS_60 * get_speed(1) + COS_60 * get_speed(2)) * ROT_SEC_TO_M_S; 
      loop_count++;   

      // Serial.println(String(get_speed(0)) + ", " + String(get_speed(1)) + "," + String(get_speed(2)));
      // Serial.println(pid_logging());
    }

    // Velocity PID runs at 1/10 speed of the other.
    if (loop_count >= 10) {
      loop_count = 0;

      vel_ex = vel_x_est - x_vel_target;
      vel_ey = vel_y_est - y_vel_target;

      vel_dx = (vel_x_est - last_vel_x) / (tcurr - last_vel_time);
      vel_dy = (vel_y_est - last_vel_y) / (tcurr - last_vel_time);

      x_tilt_target = -clamp(KpV * vel_ey + KiV * IVy + KdV * vel_dy, MAX_TILT_SETPOINT); // Negative because of how the x and y axes are related in tilt / direction. Positive y velocity is REDUCED by Negative angle
      y_tilt_target =  clamp(KpV * vel_ex + KiV * IVx + KdV * vel_dx, MAX_TILT_SETPOINT);

      IVx = IVx * exp_decay + vel_x_est;
      IVy = IVy * exp_decay + vel_y_est;
      last_vel_x = vel_x_est;
      last_vel_y = vel_y_est;
      last_vel_time = tcurr;

      Serial.println(vel_pid_logging());
    }


  } else if (mode == STOP) {

    set_voltage(0, 0);
    set_voltage(1, 0);
    set_voltage(2, 0);

  } else {
    // Serial.println("Error did not recognize MODE");
  }


  // BLUETOOTH
  // Checking if it is time to print, if not we let BT have the serial for recieving commands

  // if (BTCount >= 4){
  // BTCount = 0;
  if (loop_count == 0) {
    // Send routine output packet
    outputPacket = vel_pid_logging();
        //Writing to BT out
    if (outputPacket != "") {
      SerialBT.flush();
      SerialBT.println(outputPacket);
      // Serial.println(outputPacket);
    }
    //Recieving to BT in
    if (SerialBT.available()) {
      inputPacket = SerialBT.readStringUntil('|');
      // Serial.println(inputPacket);
      inputPacket.trim();
    }
  }
  // Recieve BT commands
  else{
    BTCount++;
    if(inputPacket != ""){
      char str_array[inputPacket.length() + 1];
      inputPacket.toCharArray(str_array, inputPacket.length() + 1);
      char* d = strtok(str_array, ",");

      String ID = String(d);
      if (ID == "PID") {
        Serial.print("Recieved PID values: ");

        Kp = atof(strtok(NULL, ","));
        Ki = atof(strtok(NULL, ","));
        Kd = atof(strtok(NULL, ","));
        
        Serial.print(Kp);
        Serial.print(", ");
        Serial.print(Ki);
        Serial.print(", ");
        Serial.print(Kd);
        Serial.println(";");

      } else if (ID == "OFFSET") {
        x_vel_target = clamp(atof(strtok(NULL, ",")), MAX_TARGET_VEL);
        y_vel_target = clamp(atof(strtok(NULL, ",")), MAX_TARGET_VEL);

        Serial.print("Recieved OFFSET values (NOW VELOCITY): ");
        Serial.print(x_vel_target);
        Serial.print(", ");
        Serial.print(y_vel_target);
        Serial.println(";");

      } else if (ID == "START") {        
        mode = RUN;

        // Reset integrals to zero
        I1 = 0;
        I2 = 0;

        Serial.println("Recieved START");

      } else if (ID == "STOP") {
        mode = STOP;
        Serial.println("Recieved STOP");

      } else if (ID = "VEL") {
        KpV = atof(strtok(NULL, ","));
        KiV = atof(strtok(NULL, ","));
        KdV = atof(strtok(NULL, ","));
        
        Serial.print("Recieved VEL PID values: ");
        Serial.print(KpV);
        Serial.print(", ");
        Serial.print(KiV);
        Serial.print(", ");
        Serial.print(KdV);
        Serial.println(";");
        
      } else {
        Serial.print("ERROR: Did not recognize identifier ");
        Serial.print(d);
        Serial.println(";");
      }
      SerialBT.flush();
    }
    
    //Clearing packets
    inputPacket = "";
    outputPacket = "";
  }
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
}

// Method to set motor speed
// motor_id: 0 to 2
// voltage: -1.0 to 1.0
void set_voltage(int motor_id, float voltage) {
  float absv = abs(voltage);
  if (absv > MIN_VOLTAGE) {
    absv = clamp(absv + BUMP_VOLTAGE, 1);
  }

  int pwm = (int) (absv * 255);
  bool in1 = voltage >= 0;
  bool in2 = voltage <= 0;
  digitalWrite(motor_cfg[motor_id].IN_1, in1);
  digitalWrite(motor_cfg[motor_id].IN_2, in2);
  ledcWrite(motor_id, pwm);
}

// motor_id: 0 to 2
// torque: -MAX_TORQUE to MAX_TORQUE
void set_torque(int motor_id, float torque) {
  float voltage = torque * KM + get_speed(motor_id) * KV;
  set_voltage(motor_id, voltage);
}

// Method to read encoders
// motor_id: 0 to 2
// returns speed in Rad / Sec
float get_speed(int motor_id) {
  unsigned long t = millis();
  unsigned long dt = t - motors[motor_id].last_speed_check;
  if (dt < 10) {
    return motors[motor_id].filtered_vel / TICKS_PER_REV * 60;
  }
  long last_pos = motors[motor_id].position;
  long new_pos = encs[motor_id].getCount();
  float raw_vel = - (1000.0f * (new_pos - last_pos)) / dt;
  float filtered_vel = 0.2 * raw_vel + 0.8 * motors[motor_id].filtered_vel;
  motors[motor_id].filtered_vel = filtered_vel;
  motors[motor_id].last_speed_check = t;
  motors[motor_id].position = new_pos;
  return filtered_vel / TICKS_PER_REV * 60;
}

String pid_logging() {
  return String(tcurr) + ",   " + String(e1) + "," + String(I1) + "," + String(d1) + ",  " + String(e2) + "," + String(I2) + "," + String(d2) + ",   " + String(res1) + "," + String(res2);
}

String vel_pid_logging() {
  return String(tcurr) + ",   " + String(vel_ex) + "," + String(IVx) + "," + String(vel_dx) + ",    " + 
                                  String(vel_ey) + "," + String(IVy) + "," + String(vel_dy) + ",    " + 
                                  String(y_tilt_target) + "," + String(x_tilt_target);
}

float clamp(float x, float lim) {
  if (x > lim) {
    return lim;
  } else if (x < -lim) {
    return -lim;
  }
  return x;
}