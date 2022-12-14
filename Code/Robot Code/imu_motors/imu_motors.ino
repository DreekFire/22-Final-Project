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
static float Kp1 = 0.0; // 0.11
static float Kd1 = 0.0; // 0.0005
static float Ki1 = 0;   // 0
static float I1 = 0;
static float offset1 = 0;

static float Kp2 = 0.0;
static float Kd2 = 0.0;
static float Ki2 = 0;
static float I2 = 0;
static float offset2 = 0;

static float KpZ = 0;
static float KiZ = 0;
static float KdZ = 0;
static float IZ = 0;

static float TORQUE_LIMIT = 0.8;

// Time Tracking
// static int tot_loop_time = 0;
// static int loops = 0;
static long tprev = 0;
static long tcurr = 0;


static float exp_decay = 0.9;

float e1, e2, eZ, d1, d2, dZ, res1, res2, resZ;

// Incrimental int to ensure that printing only happens every 256th loop
int BTCount = 0;
int printCount = 0;

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
    if (tcurr - tprev > 10) { // Feedback every 10 ms
      printCount++;
      // tot_loop_time += tcurr - tprev;
      // loops ++;
      tprev = tcurr;

      // PID Feedback:
      // X axis
      e1 = mpu.getEulerY() - offset1;
      d1 = mpu.getGyroY();

      e2 = -mpu.getEulerX() - offset2;
      d2 = -mpu.getGyroX();

      eZ = mpu.getEulerZ();
      dZ = mpu.getGyroZ();

      res1 = -Kp1 * e1 - Kd1 * d1 - Ki1 * I1;
      res2 = -Kp2 * e2 - Kd2 * d2 - Ki2 * I2;
      resZ = -KpZ * eZ - KdZ * dZ - KiZ * IZ;

      I1 = I1 * exp_decay + e1;
      I2 = I2 * exp_decay + e2;
      IZ = IZ * exp_decay + eZ;

      set_torque(0, clamp( res1 - resZ, TORQUE_LIMIT));
      set_torque(1, clamp(-res1 * COS_60 + res2 * COS_30 - resZ, TORQUE_LIMIT));
      set_torque(2, clamp(-res1 * COS_60 - res2 * COS_30 - resZ, TORQUE_LIMIT));
      
      // Serial.println(String(get_speed(0)) + ", " + String(get_speed(1)) + "," + String(get_speed(2)));
      // Serial.println(pid_logging());
    }

  } else if (mode == STOP) {
    set_voltage(0, 0);
    set_voltage(1, 0);
    set_voltage(2, 0);

    // set_torque(0, 0);
    // set_torque(1, 0);
    // set_torque(2, 0);
    // Serial.print("RPMS: ");
    // Serial.print(get_speed(0));
    // Serial.print(", ");
    // Serial.print(get_speed(1));
    // Serial.print(", ");
    // Serial.print(get_speed(2));
    // Serial.println("");

  } else {
    // Serial.println("Error did not recognize MODE");
  }


  // BLUETOOTH
  // Checking if it is time to print, if not we let BT have the serial for recieving commands
  if (BTCount >= 128){
    BTCount = 0;
    // Send routine output packet
    // outputPacket = String(x_hat[0]) +"," + String(x_hat[1]) + "," + String(x_hat[2]);
    outputPacket = pid_logging();
    // Serial.println("Avg loop time: " + String(tot_loop_time / (float) loops));
    // tot_loop_time = 0;
    // loops = 0;
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

        Kp1 = atof(strtok(NULL, ","));
        Kp2 = Kp1;

        Ki1 = atof(strtok(NULL, ","));
        Ki2 = Ki1;

        Kd1 = atof(strtok(NULL, ","));
        Kd2 = Kd1;
        
        Serial.print(Kp2);
        Serial.print(", ");
        Serial.print(Ki2);
        Serial.print(", ");
        Serial.print(Kd2);
        Serial.println(";");

      } else if (ID == "OFFSET") {
        offset1 = atof(strtok(NULL, ","));
        offset2 = atof(strtok(NULL, ","));

        Serial.print("Recieved OFFSET values: ");
        Serial.print(offset1);
        Serial.print(", ");
        Serial.print(offset2);
        Serial.println(";");

      } else if (ID == "START") {        
        mode = RUN;

        // Reset integrals to zero
        I1 = 0;
        I2 = 0;
        // Set printCount to 0
        printCount = 0;

        Serial.println("Recieved START");

      } else if (ID == "STOP") {
        mode = STOP;
        Serial.println("Recieved STOP");

      } else if (ID = "YAW") {
        KpZ = atof(strtok(NULL, ","));
        KiZ = atof(strtok(NULL, ","));
        KdZ = atof(strtok(NULL, ","));
        
        Serial.print("Recieved YAW PID values: ");
        Serial.print(KpZ);
        Serial.print(", ");
        Serial.print(KiZ);
        Serial.print(", ");
        Serial.print(KdZ);
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
  return String(tcurr) + ",   " + String(e1) + "," + String(I1) + "," + String(d1) + ",  " + String(e2) + "," + String(I2) + "," + String(d2);
}

float clamp(float x, float lim) {
  if (x > lim) {
    return lim;
  } else if (x < -lim) {
    return -lim;
  }
  return x;
}