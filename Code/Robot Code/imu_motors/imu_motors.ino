#include "MPU9250.h"
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

//Setting various globals
int MAX_RPM = 435; // no-load RPM at 12VDC
int MAX_TORQUE = 1.8326; // stall torque at 12VDC, Nm
int TICKS_PER_REV = 384.5;

float DEG2RAD = 3.1415 / 180;

//Bluetooth input/output vars
bool motorToggle = false;
bool writeToggle = false;
String inputPacket = "";
String outputPacket = "";

// ****** CONTROL MATRICES: ********//
float K[30] = {0.0, -0.87287, -6.56277, -0.0, -0.09823, 0.0, -1.13553, -1.38214, 0.0, -0.05754, 0.75593, 0.43643, 3.28137, 5.68272, -0.09823, 0.9833, 0.56776, 0.69107, 1.19635, -0.05754, -0.75593, 0.43643, 3.28137, -5.68272, -0.09823, -0.9833, 0.56776, 0.69107, -1.19635, -0.05754};
float AKF[100] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.04669, 0.0, -0.0, -0.0, 13.00869, -1.87431, 0.0, 0.0, 0.0, 0.0, 0.0, -0.04665, 0.0, 13.0096, -0.0, 0.0, -1.87414, 0.0, 0.0, 0.0, -0.0, 0.0, -0.22601, -0.0, 0.0, -0.0, 0.0, 0.1694, 0.0, 0.0, 0.0, -0.08791, 0.0, -1.41972, -0.0, 0.0, -0.23864, 0.0, 0.0, 0.0, -0.08792, 0.0, -0.0, -0.0, -1.42143, -0.23838, 0.0, 0.0, 0.0, 0.0, 30.2094, 0.0, -0.0, -0.0, 69.70034, -15.52235, 0.0, 0.0, 0.0, 0.0, 0.0, 30.2492, 0.0, 69.75045, -0.0, 0.0, -15.53149, 0.0, 0.0, 0.0, 0.0, 0.0, -0.02658, -0.0, -0.0, 0.0, 0.0, -1.10815};
float BKF_bot[15] = {0.0, -3.93952, 3.93952, 4.54896, -2.27448, -2.27448, -17.52692, 8.76342, 8.76342, 0.0, 15.19822, -15.19822, -29.25137, -29.25137, -29.25137};
float LKF[90] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2222, -0.0, -0.0, -0.58871, 0.29435, 0.29435, 0.04669, -0.0, 0.0, -0.0, 1.22192, -0.0, 0.0, 0.50987, -0.50987, -0.0, 0.04665, -0.0, 0.0, -0.0, 0.13289, -0.12431, -0.12431, -0.12431, 0.0, -0.0, 0.22601, -0.0, 0.41895, -0.0, 0.0, -0.05564, 0.05564, -0.0, 0.01505, -0.0, 0.4189, -0.0, -0.0, 0.06433, -0.03216, -0.03216, 0.01506, -0.0, 0.0, 6.67041, -0.0, -0.0, -3.15428, 1.57714, 1.57714, 0.24444, -0.0, 0.0, -0.0, 6.67318, -0.0, 0.0, 2.73365, -2.73365, -0.0, 0.24438, -0.0, -0.0, -0.0, 0.17729, -0.16585, -0.16585, -0.16585, -0.0, -0.0, 0.02658};

// Control States:
float x_hat[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float y[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float u[3] = {0, 0, 0};
long int tprev = 0;

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
  setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_500HZ;
  setting.gyro_fchoice = 0x03;
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
  setting.accel_fchoice = 0x01;
  setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

  // Trying to setup IMU on address 0x68
  if (!mpu.setup(0x68, setting)) {  
      while (1) {
          // Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }

  // Calibrating IMU in stationary.
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
  mpu.verbose(true);
  delay(5000);
  mpu.calibrateAccelGyro();

  mpu.setFilterIterations(20);

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

  //This line needed to ensure that the Encoder data is up to date
  motorVal[0] = get_speed(0);
  motorVal[1] = get_speed(1);
  motorVal[2] = get_speed(2);


  // State Feedback:
  read_sensors_to_y();
  calculate_feedback();
  set_torque(0, u[0]);
  set_torque(1, u[1]);
  set_torque(2, u[2]);


  if (printCount > 127) {
    printCount=0;
    // String outstr = "";
    Serial.println(state_logging());
    // for (int i=0; i<3; i++) {
    //   outstr = outstr + String(motorVal[i]) + ", " + String(u[i]) + ", ";
    // }
    // Serial.println(outstr);
  }


  // Checking if it is time to print, if not we let BT have the serial for recieving commands
  // if (printCount >= 256){
  //   print_speeds();
  //   printCount = 0;
  //   // Send routine output packet
  //   // outputPacket = String(x_hat[0]) +"," + String(x_hat[1]) + "," + String(x_hat[2]);
  //   outputPacket = state_logging();

  // }
  // // Recieve BT commands
  // else{
  //   //Writing to BT out
  //   if (outputPacket != "") {
  //     SerialBT.flush();
  //     SerialBT.println(outputPacket);
  //   }
  //   //Recieving to BT in
  //   if (SerialBT.available()) {
  //     inputPacket = SerialBT.readStringUntil('\n');
  //     Serial.println(inputPacket);
  //     inputPacket.trim();
  //   }
  //   //Read char e for toggle motors
  //   if (inputPacket == "e"){
  //     motorToggle = !motorToggle;
  //     Serial.print("TOGGLE");
  //   }
  //   //Clearing packets
  //   inputPacket = "";
  //   outputPacket = "";
  // }
  
  printCount++;

  // Motor speed set to different voltages, toggled via variable
  // if (motorToggle){
  //   set_voltage(0, 0.15);
  //   set_voltage(1, 0.15);
  //   set_voltage(2, 0.15);
  // }
  // else{
  //   set_voltage(0, 0);
  //   set_voltage(1, 0);
  //   set_voltage(2, 0);
  // }

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
}

// Method to set motor speed
// motor_id: 0 to 2
// voltage: -1.0 to 1.0
void set_voltage(int motor_id, float voltage) {
  float absv = abs(voltage);
  if (absv < 0.12) {
    absv = 0;
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
  float voltage = torque / MAX_TORQUE + get_speed(motor_id) / MAX_RPM;
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
  float raw_vel = (1000.0f * (new_pos - last_pos)) / dt;
  float filtered_vel = 0.2 * raw_vel + 0.8 * motors[motor_id].filtered_vel;
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


//******************** CONTROL CODE *****************************//

//Method to create the required y-vector for control
void read_sensors_to_y() {
	// y[0:3] = [dphix, dphiy, dphiz]
	y[0] = DEG2RAD * (mpu.getGyroY());  // X axis is y
	y[1] = DEG2RAD * -(mpu.getGyroX()); // Y axis is -x
	y[2] = DEG2RAD * (mpu.getGyroZ());

	// y[3:6] = wheel speed measurements (wheels 1 through 3)
	y[3] = motorVal[0] * (6.28 / 60.);
	y[4] = motorVal[1] * (6.28 / 60.);
	y[5] = motorVal[2] * (6.28 / 60.);

	// y[6:9] = Roll Pitch Yaw
	y[6] = DEG2RAD * mpu.getEulerY(); // X axis is (y)
	y[7] = -DEG2RAD * mpu.getEulerX(); // Y axis is (-x)
	y[8] = DEG2RAD * mpu.getEulerZ();
}

// Method for feedback control
void calculate_feedback() {
	// xhat += (t - tprev) * (AK @ xhat + B @ u + LKF @ y)
	float d_xhat[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	mat_vec_mul_plus(AKF, x_hat, d_xhat, 10, 10);
	// mat_vec_mul_plus(BKF_bot, u, d_xhat + 5, 5, 3);
	mat_vec_mul_plus(LKF, y, d_xhat, 10, 9);

	if (tprev == 0) {
		tprev = millis();
	}
	long int tnew = millis();
	float dt = (tnew - tprev) / 1000.0;

	for (int i=0; i<10; i++) {
		x_hat[i] += dt * d_xhat[i];
	}

	// tprev = t
	tprev = tnew;
	// u = K @ xhat
  float x_hat_temp[] = {0, 0, y[6], y[7], y[8], 0, 0, y[0], y[1], y[2]};
	mat_vec_mul_replace(K, x_hat_temp, u, 3, 10);
}

// Generate logging string for x_hat, y, and u
String state_logging() {
  String rval = String(tprev) + ",";
  for (int i=0; i<10; i++) {
    rval = rval + String(x_hat[i]) + ",";
  }
  for (int i=0; i<9; i++) {
    rval = rval + String(y[i]) + ",";
  }
  for (int i=0; i<3; i++) {
    rval = rval + String(u[i]) + ",";
  }
  return rval;
}

// Method for matrix multiplication
void mat_vec_mul_plus(float A[], float b[], float res[], int n, int m) {
	for (int i=0; i<n; i++) {
		for (int j=0; j<m; j++) {
			res[i] += A[i*m + j] * b[j];
		}
	}
}

void mat_vec_mul_replace(float A[], float b[], float res[], int n,int m) {
	for (int i=0; i<n; i++) {
		res[i] = 0;
		for (int j=0; j<m; j++) {
			res[i] += A[i*m + j] * b[j];
		}
	}
}