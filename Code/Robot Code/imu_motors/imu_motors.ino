#include "MPU9250.h"
#include <ESP32Encoder.h>
#include "BluetoothSerial.h"

//Trying to enable BT
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//Setting various globals
const int MAX_RPM = 435; // no-load RPM at 12VDC
const int MAX_TORQUE = 1.8326; // stall torque at 12VDC, Nm
const int TICKS_PER_REV = 384.5;
const float MIN_VOLTAGE = 0.02;
const float BUMP_VOLTAGE = 0.08;

const float DEG2RAD = 3.1415 / 180;

//Bluetooth input/output vars
bool motorToggle = false;
bool writeToggle = false;
String inputPacket = "";
String outputPacket = "";

// ****** CONTROL MATRICES: ********//
float K[30] = {-0.0, -0.0, -2.56292, -0.0, -0.03106, -0.0, -0.0007, -0.51443, -0.0, -0.02855, 0.0, 0.0, 1.28146, 2.21959, -0.03106, 0.00061, 0.00035, 0.25721, 0.4453, -0.02855, -0.0, 0.0, 1.28146, -2.21959, -0.03106, -0.00061, 0.00035, 0.25721, -0.4453, -0.02855};
float AKF[100] = {0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00726, 0.0, 0.0, -0.0, 1.36625, -1.56541, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00726, 0.0, 1.3667, 0.0, 0.0, -1.56546, 0.0, 0.0, 0.0, 0.0, 0.0, -0.12197, 0.0, -0.0, 0.0, 0.0, 0.00845, 0.0, 0.0, -0.0, -0.07548, 0.0, -56.25401, 0.0, -0.0, 5.48676, 0.0, 0.0, 0.0, -0.07548, 0.0, 0.0, 0.0, -56.25458, 5.48742, -0.0, 0.0, 0.0, 0.0, 30.42992, 0.0, 0.0, -0.0, 10.41518, -16.88857, 0.0, 0.0, 0.0, 0.0, 0.0, 30.46967, 0.0, 10.42059, -0.0, 0.0, -16.89523, 0.0, 0.0, 0.0, 0.0, 0.0, -0.00484, 0.0, -0.0, 0.0, -0.0, -14.31788};
float BKF_bot[15] = {0.0, -3.93952, 3.93952, 4.54896, -2.27448, -2.27448, -17.52692, 8.76342, 8.76342, 0.0, 15.19822, -15.19822, -29.25137, -29.25137, -29.25137};
float LKF[90] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.3919, -0.0, -0.0, -0.06183, 0.03091, 0.03091, 0.00726, -0.0, -0.0, -0.0, 2.39189, -0.0, 0.0, 0.05356, -0.05356, -0.0, 0.00726, -0.0, -0.0, -0.0, 0.48367, -0.09049, -0.09049, -0.09049, -0.0, -0.0, 0.12197, -0.0, 1.6575, -0.0, -0.0, -2.2047, 2.2047, 0.0, 0.00262, -0.0, 1.65691, -0.0, -0.0, 2.54579, -1.2729, -1.2729, 0.00262, -0.0, -0.0, 15.56584, -0.0, -0.0, -0.47134, 0.23567, 0.23567, 0.02392, -0.0, -0.0, -0.0, 15.57181, -0.0, 0.0, 0.4084, -0.4084, -0.0, 0.02392, -0.0, -0.0, -0.0, 6.98411, -1.30665, -1.30665, -1.30665, -0.0, -0.0, 0.00484};

// Control States:
float x_hat[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float y[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
float u[3] = {0, 0, 0};
long int tprev = 0;

// Incrimental int to ensure that printing only happens every 256th loop
int printCount = 0;

//Importing certain vars.
MPU9250 mpu;
ESP32Encoder encs[3];
BluetoothSerial SerialBT;


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
  setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_92HZ;
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
  Serial.println("Accel Gyro calibration will start in 5sec.");
  Serial.println("Please leave the device still on the flat plane.");
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
}

void loop() {
  mpu.update();

  // State Feedback:
  read_sensors_to_y();
  calculate_feedback();

  // set_torque(0, u[0] * 0.5);
  // set_torque(1, u[1] * 0.5);
  // set_torque(2, u[2] * 0.5);

  set_voltage(0, 0.3);
  set_voltage(1, -0.15);
  set_voltage(2, -0.15);

  // set_voltage(0, 0.5);
  // set_voltage(1, 0.5);
  // set_voltage(2, 0.5);

  if (printCount % 127 == 0) {
    // printCoutn=0;
    // String outstr = "";
    // for (int i=6; i<9; i++) {
    //   outstr = outstr + String(y[i]) + ",";
    // }
    // Serial.println(outstr);

    Serial.println(state_logging());

  }


  // ******* BLACK VOODOO BLUETOOTH STUFF ******** //
  // Checking if it is time to print, if not we let BT have the serial for recieving commands
  if (printCount >= 256){
    printCount = 0;
    // Send routine output packet
    // outputPacket = String(x_hat[0]) +"," + String(x_hat[1]) + "," + String(x_hat[2]);
    outputPacket = state_logging();

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
  
  printCount++;
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
  if (absv > MIN_VOLTAGE && absv < BUMP_VOLTAGE) {
    absv = BUMP_VOLTAGE;
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
  float raw_vel = - (1000.0f * (new_pos - last_pos)) / dt;
  float filtered_vel = 0.2 * raw_vel + 0.8 * motors[motor_id].filtered_vel;
  motors[motor_id].filtered_vel = filtered_vel;
  motors[motor_id].last_speed_check = t;
  motors[motor_id].position = new_pos;
  return filtered_vel / TICKS_PER_REV * 60;
}

//******************** CONTROL CODE *****************************//

//Method to create the required y-vector for control
void read_sensors_to_y() {
	// y[0:3] = [dphix, dphiy, dphiz]
	y[0] = DEG2RAD * (mpu.getGyroY());  // X axis is y
	y[1] = -DEG2RAD * (mpu.getGyroX()); // Y axis is x
	y[2] = DEG2RAD * (mpu.getGyroZ());

	// y[3:6] = wheel speed measurements (wheels 1 through 3)
	y[3] = get_speed(0) * (6.28 / 60.);
	y[4] = get_speed(1) * (6.28 / 60.);
	y[5] = get_speed(2) * (6.28 / 60.);

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
	mat_vec_mul_plus(BKF_bot, u, d_xhat + 5, 5, 3);
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
  rval += "     ";
  for (int i=0; i<9; i++) {
    rval = rval + String(y[i]) + ",";
  }
  rval += "     ";
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