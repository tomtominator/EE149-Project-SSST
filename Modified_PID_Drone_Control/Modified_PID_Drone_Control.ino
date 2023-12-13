// Modified From
// https://github.com/lobodol/drone-flight-controller/


// ---------------------------------------------------------------------------
#include <Wire.h>
#include <SoftwareSerial.h>
// ------------------- Define some constants for convenience -----------------
#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3

#define XBEE_RX  2
#define XBEE_TX  3

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3

#define X           0     // X axis
#define Y           1     // Y axis
#define Z           2     // Z axis
#define MPU_ADDRESS 0x68  // I2C address of the MPU-6050
#define FREQ        250   // Sampling frequency
#define SSF_GYRO    65.5  // Sensitivity Scale Factor of the gyro from datasheet

#define STOPPED  0
#define STARTING 1
#define STARTED  2
// ---------------- Receiver variables ---------------------------------------
SoftwareSerial XBee(XBEE_RX, XBEE_TX);

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[4];

// Duration of the pulse on each channel of the receiver in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_length[4] = {1500, 1500, 1000, 1500};

// Used to calculate pulse duration on each channel
volatile unsigned long current_time;
volatile unsigned long timer[4]; // Timer of each channel

// Used to configure which control (yaw, pitch, roll, throttle) is on which channel
int mode_mapping[4];

// ----------------------- MPU variables -------------------------------------
// The RAW values got from gyro (in °/sec) in that order: X, Y, Z
int gyro_raw[3] = {0, 0, 0};

// Average gyro offsets of each axis in that order: X, Y, Z
long gyro_offset[3] = {0, 0, 0};
long acc_offset[3] = {0, 0, 0};

// Calculated angles from gyro's values in that order: X, Y, Z
float gyro_angle[3]  = {0, 0, 0};

// The RAW values got from accelerometer (in m/sec²) in that order: X, Y, Z
int acc_raw[3] = {0 , 0 , 0};

// Calculated angles from accelerometer's values in that order: X, Y, Z
float acc_angle[3] = {0, 0, 0};

// Total 3D acceleration vector in m/s²
long acc_total_vector;

// Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};

/**
   Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
    - Left wing up implies a positive roll
    - Nose up implies a positive pitch
    - Nose right implies a positive yaw
*/
float measures[3] = {0, 0, 0};
float measures_offset[3] = {0, 0, 0};

// MPU's temperature
int temperature;

// Init flag set to TRUE after first loop
boolean initialized;
// ----------------------- Variables for servo signal generation -------------
unsigned int  period; // Sampling period
unsigned long loop_timer;
unsigned long now, difference;

unsigned long pulse_length_esc1 = 1000,
              pulse_length_esc2 = 1000,
              pulse_length_esc3 = 1000,
              pulse_length_esc4 = 1000;

// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = {0, 0, 0}; // Yaw, Pitch, Roll
// Errors
float errors[3];                     // Measured errors (compared to instructions) : [Yaw, Pitch, Roll]
float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Yaw, Pitch, Roll
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Yaw, Pitch, Roll]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Yaw, Pitch, Roll]
// PID coefficients
float Kp[3] = {4.0, 1, 1}; //{4.0, 1.3, 1.3};    // P coefficients in that order : Yaw, Pitch, Roll
float Ki[3] = {0, 0, 0};//{0.02, 0.04, 0.04}; // I coefficients in that order : Yaw, Pitch, Roll
float Kd[3] = {0, 0, 0}; //{0, 18, 18};        // D coefficients in that order : Yaw, Pitch, Roll
// ---------------------------------------------------------------------------
/**
   Status of the quadcopter:
     - 0 : stopped
     - 1 : starting
     - 2 : started

   @var int
*/
int status = STOPPED;
// ---------------------------------------------------------------------------
float battery_voltage;
// ---------------------------------------------------------------------------

// Thomas Added:
int to_start = 1;
bool started = false;
int throttle_print = 0;
unsigned long throttle_last_incr = 0;
int input_throttle = 1000;
int target_throttle = input_throttle;
long incoming = 0;

int incr = 0;

bool calibrate_acc = false;
bool gyro_angle_integration = false;

#define PID_RANGE 100 // Thomas: Original was 400 

#define BATTERY_PIN 3  // pin the voltage is read from, orignial was A0

/**
   Setup configuration
*/
void setup() {
  Serial.begin(115200); // temp change to work with xbee
  XBee.begin(115200);

  // Start I2C communication
  Wire.begin();
  TWBR = 12; // Set the I2C clock speed to 400kHz.

  // Turn LED on during setup
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  configureChannelMapping();

  // Set pins #4 #5 #6 #7 as outputs
  DDRD |= B11110000;

  Serial.println("Begin IMU setup");
  setupMpu6050Registers();

  calibrateMpu6050();

  period = (1000000 / FREQ) ; // Sampling period in µs

  // Initialize loop_timer
  loop_timer = micros();

  // Thomas: moved from isStarted()
  // Reset PID controller's variables to prevent bump start
  resetPidController();
  resetGyroAngles();

  Serial.println("IMU armed");

  setFlat(); // Thomas: begins measuring and zeros any different between IMU and drone orientation (note drone must start flat)

  // Turn LED off now setup is done
  digitalWrite(13, LOW);
  Serial.println("Setup Complete");
}

/**
   Main program loop
*/
void loop() {
  readSensor();

  calculateAngles();

  calculateErrorsAlternative();

  if (!started) {
    Serial.println("Start PID");
    started = true;
  }

  // choose how to input throttle
  //serialThrottle();
  xbeeThrottle(); // set target throttle

  // Set immediately or ramp throttle
  //input_throttle = target_throttle;
  rampThrottle();

  pidController(input_throttle);

//  compensateBatteryDrop();

  applyMotorSpeed();

}

/**
   Sean: Receive throttle via serial (over cable)

*/
void serialThrottle() {
  if (Serial.available() > 0) {
    incoming = Serial.parseInt();
    if (incoming != 0) {
      input_throttle = incoming;
      //Serial.print("Throttle set: ");
      //Serial.println(input_throttle);
    }
  }
}

/**
   Sean: Receive throttle via xbee (wireless)

*/
String RF_command = "";
int times_seen = 0;
int prev_cmd = 1000;

void xbeeThrottle() {
  if (XBee.available() > 0) {
    char data = XBee.read();
    //Serial.println(data);
    
    if (data == '.' && RF_command.length() == 4) { // syntax failsafe
      int cmd = RF_command.toInt();
      if (cmd == prev_cmd) {
        times_seen += 1;
      } else {
        times_seen = 0;
        prev_cmd = cmd;
      }
      if (cmd >= 1000 && cmd <= 2000 && times_seen > 5) { // 5 continuous msgs
        target_throttle = cmd;
      }
      if (cmd == 911) {
        target_throttle = 1000;
      }
      
      XBee.print("M1");
      XBee.println(pulse_length_esc1);
      XBee.print("M2");
      XBee.println(pulse_length_esc2);
      XBee.print("M3");
      XBee.println(pulse_length_esc3);
      XBee.print("M4");
      XBee.println(pulse_length_esc4);
      XBee.print("Head");
      XBee.println(measures[ROLL] - measures_offset[ROLL]);
      XBee.print("Ang");
      XBee.println(angular_motions[ROLL]);
      XBee.print("V");
      XBee.print(battery_voltage);
      Serial.print("V");
      Serial.println(battery_voltage);
      RF_command = "";
    } else if (data == '.' || RF_command.length() > 4) {
      RF_command = "";
    } else {
      RF_command += data;
    }
    //Serial.println(RF_command);
  }
}

// Ramp throttle
void rampThrottle() {
  // Ramp up
  if (target_throttle == input_throttle) {
    //Serial.print("Target reached: ");
    //Serial.println(input_throttle);
    return;
  } else if (target_throttle > input_throttle) {
    if (throttle_last_incr - millis() > 200) { // increase throttle by 1 every 50ms
      input_throttle += 1;
      throttle_last_incr = millis();
      //Serial.print("Ramped Up: ");
      //Serial.println(input_throttle);
    }
  }
  // Ramp down
  else if (target_throttle < input_throttle) {
    if (throttle_last_incr - millis() > 20) { // decrease by 1 every 20ms
      input_throttle -= 1;
      throttle_last_incr = millis();
      //Serial.print("Ramped Down: ");
      //Serial.println(input_throttle);
    }
  }
}

/**
   Generate servo-signal on digital pins #4 #5 #6 #7 with a frequency of 250Hz (4ms period).
   Direct port manipulation is used for performances.

   This function might not take more than 2ms to run, which lets 2ms remaining to do other stuff.

   @see https:// www.arduino.cc/en/Reference/PortManipulation
*/
void applyMotorSpeed() {
  // Refresh rate is 250Hz: send ESC pulses every 4000µs
  while ((now = micros()) - loop_timer < period);

  // Update loop timer
  loop_timer = now;

  // Set pins #4 #5 #6 #7 HIGH
  PORTD |= B11110000;

  // Wait until all pins #4 #5 #6 #7 are LOW
  while (PORTD >= 16) {
    now        = micros();
    difference = now - loop_timer;

    if (difference >= pulse_length_esc1) PORTD &= B11101111; // Set pin #4 LOW
    if (difference >= pulse_length_esc2) PORTD &= B11011111; // Set pin #5 LOW
    if (difference >= pulse_length_esc3) PORTD &= B10111111; // Set pin #6 LOW
    if (difference >= pulse_length_esc4) PORTD &= B01111111; // Set pin #7 LOW
  }
}

/**
   Request raw values from MPU6050.
*/
void readSensor() {
  Wire.beginTransmission(MPU_ADDRESS); // Start communicating with the MPU-6050
  Wire.write(0x3B);                    // Send the requested starting register
  Wire.endTransmission();              // End the transmission
  Wire.requestFrom(MPU_ADDRESS, 14);   // Request 14 bytes from the MPU-6050

  // Wait until all the bytes are received
  while (Wire.available() < 14);

  acc_raw[X]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[X] variable
  acc_raw[Y]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Y] variable
  acc_raw[Z]  = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the acc_raw[Z] variable
  temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable
  gyro_raw[X] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[X] variable
  gyro_raw[Y] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Y] variable
  gyro_raw[Z] = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the gyro_raw[Z] variable
}


/**
   Thomas: Set what is flat for drone (may be differnt than IMU flat)
*/
void setFlat() {
  long start = millis();
  // Do this for 2 seconds

  while (2000 > millis() - start) {
    readSensor();
    calculateAngles();
  }
  measures_offset[ROLL] = measures[ROLL];
  measures_offset[PITCH] = measures[PITCH];
}
/**
   Calculate real angles from gyro and accelerometer's values
*/
void calculateAngles() {
  calculateGyroAngles();
  calculateAccelerometerAngles();

  if (initialized) {
    // Correct the drift of the gyro with the accelerometer
    gyro_angle[X] = gyro_angle[X] * 0.996 + acc_angle[X] * 0.004;
    gyro_angle[Y] = gyro_angle[Y] * 0.996 + acc_angle[Y] * 0.004;
  } else {
    // At very first start, init gyro angles with accelerometer angles
    resetGyroAngles();

    initialized = true;
  }

  // To dampen the pitch and roll angles a complementary filter is used
  measures[ROLL]  = measures[ROLL]  * 0.5 + gyro_angle[X] * 0.5;
  measures[PITCH] = measures[PITCH] * 0.5 + gyro_angle[Y] * 0.5;
  measures[YAW]   = -gyro_raw[Z] / SSF_GYRO; // Store the angular motion for this axis

  // Apply low-pass filter (10Hz cutoff frequency)
  angular_motions[ROLL]  = 0.7 * angular_motions[ROLL]  + 0.3 * gyro_raw[X] / SSF_GYRO;
  angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO;
  angular_motions[YAW]   = 0.7 * angular_motions[YAW]   + 0.3 * gyro_raw[Z] / SSF_GYRO;
}

/**
   Calculate pitch & roll angles using only the gyro.
*/
void calculateGyroAngles() {
  // Subtract offsets
  gyro_raw[X] -= gyro_offset[X];
  gyro_raw[Y] -= gyro_offset[Y];
  gyro_raw[Z] -= gyro_offset[Z];

  // Angle calculation using integration
  gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO));
  gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO)); // Change sign to match the accelerometer's one

  // Transfer roll to pitch if IMU has yawed
  gyro_angle[Y] += gyro_angle[X] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
  gyro_angle[X] -= gyro_angle[Y] * sin(gyro_raw[Z] * (PI / (FREQ * SSF_GYRO * 180)));
}

/**
   Calculate pitch & roll angles using only the accelerometer.
*/
void calculateAccelerometerAngles() {
  // Thomas: calibrate acc as well
  if (calibrate_acc) {
    acc_raw[X] -= acc_offset[X];
    acc_raw[Y] -= acc_offset[Y];
    acc_raw[Z] -= acc_offset[Z];
  }

  // Calculate total 3D acceleration vector : √(X² + Y² + Z²)
  acc_total_vector = sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2));

  // To prevent asin to produce a NaN, make sure the input value is within [-1;+1]
  if (abs(acc_raw[X]) < acc_total_vector) {
    acc_angle[X] = asin((float)acc_raw[Y] / acc_total_vector) * (180 / PI); // asin gives angle in radian. Convert to degree multiplying by 180/pi
  }

  if (abs(acc_raw[Y]) < acc_total_vector) {
    acc_angle[Y] = asin((float)acc_raw[X] / acc_total_vector) * (180 / PI);
  }
}

/**
   Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
   by applying PID control.

   (A) (B)     x
     \ /     z ↑
      X       \|
     / \       +----→ y
   (C) (D)

   Motors A & D run clockwise.
   Motors B & C run counter-clockwise.

   Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
*/
void pidController(int input_throttle) {
  float yaw_pid      = 0;
  float pitch_pid    = 0;
  float roll_pid     = 0;
  int   throttle     = input_throttle; // 1000 == 0, 1005 == fly!?

  // Initialize motor commands with throttle
  pulse_length_esc1 = throttle;
  pulse_length_esc2 = throttle;
  pulse_length_esc3 = throttle;
  pulse_length_esc4 = throttle;

  // Do not calculate anything if throttle is 0
  if (throttle > 1000) {
    // PID = e.Kp + ∫e.Ki + Δe.Kd
    yaw_pid   = (errors[YAW]   * Kp[YAW])   + (error_sum[YAW]   * Ki[YAW])   + (delta_err[YAW]   * Kd[YAW]);
    pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
    roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

    // Keep values within acceptable range
    yaw_pid   = minMax(yaw_pid, -PID_RANGE, PID_RANGE);
    pitch_pid = minMax(pitch_pid, -PID_RANGE, PID_RANGE);
    roll_pid  = minMax(roll_pid, -PID_RANGE, PID_RANGE);

    // Calculate pulse duration for each ESC
    pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid;
    pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid;
    pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid;
    pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid;
  }

  // Prevent out-of-range-values
  pulse_length_esc1 = minMax(pulse_length_esc1, 1000, 1200); // Max value 2000, 1200 for testing purposes
  pulse_length_esc2 = minMax(pulse_length_esc2, 1000, 1200);
  pulse_length_esc3 = minMax(pulse_length_esc3, 1000, 1200);
  pulse_length_esc4 = minMax(pulse_length_esc4, 1000, 1200);

  throttle_print += 1;
   if (throttle_print >  300) {
//     XBee.println("Motors: ");
//     Serial.println("Send");
//      XBee.println(pulse_length_esc1);
//      XBee.println(pulse_length_esc2);
//      XBee.println(pulse_length_esc3);
//      XBee.println(pulse_length_esc4);
     // XBee.print("Heading: ");
     // XBee.println(measures[ROLL] - measures_offset[ROLL]);
     // XBee.print("Ang V: ");
     // XBee.println(angular_motions[ROLL]);
     throttle_print = 0;
   }
  if (throttle_print > 30) {
      Serial.println("Throttle Set: ");
      Serial.print((int)target_throttle-1000);
      Serial.print("\t");
      Serial.print((int)(pulse_length_esc1 - 1000));
      Serial.print("\t");
      Serial.print((int)pulse_length_esc2-1000);
      Serial.print("  ");
      Serial.print((int)pulse_length_esc3-1000);
      Serial.print("  ");
      Serial.print((int)(pulse_length_esc4 - 1000));

    // Serial.print("PID: ");
    // Serial.print(pitch_pid);
    // Serial.print("  ");
    // Serial.print(roll_pid);
    // Serial.print("  ");
    // Serial.println(yaw_pid);

    // Serial.print("Error: ");
    // Serial.print(errors[PITCH] + errors[ROLL]);
    // Serial.print("  ");
    // Serial.print(errors[ROLL]);
    // Serial.print("  ");
    // Serial.println(errors[YAW]);
    // Serial.print("Error Sum: ");
    // Serial.print(error_sum[PITCH]);
    // Serial.print("  ");
    // Serial.print(error_sum[ROLL]);
    // Serial.print("  ");
    // Serial.println(error_sum[YAW]);
    // Serial.print("Error delta: ");
    // Serial.print(delta_err[PITCH]);
    // Serial.print("  ");
    // Serial.print(delta_err[ROLL]);
    // Serial.print("  ");
    // Serial.println(delta_err[YAW]);
    // Serial.print("Set points: ");
    // Serial.print(pid_set_points[PITCH]);
    // Serial.print("  ");
    // Serial.print(pid_set_points[ROLL]);
    // Serial.print("  ");
    // Serial.println(pid_set_points[YAW]);

    // Serial.print("Measured: ");
      Serial.print("\t");

      Serial.print(measures[ROLL] - measures_offset[ROLL]);
      Serial.print("\t");
    // Serial.print("  ");
    // Serial.println(measures[ROLL] - measures_offset[ROLL]);

    // Serial.print("Angular Motion: ");
      Serial.println(angular_motions[ROLL]);
    // Serial.print("  ");
    // Serial.println(angular_motions[ROLL]);

    // Serial.print("Gyro Angle: ");
    // Serial.print(gyro_angle[X]);
    // Serial.print("  ");
    // Serial.println(gyro_angle[Y]);

    // Serial.print("Acc raw: ");
    // Serial.print(acc_raw[X]);
    // Serial.print("  ");
    // Serial.println(acc_raw[Y]);

    // Serial.print("Acc Offset from Calibration: ");
    // Serial.print(acc_offset[X]);
    // Serial.print("  ");
    // Serial.println(acc_offset[Y]);

    // Serial.print("Gyro Raw (calibrated): ");
    // Serial.print(gyro_raw[X]);
    // Serial.print("  ");
    // Serial.print(gyro_raw[Y]);
    // Serial.print("  ");
    // Serial.println(gyro_raw[Z]);

    // Serial.print("Gyro Offset from Calibration: ");
    // Serial.print(gyro_offset[X]);
    // Serial.print("  ");
    // Serial.println(gyro_offset[Y]);


    throttle_print = 0;
    // Serial.println("-------");
  }
}

// Thomas: alternative error calculation. Does not use angular motion but
//         attempts to set the measure orientation to 0.0, 0.0

void calculateErrorsAlternative() {
  // Calculate current errors
  errors[YAW]   = 0; //angular_motions[YAW]   - pid_set_points[YAW]; // Don't change YAW error for now
  errors[PITCH] = ((measures[PITCH] - measures_offset[PITCH]) - 0) + .5 * (angular_motions[PITCH] - 0); // desired angular motion is 0
  errors[ROLL]  = ((measures[ROLL]  - measures_offset[ROLL]) - 0) + .5 * (angular_motions[ROLL] - 0);

  // if (abs(errors[PITCH] + errors[ROLL]) < 3) {
  //   errors[PITCH] = 0;
  //   errors[ROLL] = 0;
  // }

  // Calculate sum of errors : Integral coefficients
  error_sum[YAW]   += errors[YAW] / 1000;
  error_sum[PITCH] += errors[PITCH] / 1000;
  error_sum[ROLL]  += errors[ROLL] / 1000;

  // Keep values in acceptable range
  error_sum[YAW]   = minMax(error_sum[YAW],   -PID_RANGE / Ki[YAW],   PID_RANGE / Ki[YAW]);
  error_sum[PITCH] = minMax(error_sum[PITCH], -PID_RANGE / Ki[PITCH], PID_RANGE / Ki[PITCH]);
  error_sum[ROLL]  = minMax(error_sum[ROLL],  -PID_RANGE / Ki[ROLL],  PID_RANGE / Ki[ROLL]);

  // Calculate error delta : Derivative coefficients
  delta_err[YAW]   = errors[YAW]   - previous_error[YAW];
  delta_err[PITCH] = errors[PITCH] - previous_error[PITCH];
  delta_err[ROLL]  = errors[ROLL]  - previous_error[ROLL];

  // Save current error as previous_error for next time
  previous_error[YAW]   = errors[YAW];
  previous_error[PITCH] = errors[PITCH];
  previous_error[ROLL]  = errors[ROLL];
}

/**
   Configure gyro and accelerometer precision as following:
    - accelerometer: ±8g
    - gyro: ±500°/s

   @see https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
*/
void setupMpu6050Registers() {
  // Configure power management
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
  Wire.write(0x00);                    // Apply the desired configuration to the register
  Wire.endTransmission();              // End the transmission

  // Configure the gyro's sensitivity
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1B);                    // Request the GYRO_CONFIG register
  Wire.write(0x08);                    // Apply the desired configuration to the register : ±500°/s
  Wire.endTransmission();              // End the transmission

  // Configure the acceleromter's sensitivity
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
  Wire.write(0x10);                    // Apply the desired configuration to the register : ±8g
  Wire.endTransmission();              // End the transmission

  // Configure low pass filter
  Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
  Wire.write(0x1A);                    // Request the CONFIG register
  Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
  Wire.endTransmission();              // End the transmission
}

/**
   Calibrate MPU6050: take 2000 samples to calculate average offsets.
   During this step, the quadcopter needs to be static and on a horizontal surface.

   This function also sends low throttle signal to each ESC to init and prevent them beeping annoyingly.

   This function might take ~2sec for 2000 samples.
*/
void calibrateMpu6050() {
  int max_samples = 2000;

  for (int i = 0; i < max_samples; i++) {
    readSensor();

    gyro_offset[X] += gyro_raw[X];
    gyro_offset[Y] += gyro_raw[Y];
    gyro_offset[Z] += gyro_raw[Z];

    // Thomas: calibrate acc as well
    acc_offset[X] += acc_raw[X];
    acc_offset[Y] += acc_raw[Y];
    acc_offset[Z] += acc_raw[Z];

    // Generate low throttle pulse to init ESC and prevent them beeping
    PORTD |= B11110000;      // Set pins #4 #5 #6 #7 HIGH
    delayMicroseconds(1000); // Wait 1000µs
    PORTD &= B00001111;      // Then set LOW

    // Just wait a bit before next loop
    delay(3);
  }

  // Calculate average offsets
  gyro_offset[X] /= max_samples;
  gyro_offset[Y] /= max_samples;
  gyro_offset[Z] /= max_samples;

  // Thomas: calibrate acc as well
  acc_offset[X] /= max_samples;
  acc_offset[Y] /= max_samples;
  acc_offset[Z] /= max_samples;

}

/**
   Make sure that given value is not over min_value/max_value range.

   @param float value     : The value to convert
   @param float min_value : The min value
   @param float max_value : The max value

   @return float
*/
float minMax(float value, float min_value, float max_value) {
  if (value > max_value) {
    value = max_value;
  } else if (value < min_value) {
    value = min_value;
  }

  return value;
}

/**
   Return whether the quadcopter is started.
   To start the quadcopter, move the left stick in bottom left corner then, move it back in center position.
   To stop the quadcopter move the left stick in bottom right corner.

   @return bool
*/
bool isStarted() {
  // When left stick is moved in the bottom left corner
  if (status == STOPPED && to_start == 1) { // Autostart after 1 sec
    status = STARTING;
    Serial.println("Starting");
    to_start = 0;
  }

  // When left stick is moved back in the center position
  if (status == STARTING) { // Begin flying after 3 sec
    status = STARTED;

    // Reset PID controller's variables to prevent bump start
    resetPidController();

    resetGyroAngles();
    Serial.println("Started");
  }

  // When left stick is moved in the bottom right corner
  // if (status == STARTED && micros() > 200000000) { // stop after 200 seconds
  //     status = STOPPED;
  //     // Make sure to always stop motors when status is STOPPED
  //     stopAll();
  //     Serial.println("Stopped");
  // }

  return status == STARTED;
}

/**
   Reset gyro's angles with accelerometer's angles.
*/
void resetGyroAngles() {
  gyro_angle[X] = acc_angle[X];
  gyro_angle[Y] = acc_angle[Y];
}

/**
   Reset motors' pulse length to 1000µs to totally stop them.
*/
void stopAll() {
  pulse_length_esc1 = 1000;
  pulse_length_esc2 = 1000;
  pulse_length_esc3 = 1000;
  pulse_length_esc4 = 1000;
}

/**
   Reset all PID controller's variables.
*/
void resetPidController() {
  errors[YAW]   = 0;
  errors[PITCH] = 0;
  errors[ROLL]  = 0;

  error_sum[YAW]   = 0;
  error_sum[PITCH] = 0;
  error_sum[ROLL]  = 0;

  previous_error[YAW]   = 0;
  previous_error[PITCH] = 0;
  previous_error[ROLL]  = 0;
}


/**
   Customize mapping of controls: set here which command is on which channel and call
   this function in setup() routine.
*/
void configureChannelMapping() {
  mode_mapping[YAW]      = CHANNEL4;
  mode_mapping[PITCH]    = CHANNEL2;
  mode_mapping[ROLL]     = CHANNEL1;
  mode_mapping[THROTTLE] = CHANNEL3;
}

/**
   Compensate battery drop applying a coefficient on output values
*/
void compensateBatteryDrop() {
  // battery_voltage = battery_voltage * 0.92 + (analogRead(BATTERY_PIN) + 65) * 0.09853;
  float battery_reading = analogRead(BATTERY_PIN) * (5.0 / 1023.0);
  battery_voltage = battery_reading / 0.333; // / 0.389;
  //Serial.println(battery_voltage);
  if (battery_voltage > 11) {
  } else {
    Serial.println("BATTERY DEAD");
    Serial.print("Battery Voltage: ");
    Serial.println(battery_voltage);

    stopAll();
    while (true);
  }
}
