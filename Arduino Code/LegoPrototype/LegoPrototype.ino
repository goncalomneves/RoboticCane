#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <TimedAction.h> // Source: http://playground.arduino.cc/Code/TimedAction

// Variables for encoder
#define outputA 2
#define outputB 3
int counter = 0;
int aState;
int bState;
int aLastState;
float encoder_pos = 0;
float wheel_perimeter = 25.8; // Wheel perimeter 25.8
int sensor_res = 100; // Number of slots on the encoder's disk


// Variables for IMU
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data


// Variables for motors
/* Motor A connections */
int enA = 11;
int in1 = 12;
int in2 = 13;
/* Motor B connections */
int enB = 10;
int in3 = 9;
int in4 = 8;


// Variables for controller
float pos = 0;
float last_pos, d_pos;
double angle = 0.00;
double last_angle, d_angle;
double motor_vel;
//int motor_dir;
float currentveloc;
float ctrl_timer = 0;
float ctrl_timer_old, d_ctrl_timer;
float control_input, control_signal;
float ref_angle = 0;
float ref_pos = 0;

/* LQR gains obtained from matlab code */
// For small Lego prototype
//float K1 = 0;
//float K2 = 10.724278255988015;
//float K3 = 5.56993569145128;

// For bigger Lego prototype
//float K1 = 0;
//float K2 = 15.724278255988015;
//float K3 = 8.56993569145128;

// For bigger aluminium prototype
float K1 = 0;
float K2 = 25.724278255988015;
float K3 = 20.56993569145128;

/*|||||||||||||||||||||||||||||| ENCODER ||||||||||||||||||||||||||||||*/
float Encoder() {
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    if (control_input >= 0) {
      counter ++;
    } else if (control_input < 0) {
      counter --;
    }
    encoder_pos = counter * wheel_perimeter / sensor_res; // Only clockwise movement assumed
  }
  aLastState = aState; // Updates the previous state of the outputA with the current state
  return encoder_pos;
}


/*|||||||||||||||||||||||||||||| IMU ||||||||||||||||||||||||||||||*/
void setupIMU() {
  Wire.begin();
  setup_mpu_6050_registers(); //Setup the registers of the MPU-6050

  delay(100); // Wait for sensor to stabilize

  read_mpu_6050_data();

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  // Set starting angle
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  timer = micros();
}


double IMU() {
  // Update all the values
  read_mpu_6050_data();

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  return kalAngleY;
}


void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);              //Start communicating with the MPU-6050
  Wire.write(0x6B);                          //Send the requested starting register
  Wire.write(0x00);                          //Set the requested starting register
  Wire.endTransmission();
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);              //Start communicating with the MPU-6050
  Wire.write(0x1C);                          //Send the requested starting register
  Wire.write(0x10);                          //Set the requested starting register
  Wire.endTransmission();
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);              //Start communicating with the MPU-6050
  Wire.write(0x1B);                          //Send the requested starting register
  Wire.write(0x08);                          //Set the requested starting register
  Wire.endTransmission();
}


void read_mpu_6050_data() {                  //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);              //Start communicating with the MPU-6050
  Wire.write(0x3B);                          //Send the requested starting register
  Wire.endTransmission();                    //End the transmission
  Wire.requestFrom(0x68, 14);                //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);             //Wait until all the bytes are received
  accX = Wire.read() << 8 | Wire.read();
  accY = Wire.read() << 8 | Wire.read();
  accZ = Wire.read() << 8 | Wire.read();
  tempRaw = Wire.read() << 8 | Wire.read();
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}


/*|||||||||||||||||||||||||||||| MOTORS ||||||||||||||||||||||||||||||*/
void setupMotors() {
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void MotorMovement(float veloc) {
  /*void MotorMovement(int veloc, int direc) {
    // veloc is the desired velocity for the motors
    // direc is the desired direction of rotation for the motors (0 to turn off motors, 1 for Anti-Clockwise and 2 for Clockwise

    switch (direc) {

      case 0:
        // Turn off motors
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        break;

      case 1:
        // Turn on motors Clockwise
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        break;

      case 2:
        // Turn on motors Anti-Clockwise
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        break;

      default:
        // Turn off motors if nothing is said
        digitalWrite(in1, LOW);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, LOW);
        break;
    }

    analogWrite(enA, veloc);
    analogWrite(enB, veloc);

  */
  if (veloc == currentveloc) return;

  if (veloc > 255) veloc = 255;
  else if (veloc < -255) veloc = -255;
  if (abs(angle) <= 40 ) {
    digitalWrite(in1, veloc < 0 ? HIGH : LOW);
    digitalWrite(in2, veloc < 0 ? LOW : HIGH);
    digitalWrite(in3, veloc < 0 ? HIGH : LOW);
    digitalWrite(in4, veloc < 0 ? LOW : HIGH);
    analogWrite(enA, map((abs(veloc)) * 10, 0, 2550, 40, 255));
    analogWrite(enB, map((abs(veloc)) * 10, 0, 2550, 40, 255));
  } else if (abs(angle) > 70 ) {
    veloc = 0;
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enA, 0);
    analogWrite(enB, 0);
  }
  currentveloc = veloc;
}

/*|||||||||||||||||||||||||||||| CONTROL ||||||||||||||||||||||||||||||*/
void Control() {
  // Calculate position and angle variations
  d_pos = (pos - last_pos) / d_ctrl_timer;
  //d_angle = (angle - last_angle) / d_ctrl_timer;
  d_angle = (angle - last_angle);

  // Apply LQR control
  control_input = (d_pos * K1 + (angle - ref_angle) * K2 + d_angle * K3);

  /*
    if (control_input >= 0) {
      motor_vel = constrain(abs(control_input)+30, 0, 255);
      motor_dir = 1;
    } else if (control_input < 0) {
      motor_vel = constrain(abs(control_input)+30, 0, 255);
      motor_dir = 2;
    }

    MotorMovement(motor_vel, motor_dir);
  */
  MotorMovement(control_input);
}


TimedAction ControlThread = TimedAction(0.1, Control);


/*|||||||||||||||||||||||||||||| SETUP ||||||||||||||||||||||||||||||*/
void setup() {
  pinMode (outputA, INPUT);
  pinMode (outputB, INPUT);

  Serial.begin (115200);
  Serial.setTimeout (10);

  // Reads the initial state of the outputA of the encoder
  aLastState = digitalRead(outputA);

  setupIMU();
  setupMotors();

  ctrl_timer = millis();
}


/*|||||||||||||||||||||||||||||| LOOP ||||||||||||||||||||||||||||||*/
void loop() {

  // Save last states
  last_pos = pos;
  last_angle = angle;

  // Read new states
  pos = Encoder();
  angle = IMU();

  // Obtain the time difference
  ctrl_timer_old = ctrl_timer;
  ctrl_timer = millis();
  d_ctrl_timer = (ctrl_timer - ctrl_timer_old) / 1000.0;

  ControlThread.check();

  /*// This condition is here to create a control signal that is printable and true to what is sent to the motors
  if (control_input >= 0) control_signal = map((abs(control_input)) * 10, 0, 2550, 40, 255);
  else if (control_input < 0) control_signal = - map((abs(control_input)) * 10, 0, 2550, 40, 255);*/

  // Print all signals
  /*//Serial.print("Position: "); Serial.print(pos); Serial.print("\t");
  Serial.print("Angle: "); Serial.print(angle); Serial.print("\t");
  //Serial.print("Ref: "); Serial.print(ref_angle); Serial.print("\t");
  //Serial.print("Control Input: "); Serial.print(control_input); Serial.print("\t");
  //Serial.print("Motor Vel: "); Serial.print(map((abs(control_input)) * 10, 0, 2550, 40, 255));
  Serial.println();*/
  
  // Print to csv format
  /*Serial.print(pos); Serial.print(",");
  Serial.print(d_pos); Serial.print(",");
  Serial.print(angle); Serial.print(",");
  Serial.print(d_angle); Serial.print(",");
  Serial.print(ref_pos); Serial.print(",");
  Serial.print(ref_angle); Serial.print(",");
  Serial.print(control_input); Serial.print(",");
  Serial.print(control_signal); Serial.print(",");
  Serial.print(millis()); Serial.println();*/

  // Print to Serial.Plotter
  /*Serial.print("Angle: "); Serial.print(angle);
  //Serial.print(",Ref: "); Serial.print(ref_angle);
  Serial.print(",-20,20");
  Serial.println();*/

  // Recieve angle ref value from serial
  /*if (Serial.available() > 0 ) {
  ref_angle = Serial.parseInt();
  }*/
  
  // Change ref through time
  /*if ((ctrl_timer >= 15000) && (ctrl_timer < 20000)) ref_angle = -30;
  else if ((ctrl_timer >= 20000) && (ctrl_timer < 25000)) ref_angle = -20;
  else if ((ctrl_timer >= 25000) && (ctrl_timer < 30000)) ref_angle = -10;
  else if ((ctrl_timer >= 30000) && (ctrl_timer < 35000)) ref_angle = -5;
  else if ((ctrl_timer >= 35000) && (ctrl_timer < 40000)) ref_angle = 0;
  else if ((ctrl_timer >= 40000) && (ctrl_timer < 45000)) ref_angle = 5;
  else if ((ctrl_timer >= 45000) && (ctrl_timer < 50000)) ref_angle = 10;
  else if ((ctrl_timer >= 50000) && (ctrl_timer < 55000)) ref_angle = 20;
  else if ((ctrl_timer >= 55000) && (ctrl_timer < 60000)) ref_angle = 30;
  else if (ctrl_timer >= 60000) ref_angle = 0;*/

  // Sine ref
  /*ref_angle = 7*(sin(ctrl_timer/400)+1);*/
    
  delay(2);
}
