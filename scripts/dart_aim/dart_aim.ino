#define ATTINY88
//#define USE_WIRE

  #ifdef ATTINY85
#include <TinyWireM.h>
#define Wire TinyWireM
  #elif defined(ATTINY88)
    #ifdef USE_WIRE
#include <Wire.h>
    #else
#define I2C_HARDWARE 1
#include <SoftI2CMaster.h>
    #endif
  #endif

#include <digitalWriteFast.h>


// Define Pins
#define ANGLE_SERVO_PIN 7
#define RELEASE_SERVO_PIN 9
#define ULTRASONIC_TRIG_PIN 1
#define ULTRASONIC_ECHO_PIN 2
#define BUTTON_PIN 14
#define LASER_PIN 5
#define VALID_ROLL_LIGHT_PIN 18 // Displays if the roll is below the minimum amount
#define VALID_SHOT_LIGHT_PIN 21 // Displays if there is a valid shooting angle

// For moving either servo
#define UPDATE_DELAY 20000
#define UPDATE_COUNT 2000000 / UPDATE_DELAY

// Angle Servo
#define SERVO_DELAY 250000
const double ANGLE_SERVO_OFFSET = -3 * DEG_TO_RAD;
const double ANGLE_SERVO_MIN_ANGLE = 0 * DEG_TO_RAD;
const double ANGLE_SERVO_MAX_ANGLE = 45.0 * DEG_TO_RAD;
const double ANGLE_SERVO_DEFAULT_ANGLE = 35.0 * DEG_TO_RAD;

// Release Servo
const double RELEASE_SERVO_MIN_ANGLE = 10 * DEG_TO_RAD;
const double RELEASE_SERVO_MAX_ANGLE = -28.07 * DEG_TO_RAD;

// Ultrasonic sensor
#define MAX_DELTA 0.1
//#define SENSOR_DELAY 400000
double distance = 0;
double prevDistance = distance;

// Tilt sensor
const double TILT_SENSOR_OFFSET = 0 * DEG_TO_RAD;
#define MPU_ADDR 0x68
#define minVal 265
#define maxVal 402
#define t_diff  137
double pitchAngle = 0;
double rollAngle = 0;

// Shot management
#define MAX_ROLL 5 * DEG_TO_RAD
#define MIN_VALID_TIME 100
bool idleMode = true;
bool validShot = false; 
unsigned long validShotStartTime = 0;
double shootAngle = 0;
bool prevButtonPressed = false;


void setup() {
  // Set pin modes
  pinModeFast(ANGLE_SERVO_PIN, OUTPUT);
  pinModeFast(RELEASE_SERVO_PIN, OUTPUT);
  pinModeFast(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinModeFast(ULTRASONIC_ECHO_PIN, INPUT);

  pinModeFast(BUTTON_PIN, INPUT);

  pinModeFast(LASER_PIN, OUTPUT);
  pinModeFast(VALID_ROLL_LIGHT_PIN, OUTPUT);
  pinModeFast(VALID_SHOT_LIGHT_PIN, OUTPUT);


  // Set up Gyro sensor reading
  // sda: 0   scl: 2
  #ifdef USE_WIRE
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);  // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B);                  // PWR_MGMT_1 register
  Wire.write(0x00);                  // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  #else
  i2c_init();
  i2c_start((MPU_ADDR << 1) | I2C_WRITE);
  i2c_write(0x6B);
  i2c_write(0x00);
  i2c_stop();
  #endif


  // Set angle to 0
  int c = 0;
  while(c < UPDATE_COUNT){
    moveServo(ANGLE_SERVO_PIN, 0 + ANGLE_SERVO_OFFSET);
    delayMicroseconds(UPDATE_DELAY);
    c++;
  }
}


void loop() {
  // IDLE MODE (Waiting to load dart)
  if(idleMode){
    digitalWriteFast(VALID_ROLL_LIGHT_PIN, LOW);
    if(digitalReadFast(BUTTON_PIN)){
      if(!prevButtonPressed)
        LoadDart();

      prevButtonPressed = true;
    }
    else {
      prevButtonPressed = false;
    }
  }

  // ACTIVE MODE (Finding angle to shoot dart)
  else{
    // Find angle and distance
    getDistance();
    getTiltAngle();
    // Determine angle at which to move the servo to
    double prevShootAngle = shootAngle;
    findShootAngle(pitchAngle, distance);

    // Check if the angle is within the allowed bounds, if not, don't move the servo
    if(!(shootAngle + ANGLE_SERVO_OFFSET >= ANGLE_SERVO_MIN_ANGLE  &&  shootAngle + ANGLE_SERVO_OFFSET <= ANGLE_SERVO_MAX_ANGLE)){
      // Re-save the previous shoot angle
      shootAngle = prevShootAngle;
      // Display error
      digitalWriteFast(VALID_SHOT_LIGHT_PIN, LOW);
    }

    // Valid shot angle
    else{
      digitalWriteFast(VALID_SHOT_LIGHT_PIN, HIGH);

      // Move servo to shoot angle
      moveServo(ANGLE_SERVO_PIN, shootAngle + ANGLE_SERVO_OFFSET);

      // Ensure the roll angle is small enough
      if(rollAngle < -MAX_ROLL || rollAngle > MAX_ROLL){
        digitalWriteFast(VALID_ROLL_LIGHT_PIN, LOW);
      }
      else{
        digitalWriteFast(VALID_ROLL_LIGHT_PIN, HIGH);

        // At this point, the shot should be valid
        if(!validShot){
          validShot = true;
          validShotStartTime = millis();
        }
        else if(millis() - validShotStartTime > MIN_VALID_TIME){
          if(digitalReadFast(BUTTON_PIN)){
            validShot = false;
            prevButtonPressed = true;
            ReleaseDart();
          }
        }

        return;
      }
    }


    validShot = false;
  }
}

// Closes the release mechanism to lock in the dart
__attribute__((always_inline))
void LoadDart(){
  // Close release mechanism
  int c = 0;
  while(c < UPDATE_COUNT){
    moveServo(RELEASE_SERVO_PIN, RELEASE_SERVO_MAX_ANGLE);
    delayMicroseconds(UPDATE_DELAY);
    c++;
  }
  // Turn on laser
  digitalWriteFast(LASER_PIN, HIGH);
  // End idle mode
  idleMode = false;
}

// Releases the dart
__attribute__((always_inline))
void ReleaseDart(){
  // Open release mechanism
  int c = 0;
  while(c < UPDATE_COUNT){
    moveServo(RELEASE_SERVO_PIN, RELEASE_SERVO_MIN_ANGLE);
    delayMicroseconds(UPDATE_DELAY);
    c++;
  }
  // Turn off laser
  digitalWriteFast(LASER_PIN, LOW);
  // Start idle mode
  idleMode = true;

  // Move angle to 0 for reloading
  c = 0;
  while(c < UPDATE_COUNT){
    moveServo(ANGLE_SERVO_PIN, 0 + ANGLE_SERVO_OFFSET);
    delayMicroseconds(UPDATE_DELAY);
    c++;
  }
}


// Returns the angle at which the tilt sensor is tilted, in radians
__attribute__((always_inline))
void getTiltAngle() {

  #ifdef USE_WIRE
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false);     // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 6);  // request a total of 3*2=6 registers
  
  int16_t acc_x = Wire.read() << 8 | Wire.read();  // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  int16_t acc_y = Wire.read() << 8 | Wire.read();  // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  int16_t acc_z = Wire.read() << 8 | Wire.read();  // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)

  #else
  i2c_start((MPU_ADDR << 1) | I2C_WRITE);
  i2c_write(0x3B);               // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  i2c_rep_start((MPU_ADDR << 1) | I2C_READ);     // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.

  int16_t acc_x = i2c_read(false) << 8 | i2c_read(false);  // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  int16_t acc_y = i2c_read(false) << 8 | i2c_read(false);  // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  int16_t acc_z = i2c_read(false) << 8 | i2c_read(true);  // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  i2c_stop();
  #endif

  float xAng = -((float)(acc_x - minVal) / t_diff) * 180 + 90;
  float yAng = -((float)(acc_y - minVal) / t_diff) * 180 + 90;
  float zAng = -((float)(acc_z - minVal) / t_diff) * 180 + 90;

  pitchAngle = atan2(xAng, zAng) + PI; // y
  rollAngle = atan2(yAng, zAng) - PI; // x

  if (pitchAngle > PI)
    pitchAngle -= TWO_PI;
}

// Finds the distance from the back of the ultrasonic sensor
__attribute__((always_inline))
void getDistance() {
  // Ping the sensor
  digitalWriteFast(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWriteFast(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWriteFast(ULTRASONIC_TRIG_PIN, LOW);

  // Read input
  double duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);

  // Set distance from duration to get the distance in meters
  double newDistance = duration / 2 * .000343;
  // Subtract by an inch to be at the back of the sensor
  newDistance -= 0.0254;

  // Only set the on this pass if the difference is small enough, to reduce noise
  double diff = newDistance - prevDistance;
  if(diff < MAX_DELTA && diff > -MAX_DELTA)
    distance = newDistance;

  prevDistance = newDistance;


  //delayMicroseconds(SENSOR_DELAY);
}

// Moves the servo to specified rotation in radians
__attribute__((always_inline))
void moveServo(const int servoPin, const double newAngle) {
  // int delayTime = (int)(1000 * (1.5 + (newAngle / 90.0))); //in degrees
  // int delayTime = (int)(1500 + (newAngle / .09)); // in degrees (more efficient)
  int delayTime = (int)(1500 + (newAngle / 0.00157079632679489661923132169164)); // convert above to radians
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(delayTime);
  digitalWrite(servoPin, LOW);
  //delayMicroseconds(UPDATE_DELAY - delayTime); //NOT SURE I NEED THIS, MAY ONLY NEED IT WHEN THERE AREN'T EXTERNAL DELAYS
}


const double g = 9.81;
//const double s = 4.75; // Two springs
const double s = 5.2; // Three springs
const double oX = 0.067;
const double oY = 0.0258;
const double r = 0.0855;
const double alpha = 1;
const double EPSILON = 0.0000001;
const double BOUND = 70.0 * DEG_TO_RAD;
const int MAX_ITER = 2000;

//#define cos(a) sin((PI/2) - (a)) USES MORE MEMORY FOR SOME REASON

// Finds the angle that the angle servo should move to, in radians
__attribute__((always_inline))
void findShootAngle(const double theta1, const double d) {
  if (theta1 > BOUND || theta1 < -BOUND) {
    shootAngle = NAN;
    return;
  }

  const double dx = d * cos(theta1);
  const double dy = d * sin(theta1);

  const double cosTheta1 = cos(theta1);
  const double sinTheta1 = sin(theta1);

  double guessTheta2 = shootAngle;
  int iter = 0;
  while (true) {
    // Use the guess to find the error
    const double thetaT = theta1 + guessTheta2;

    const double vx = s * cos(thetaT);
    const double vy = s * sin(thetaT);

    const double rx = r * cos(guessTheta2);
    const double ry = r * sin(guessTheta2);
    const double cx = oX - rx;
    const double cy = oY + ry;

    const double Dx = dx + (cx * cosTheta1) + (cy * sinTheta1);
    const double Dy = dy + (cx * sinTheta1) - (cy * cosTheta1);

    const double DxOvervx = Dx / vx;

    const double err = vy * DxOvervx - (g / 2) * DxOvervx * DxOvervx - Dy;

    // Check if the error is small enough
    if (err <= EPSILON && err >= -EPSILON) { // This uses less instruction memory than using abs()
      break;
    }
  
    // Update the guess
    const double step = -alpha * err;
    guessTheta2 += step;

    // If guess becomes too high, there most likely isn't a solution
    if(guessTheta2 < -PI/2 || guessTheta2 > PI/2){
      shootAngle = NAN;
      return;
    }

    // Limit the number of iterations
    iter++;
    if(iter >= MAX_ITER)
      break;
  }

  shootAngle = guessTheta2;
}