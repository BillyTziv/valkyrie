/**
 * Valkyrie 1.0
 *
 * Developed by Vasilis Tzivaras
 * contact me at vtzivaras(at)gmail.com
 * 
 * *******************************************
 * Εquipment used
 * 
 * Arduino Mega
 * HC-SR04 Ultrasonic Sensor
 * Motor Shield v2
 * Two DC Motors
 * One Servo
 * *******************************************
 * Useful documentation
 * 
 * Website reference: https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino?view=all
 * 
 * Documentation about the Adafruit Motor Shield V2 Arduino Library used can be found here: http://adafruit.github.io/Adafruit_Motor_Shield_V2_Library/html/annotated.html
 * *******************************************
 * Author: Vassilis Tzivaras
 * Created: 15/01/2023
 * Updated: 15/01/2023
 * 
 * Email: vtzivaras@gmail.com
 * *******************************************
 */
 
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <NewPing.h>

// Pinout definition
#define TRIG_PIN  6      // Ultrasonic distance sensor trig pin attached to.
#define ECHO_PIN 7          // Ultrasonic distance sensor echo pin attached to.
#define MAX_DISTANCE 400    // Maximum distance we want to measure (in centimeters).
#define L_ENC 18            // Left motor encoder.
#define R_ENC 19            // Right motor encoder.
#define SERVO_PIN 10        // Servo.

#define WHEEL_DIAMETER 20   // cm per rotation a wheel does.
#define WHEEL_HOLES 21      // Number of holes a wheel encoder has.
#define DES_DISTANCE 30       // Desired distance in cm.

// Enable/Disable the test runs.
const boolean ENABLE_TEST_RUN = false;

// Initialize DC Motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor *lMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rMotor = AFMS.getMotor(4);

// Initialize Servo
Servo fSonar;

// Initialize Ultrasonic Distance Sensor
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

// Used to count the total rotations of the wheel encoders.
// 21 holes is a full rotation (20cm).
int lRot, rRot, avgRot = 0;

// Keep the left and right wheel speed.
int leftWheelSpeed = 150;
int rightWheelSpeed = 150;

// ............
int pos = 0;
int error = 0;
int duration = 0;

long prevT = 0;
float eprev = 0;
float eintegral = 0;

// ---------------------------------------------
// TEST RUNS
// ---------------------------------------------
boolean testMotors() {
  moveForward( 25 );
  delay(1000);
  stopMoving();
  delay(500);

  moveBackward( 25 );
  delay(1000);
  stopMoving();
  delay(500);

  turnRight( 100 );
  delay(1000);
  stopMoving();
  delay(500);

  turnLeft( 100 );
  delay(1000);
  stopMoving();
  delay(500);

  return true;
}

boolean testServo() {
  
  for(int i=10; i<=170; i++) {
    fSonar.write(i);
    delay(15);
  }

  for(int i=170; i>=10; i--) {
    fSonar.write(i);
    delay(15);
  }
  
  fSonar.write(90);

  return true;
}

boolean testSonar() {
  return true;
}

boolean testLED() {
  return true;
}

// ---------------------------------------------
// METHODS
// ---------------------------------------------
void moveForward(int vSpeed) {
  lMotor->setSpeed( vSpeed );
  rMotor->setSpeed( vSpeed );
  lMotor->run(FORWARD);
  rMotor->run(FORWARD);
}

void moveBackward(int vSpeed) {
  lMotor->setSpeed( vSpeed );
  rMotor->setSpeed( vSpeed );
  lMotor->run(BACKWARD);
  rMotor->run(BACKWARD);
}

void stopMoving() {
  lMotor->setSpeed( 0 );
  rMotor->setSpeed( 0 );
  lMotor->run(RELEASE);
  rMotor->run(RELEASE);
}

void turnLeft(int vSpeed) {
  lMotor->setSpeed( vSpeed );
  rMotor->setSpeed( vSpeed );
  lMotor->run(BACKWARD);
  rMotor->run(FORWARD);
}

void turnRight(int vSpeed) {
  lMotor->setSpeed( vSpeed );
  rMotor->setSpeed( vSpeed );
  lMotor->run(FORWARD);
  rMotor->run(BACKWARD);
}

// ---------------------------------------------
// SETUP
// ---------------------------------------------
void setup() {
  Serial.begin(9600);

  Serial.println("\n---------------------------------------");
  Serial.print("\t Valkyrie 1.0");
  Serial.println("\n---------------------------------------");

  AFMS.begin();
  fSonar.attach( SERVO_PIN );
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input
  //pinMode(L_ENC, INPUT);
  //pinMode(R_ENC, INPUT);
  //attachInterrupt( digitalPinToInterrupt(L_ENC), updateLMHRot, RISING);
  //attachInterrupt( digitalPinToInterrupt(R_ENC), updateRMRot, RISING);

  if( ENABLE_TEST_RUN ) {
    Serial.print("Testing Motors...");
    if( testMotors() ) Serial.println("OK");

    Serial.print("Testing Servo...");
    if( testServo() ) Serial.println("OK");
    
    Serial.print("Testing Sonar...");
    if( testSonar() ) Serial.println("OK");

    Serial.print("Testing LED...");
    if( testLED() ) Serial.println("OK");

    Serial.println("\nAll tests passed...\n");
  }
  
  delay(500);
  
  Serial.println("Valkyrie is ready...");
}

// void attempt1() {
//   error = 250 - avgRot;
  
//   if( error > 0) {
//       avgRot = (lRot + rRot) / 2;

//       Serial.print( "ERROR ," );
//       Serial.print( error );

//       // Serial.print( ",___LENC: ,");
//       // Serial.print( lRot );

//       // Serial.print(",___RENC: ,");
//       // Serial.print( rRot );

//       // Serial.print(",___AVGENC: ,");
//       // Serial.print( avgRot );

//       Serial.print( " ,L_ERR ," );
//       Serial.print( lRot - avgRot );

//       // IF you go too fast, slow down, else keep it up.
//       //Serial.print( ",___LSPD: " );
//       if( lRot - avgRot > 0 ) leftWheelSpeed = leftWheelSpeed-1;
//       if( lRot - avgRot < 0 ) leftWheelSpeed = leftWheelSpeed+1;
//       rotateLW( leftWheelSpeed );
//       //Serial.print( leftWheelSpeed );

//       Serial.print( " ,R_ERR ," );
//       Serial.print( rRot - avgRot );

//       // IF you go too fast, slow down, else keep it up.
//       //Serial.print( "___RSPD: " );
//       if( rRot - avgRot > 0 ) rightWheelSpeed = rightWheelSpeed-1;
//       if( rRot - avgRot < 0 ) rightWheelSpeed = rightWheelSpeed+1;
//       rotateRW( rightWheelSpeed );
//       //Serial.print( rightWheelSpeed );

//       Serial.println( );
//   }else {
//     stopMoving();
//   }
// }

int getDesTotalHoles() {
  return (DES_DISTANCE * WHEEL_HOLES) / WHEEL_DIAMETER;
}

void moveForward2( int desHoles ) {
  int rem_holes = desHoles - lRot;
  

  if( rem_holes > 0 ) {
   // Serial.println( rem_holes );
    lMotor->setSpeed( 50 );
    //rMotor->setSpeed( 50 );
    lMotor->run(FORWARD);
    // rMotor->run(FORWARD);
    //Serial.println( rem_holes );

  }else {
    stopMoving();
  }
}

int readSonar() {
  // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculating the distance
  return duration * 0.034 / 2;
}

void moveSonar( int deg ) {  
  fSonar.write( deg );
}

int findPath() {
  Serial.println("Found an object.");
  Serial.println("Searching for optimal path...");

  int distances[] = {0, 0, 0};

  // Check left
  moveSonar( 10 );
  distances[0]  = readSonar();

  // Check front
  moveSonar( 90 );
  distances[1]  = readSonar();

  // Check right
  moveSonar( 170);
  distances[2]  = readSonar();

  // Return sonar back to the default position.
  moveSonar( 90 );

  int min = 999;
  int index = -1;
  for(int i=0; i < 3; i++) {
    if(distances[i] < min) {
      min = distances[i];
      index = i;
    }
  }

  Serial.print("Paths: [ ");
  Serial.print( distances[0] );
  Serial.print( distances[1] );
  Serial.print( distances[2] );
  Serial.println(" ]");


  Serial.print("Choose Path: \t");
  Serial.println( index );
      
  // Return path with maximum distance.
  return index;
}

void objectAvoidance() {
  int fDistance = readSonar();

  if( fDistance <= 15 ) {
    stopMoving();

    int path = findPath();
    
    switch( path ) {
      case 0:
        turnLeft( 100 );
        delay(2500);
        break;
      case 1:
        moveForward( 100 );
        delay(2500);
        break;
      case 2:
        turnRight( 100 );
        delay(2500);
        break;
      default:
        stopMoving();
        break;
    }
  }else {
    moveForward( 150 );
    delay(2500);
  }
}

void loop() {
  /******************************************************
   * Object avoidance algorithm.
   *
   ******************************************************/
  objectAvoidance();  
}