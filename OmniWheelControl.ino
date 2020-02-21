#include <math.h>
#include <WickedMotorShield.h>
Wicked_DCMotor m1(M1);
Wicked_DCMotor m2(M2);
Wicked_DCMotor m3(M3);

#define RC1 4 //Radio control inputs
#define RC2 8 //Let's avoid magic magic numbers

long ch1;
long ch2;
bool breaked = false;

int mode = 1;
float s = 0; // time in seconds, updated in loop

void setup() {
  Serial.begin(115200);
  Serial.print("<3");
  pinMode(RC1, INPUT);
  pinMode(RC2, INPUT);
  setBreaksOff();
}


void loop(){
  // Time in seconds
  s = (float) millis() / 1000.0f;
  
  // Read Remote Control
  ch1 = pulseIn(4, HIGH, 25000);
  ch2 = pulseIn(8, HIGH, 25000);
  

  // If the remote is acting weird or is turned off, the value of ch1 is 0
  if (abs(ch1) < 1) {
    Serial.println("REMTOTE ERROR");
    // then breai the motors.
    if (!breaked) setBreaksOn();
  } else {
    // start moving again if the breaks were on:
    if (breaked) setBreaksOff();

    // This centers the values
    ch1 = ch1 - 1500;
    ch2 = ch2 - 1550;
  
    int vy =  map(ch1, -460, 440, -1000, 1000); // FORWARD / BACKWARDS
    int vx =  map(ch2, -460, 420, 1000, -1000); // LEFT / RIGHT

    // threshold the input, so the robot is still when it's just in the center...
    if (abs(vy) < 300) vy = 0;
    if (abs(vx) < 300) vx = 0;
    
    Serial.print(vx);
    Serial.print("\t");
    Serial.print(vy);
    Serial.print("\t");
    if (mode == 1) {
      behaviorSmooth(vx, vy);
      Serial.println("SMOOTH");
    } else if (mode == 2) {
      behaviorErratic(vx, vy);
      Serial.println("ERRATIC");
    } else {
      behaviorNormal(vx, vy);
      Serial.println("NORMAL");
    }
  }
}


// Sine-wave motion style 
void behaviorSmooth(int vx, int vy) {
  // the higher, the faster it's switching from left to right
  float phasesOverTime = 1.5; 
  
  // How strong are the curves:
  float amplitudeOfRotation = 300; 

  // Combine for angular motion
  float angularMotion = amplitudeOfRotation*sin(s*phasesOverTime);
  
  // But if we're driving backwards (or stand) set angularMotion to 0;
  if (vy <= 0) angularMotion = 0;
  
  // Modulate angular motion over time
  setMotors(vy, 0, vx + angularMotion);
}


// Jerky motion style
void behaviorErratic(int vx, int vy) {
  // the higher, the more often it's switching the back/forth
  float phasesOverTime = 3; 
  
  float magnitudeMotion = 300+300*sin(s = phasesOverTime);
  if (vy <= 0) magnitudeMotion = vy;

  if (vy > 0 && magnitudeMotion < 100) magnitudeMotion = -300; 
  if (vy > 0 && magnitudeMotion > 300) magnitudeMotion = 800; 
  
  setMotors(magnitudeMotion, 0, vx);
}

// Linear motion style
void behaviorNormal(int vx, int vy) {
  setMotors(vx, 0, vy);
}


/* Parameters:
 *  magnitude: how fast to move in the direction (a value between 0 and 1000)
 *  dir: angle of the direction (in degrees) relative to the robot orientation.
 *  rot: angular motion (a value between 0 and 1000)
 *  
 */
void setMotors(float magnitude, float dir, float angularMotion) {
    // Normalize the magnitude such that the total speed is the same...
    // ...not sure if this is perfect:
    float correctedMagnitude = magnitude - abs(angularMotion);
    
    // Formula from http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.99.1083&rep=rep1&type=pdf
    float w1 = correctedMagnitude * sin(rad(  0 - dir)) + angularMotion;
    float w2 = correctedMagnitude * sin(rad(120 - dir)) + angularMotion;
    float w3 = correctedMagnitude * sin(rad(240 - dir)) + angularMotion;
    // 0 / 120 / 240 define the frame of reference of the robot.
    // i.e. setting dir to 0 means that motor 1 won't turn / we're turning in direction of motor 1.

    m1.setDirection(w1 < 0 ? DIR_CCW : DIR_CW); // If w0 is smaller than 0, motor goes CCW=counterclockwise..
    m2.setDirection(w2 < 0 ? DIR_CCW : DIR_CW); // For w1 we changed the sign to get it to rotate in the correct direction
    m3.setDirection(w3 < 0 ? DIR_CCW : DIR_CW); // ...

    // Map the computed motor speed to the control range.
    // i.e. a value of 1000 gets mapped to the maximum motor speed of 255.
    byte w1_speed = (byte) map(abs(w1), 0, 1000, 0, 255);
    byte w2_speed = (byte) map(abs(w2), 0, 1000, 0, 255);
    byte w3_speed = (byte) map(abs(w3), 0, 1000, 0, 255);
    
    m1.setSpeed(w1_speed);
    m2.setSpeed(w2_speed);
    m3.setSpeed(w3_speed);

   
}


// Enable breaks (stop)
void setBreaksOn() {
  m1.setSpeed(0);
  m2.setSpeed(0);
  m3.setSpeed(0);
  //m1.setBrake(BRAKE_ON);
  //m2.setBrake(BRAKE_ON);
  //m3.setBrake(BRAKE_ON);
  breaked = true;
}


// Enable breaks (allow driving)
void setBreaksOff() {
  m1.setBrake(BRAKE_OFF);
  m2.setBrake(BRAKE_OFF);
  m3.setBrake(BRAKE_OFF);
  breaked = false;
}

// Convert angles to radians;
float rad(float ang) {
  return ang / 57.29578;
}
