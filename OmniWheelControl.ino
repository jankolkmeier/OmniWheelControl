#include <math.h>
#include <WickedMotorShield.h>
Wicked_DCMotor m1(M1);
Wicked_DCMotor m2(M2);
Wicked_DCMotor m3(M3);

#define RC1 4 //Radio control inputs
#define RC2 8 //Let's avoid magic magic numbers

long ch1;
long ch2;

void setup() {
  Serial.begin(115200);
  Serial.print("Hello World!");

  pinMode(RC1, INPUT);
  pinMode(RC2, INPUT);
  
  m1.setBrake(BRAKE_OFF);
  m2.setBrake(BRAKE_OFF);
  m3.setBrake(BRAKE_OFF);
}


void loop(){
  // Time in seconds
  float s = (float) millis() / 1000.0f;

  // Modulate angular motion over time
  setMotors(300, 0, 100*sin(s*2));

  // Modulate speed over time (driving straight)
  //setMotors(550 + 450*sin(t*5), 0, 0);
}


/* Parameters:
 *  magnitude: how fast to move in the direction (a value between 0 and 1000)
 *  dir: angle of the direction (in degrees) relative to the robot orientation.
 *  rot: angular motion (a value between 0 and 1000)
 *  
 *  !!! TODO: right now, the sum of rot and magnitude should not be bigger than 1000.
 */
void setMotors(float magnitude, float dir, float angularMotion) {

    // TODO: if (rot + magnitude > 1000) normalize/scale the two?
    
    // Formula from http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.99.1083&rep=rep1&type=pdf
    float w1 = magnitude * sin(rad(  0 - dir)) + angularMotion;
    float w2 = magnitude * sin(rad(120 - dir)) + angularMotion;
    float w3 = magnitude * sin(rad(240 - dir)) + angularMotion;
    // 0 / 120 / 240 define the frame of reference of the robot.
    // i.e. setting dir to 0 means that motor 1 won't turn / we're turning in direction of motor 1.

    m1.setDirection(w1 < 0 ? DIR_CCW : DIR_CW); // If w0 is smaller than 0, motor goes CCW=counterclockwise..
    m2.setDirection(w2 > 0 ? DIR_CCW : DIR_CW); // For w1 we changed the sign to get it to rotate in the correct direction
    m3.setDirection(w3 < 0 ? DIR_CCW : DIR_CW); // ...

    // Map the computed motor speed to the control range.
    // i.e. a value of 1000 gets mapped to the maximum motor speed of 255.
    byte w1_speed = (byte) map(abs(w1), 0, 1000, 0, 255);
    byte w2_speed = (byte) map(abs(w2), 0, 1000, 0, 255);
    byte w3_speed = (byte) map(abs(w3), 0, 1000, 0, 255);
    
    m1.setSpeed(w1_speed);
    m2.setSpeed(w2_speed);
    m3.setSpeed(w3_speed);

    // TODO: for more erratic behavior, we could use the mX.setBrake(BRAKE_ON); at times, to get harder stops
}

// Convert angles to radians;
float rad(float ang) {
  return ang / 57.29578;
}

