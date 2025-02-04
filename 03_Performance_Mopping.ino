// From: https://makezine.com/article/technology/robotics/robot-quadruped-arduino-program/ to be main creep gait

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  127
#define SERVOMAX  492
#define SERVO_FREQ 50

#define M1 8  // Front Left Hip
#define M2 0  // Front Right Hip
#define M3 2  // Rear Right Hip
#define M4 10 // Rear Left Hip

#define m1 9  // Front Left Knee
#define m2 1  // Front Right Knee
#define m3 3  // Rear Right Knee
#define m4 11 // Rear Left Knee

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void set_servo_angle(uint8_t servo, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulse);
}

void initial_walking() { 
  set_servo_angle(M1, 90);  // Hips neutral
  set_servo_angle(M2, 60);
  set_servo_angle(M3, 135);
  set_servo_angle(M4, 90);

  set_servo_angle(m1, 90);  // Knees neutral
  set_servo_angle(m2, 90);
  set_servo_angle(m3, 90);
  set_servo_angle(m4, 90);
}

////////////////Creep gait////////////////////////////////////////////////////////////////////////////////////////////////////////
void move_leg(uint8_t hip, uint8_t knee, int hip_angle, int knee_angle_lift, int knee_angle_lower) {
  set_servo_angle(knee, knee_angle_lift);  // Lift knee
  delay(500);
  set_servo_angle(hip, hip_angle);    // Move hip
  delay(500);
  set_servo_angle(knee, knee_angle_lower);          // Lower knee
  delay(500);
}

void move_front(uint8_t hip, int hip_angle) {
  set_servo_angle(hip, hip_angle);    // Move hip
  delay(500);
}

void shift_body_right(int M1_offset, int M2_offset, int M3_offset, int m4_offset ) {
  set_servo_angle(M1, M1_offset);
  set_servo_angle(M2, M2_offset);
  set_servo_angle(M3, M3_offset);
  set_servo_angle(m4, m4_offset);
  delay(500);
}

void shift_body_left(int M1_offset, int M2_offset, int m3_offset, int M4_offset) {
    // Hip
  set_servo_angle(M1, M1_offset);
  set_servo_angle(M2, M2_offset);
  set_servo_angle(m3, m3_offset);
  set_servo_angle(M4, M4_offset);
  delay(500);
}

void creep_gait() {
  // Step 1: Move front-right leg forward (hip, knee, hip_angle, knee_angle_lift, knee_angle_lower) 
  //move_leg(M2, m2, 135, 170, 90);

  // Step 2: Shift body forward (M1 ,M2, M3, m4)
  shift_body_right(125, 90, 90, 125);

  // Step 3: Move rear-left leg forward (hip, knee, hip_angle, knee_angle_lift, knee_angle_lower) 
  move_leg(M4, m4, 55 , 170, 90);

  // Step 4: Move front-left leg forward (hip, knee, hip_angle, knee_angle_lift, knee_angle_lower) 
  move_leg(M1, m1, 55, 10, 90);

  // Step 5: Shift body forward (M1 M2 m3 M4)
  shift_body_left(90, 60, 55, 90);

  // Step 6: Move rear-right leg forward (hip, knee, hip_angle, knee_angle_lift, knee_angle_lower) 
  move_leg(M3, m3, 135, 10, 90);
}

//////////////Mop floor////////////////////////////////////////////////////////////////////////////////////////////////////
void position_legs_forward() {
  // Adjust the hip angles of the front legs to position them forward
  set_servo_angle(M1, 55);  // ขาหน้าซ้าย
  delay(200);
  move_leg(M2, m2, 135, 170, 90);

  // Set the knee angles to a position ready for mopping the floor
  set_servo_angle(m1, 90);  // ขาหน้าซ้าย
  set_servo_angle(m2, 90);  // ขาหน้าขวา
  delay(200);
}

void mop_with_knees(int m1_angle, int m2_angle) {
  // move hip forward
  set_servo_angle(m1, m1_angle);  // Left
  set_servo_angle(m2, m2_angle);  // right
  delay(500);

  // move hip backward
  set_servo_angle(m1, 90); // left
  set_servo_angle(m2, 90); // right
  delay(500);
}

void position_legs_backward() {
  // Adjust the hip angles of the front legs to position them forward
  set_servo_angle(M1, 90);  // ขาหน้าซ้าย 

  // Set the knee angles to a position ready for mopping the floor
  set_servo_angle(m1, 90);  // ขาหน้าซ้าย
  set_servo_angle(m2, 90);  // ขาหน้าขวา
  delay(500);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  initial_walking();
}

void loop() {
  //initial_walking();
  position_legs_forward();
  
  // Mop floor 3 times
  for (int i = 0; i < 3; i++) {  // Mop floor 5 times
    mop_with_knees(20, 160);  // m1_angle, m2_angle
  }

  position_legs_backward(); 
  creep_gait();
}
