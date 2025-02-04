#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  127
#define SERVOMAX  492
#define SERVO_FREQ 50

#define M1 8  // Top Left Hip
#define M2 0  // Top Right Hip
#define M3 2  // Bottom Right Hip
#define M4 10 // Bottom Left Hip

#define m1 9  // Top Left Knee
#define m2 1  // Top Right Knee
#define m3 3  // Bottom Right Knee
#define m4 11 // Bottom Left Knee

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Variables for triggers
int tricker_left = 0;
int tricker_right = 0;
const int trigger_threshold = 10;

unsigned long last_command_time = 0;
const unsigned long command_timeout = 1000; // 3 seconds timeout

void set_servo_angle(uint8_t servo, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servo, 0, pulse);
}

void rest() {
  set_servo_angle(M1, 125);
  set_servo_angle(M2, 50);
  set_servo_angle(M3, 140);
  set_servo_angle(M4, 45);
  set_servo_angle(m1, 60);
  set_servo_angle(m2, 120);
  set_servo_angle(m3, 60);
  set_servo_angle(m4, 120);
  Serial.println("Robot is in resting position.");
}

void left() {
  // Movement sequence for left
  set_servo_angle(m1, 45); 
  set_servo_angle(m4, 135);
  delay(300);
  set_servo_angle(m2, 55); 
  set_servo_angle(m3, 125); 
  delay(300);
  set_servo_angle(m2, 155); 
  set_servo_angle(m3, 25); 
  delay(3000);
  set_servo_angle(m1, 90); 
  set_servo_angle(m4, 90);
  set_servo_angle(m2, 90); 
  set_servo_angle(m3, 90);
  delay(300);
  set_servo_angle(m1, 125); 
  set_servo_angle(m4, 55);
  delay(300);
  set_servo_angle(m1, 90); 
  set_servo_angle(m4, 90);


}

void right() {
  // Movement sequence for right
  set_servo_angle(m2, 125); 
  set_servo_angle(m3, 55); 
  delay(300);
  set_servo_angle(m1, 125); 
  set_servo_angle(m4, 55);
  delay(300);
  set_servo_angle(m1, 45); 
  set_servo_angle(m4, 135);
  delay(3000);
  set_servo_angle(m1, 90); 
  set_servo_angle(m4, 90);
  set_servo_angle(m2, 90); 
  set_servo_angle(m3, 90);
  delay(100);
  set_servo_angle(m2, 55); 
  set_servo_angle(m3, 125); 
  delay(300);
  set_servo_angle(m2, 90); 
  set_servo_angle(m3, 90); 
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  rest();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Clear the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }

    last_command_time = millis(); // Reset the command timer

    if (input == "c") {
      rest();
      tricker_left = 0;
      tricker_right = 0;
      Serial.println("Center");

    } else if (input == "l") {
      tricker_left++;
      Serial.println("Leaning left: " + String(tricker_left));
      if (tricker_left >= trigger_threshold) {
        left();
        delay(3000);
        rest();
        tricker_left = 0; // Reset after action
      }

    } else if (input == "r") {
      tricker_right++;
      Serial.println("Leaning right: " + String(tricker_right));
      if (tricker_right >= trigger_threshold) {
        right();
        delay(3000);
        rest();
        tricker_right = 0; // Reset after action
      }

    } else {
      Serial.println("Unknown command!");
    }
  }

  // Check for inactivity
  if (millis() - last_command_time >= command_timeout) {
    rest(); // Reset to rest position
  }
}
