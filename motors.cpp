#include "motors.h"
#include "sensors.h"
#include "Arduino.h"

Servo myservoA, myservoB, myservoC, myservoD;

// Motor pin addresses
#define LEFT_MOTOR_ADDRESS 0
#define RIGHT_MOTOR_ADDRESS 1
#define SCREW_MOTOR_ADDRESS 7
#define BELT_MOTOR_ADDRESS 8

// Motor drive macros
#define NEUTRAL 1500
#define OFFSET 400
#define SPEED_1 NEUTRAL - OFFSET
#define SPEED_2 NEUTRAL + OFFSET

#define PULSE_SMOOTH 200

void motors_init(void) {
  myservoA.attach(LEFT_MOTOR_ADDRESS);
  myservoB.attach(RIGHT_MOTOR_ADDRESS);
  myservoC.attach(SCREW_MOTOR_ADDRESS);
  myservoD.attach(BELT_MOTOR_ADDRESS);
}


void get_to_speed(Servo servo, int speed) {
  int pulse_width, write_value;
  if (servo.readMicroseconds() != speed) {
    pulse_width = servo.readMicroseconds();
    write_value = (pulse_width < speed) ? pulse_width + PULSE_SMOOTH : pulse_width - PULSE_SMOOTH; // increments the speed
    write_value = ((speed > NEUTRAL) && (pulse_width > 1350) && (pulse_width < 1650)) ? 1650 : write_value; // gets rid of band where robot does not move
    write_value = ((speed < NEUTRAL) && (pulse_width > 1350) && (pulse_width < 1650)) ? 1350 : write_value; // ditto
    write_value = (write_value < SPEED_1) ? SPEED_1 : write_value; // upper and lower speed caps
    write_value = (write_value > SPEED_2) ? SPEED_2 : write_value;
    servo.writeMicroseconds(write_value);
  }
}

void load_belt(void) {
  myservoD.writeMicroseconds(SPEED_1);
}

void unload_belt(void) {
  myservoD.writeMicroseconds(SPEED_2);
}

void stop_belt(void) {
  myservoD.writeMicroseconds(NEUTRAL);
}

void forward_screw(void) {
  myservoC.writeMicroseconds(1050);
}

void reverse_screw(void) {
  myservoC.writeMicroseconds(1950);
}

void stop_screw(void) {
  myservoC.writeMicroseconds(NEUTRAL);
}

void forward(void) {
  get_to_speed(myservoA, SPEED_1);
  get_to_speed(myservoB, SPEED_1);
  myservoA.writeMicroseconds(SPEED_1);
  myservoB.writeMicroseconds(SPEED_1);
}

void backward(void) {
  get_to_speed(myservoA, SPEED_2);
  get_to_speed(myservoB, SPEED_2);
  // myservoA.writeMicroseconds(SPEED_2);
  // myservoB.writeMicroseconds(SPEED_2);
}

void turn_left(void) {
  get_to_speed(myservoA, SPEED_2);
  get_to_speed(myservoB, SPEED_1);
  // myservoA.writeMicroseconds(SPEED_2);
  // myservoB.writeMicroseconds(SPEED_1);
}

void turn_right(void) {
  get_to_speed(myservoA, SPEED_1);
  get_to_speed(myservoB, SPEED_2);
  // myservoA.writeMicroseconds(SPEED_1);
  // myservoB.writeMicroseconds(SPEED_2);
}

void hard_stop(void) {
  myservoA.writeMicroseconds(NEUTRAL);
  myservoB.writeMicroseconds(NEUTRAL);
}

void soft_stop(void) {
  get_to_speed(myservoA, NEUTRAL);
  get_to_speed(myservoB, NEUTRAL);
}