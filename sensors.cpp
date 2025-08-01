//************************************
//         sensors.cpp       
//************************************

 // This file contains functions used to read and average
 // the sensors.

// InfraRed Includes
#include "sensors.h"
#include "Arduino.h"

// TOFs Includes
#include <Wire.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <SparkFunSX1509.h>

// IMU Includes
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// Colour sensor
#include <Adafruit_TCS34725.h>


/********************************************************************************
 *                               CONSTANTS
 *******************************************************************************/
// TOFs constants
// The number of sensors in the system.
const uint8_t sensorCountL0 = 3;
const uint8_t sensorCountL1 = 4;

// Buffers for averaging (??storing??) sensor values
int sensorValues[sensorCountL0 + sensorCountL1] = {0};

// The Arduino pin connected to the XSHUT pin of each sensor.
const uint8_t xshutPinsL1[8] = {0,1,2,5}; //long rangers - L1
const uint8_t xshutPinsL0[8] = {3,4,6}; //short rangers - L0

SX1509 io;
VL53L0X sensorsL0[sensorCountL0];
VL53L1X sensorsL1[sensorCountL1];

// IMU Constants
// Set the delay between fresh samples
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Encoder constants
#define PULSE_PER_ROTATION 663  // from datasheet 663 encoder pulses per revolution
#define WHEEL_CIRCUMF 0.0038 * 2 * 3.14 // measured with a ruler :)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                    id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire1);

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

sensors_event_t linearAccelData, magnetometerData, accelerometerData, orientationData, angVelocityData, gravityData;

volatile int encoderPos1 = 0;
int lastReportedPos1 = 1;
volatile int encoderPos2 = 0;
int lastReportedPos2 = 1;

boolean A_set1 = false;
boolean B_set1 = false;
boolean A_set2 = false;
boolean B_set2 = false;

bool next_switch_state;
int lim_count = 0;
int count_threshold_lim = 1;
bool current_switch_state = get_limit_switch();

bool next_prox_state;
int prox_count = 0;
int count_threshold_prox = 3;
bool current_prox_state = get_prox();

uint16_t clear, red, green, blue;
float r, g, b;
uint32_t sum;

/********************************************************************************
 *                               FUNCTIONS
 *******************************************************************************/
void encoder_init(void) 
{
  attachInterrupt(digitalPinToInterrupt(2), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), doEncoder2A, CHANGE);

  encoderPos1 = 0;
  lastReportedPos1 = 1;
  encoderPos2 = 0;
  lastReportedPos2 = 1;
}

void colour_sensor_init(void) {
  if (tcs.begin(41, &Wire1)) {
    Serial.println("Found Colour sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1); // halt!
  }
}

void tof_init(void) {
  io.begin(SX1509_ADDRESS);
  // Disable/reset all sensors by driving their XSHUT pins low.
  for (uint8_t i = 0; i < sensorCountL0; i++) {
    io.pinMode(xshutPinsL0[i], OUTPUT);
    io.digitalWrite(xshutPinsL0[i], LOW);
  }
  for (uint8_t i = 0; i < sensorCountL1; i++) {
    io.pinMode(xshutPinsL1[i], OUTPUT);
    io.digitalWrite(xshutPinsL1[i], LOW);
  }

  // L0 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCountL0; i++)
  {
    io.digitalWrite(xshutPinsL0[i], HIGH);
    delay(10);
    sensorsL0[i].setTimeout(500);
    if (!sensorsL0[i].init()) {
      Serial.print("Failed to detect and initialize sensor L0 ");
      Serial.println(i);
      while (1);
    }
    if (i == 0) {
      sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i);
    } else {
      sensorsL0[i].setAddress(VL53L0X_ADDRESS_START + i + 1);
    }
    sensorsL0[i].startContinuous(50);   
  }

  // L1 Enable, initialize, and start each sensor, one by one.
  for (uint8_t i = 0; i < sensorCountL1; i++)
  {
    io.digitalWrite(xshutPinsL1[i], HIGH);
    delay(10);
    sensorsL1[i].setTimeout(500);
    if (!sensorsL1[i].init()) {
      Serial.print("Failed to detect and initialize sensor L1 ");
      Serial.println(i);
      while (1);
    }
    sensorsL1[i].setAddress(VL53L1X_ADDRESS_START + i);
    sensorsL1[i].startContinuous(50);
  }
}

static float prev_encoder1, prev_encoder2;
static float g_odometry;

void read_odometry(void) {
  float encoder_position1 = encoderPos1;
  float encoder_position2 = encoderPos2;
  float delta_encoder = (encoder_position1 - prev_encoder1) + (encoder_position2 - prev_encoder2) / 2;
  g_odometry += 85 * delta_encoder / 1000;
  prev_encoder1 = encoder_position1;
  prev_encoder2 = encoder_position2;
}

void read_green(void) {
  tcs.setInterrupt(false);
  delay(60);
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);
  sum = clear;
  g = green; g /= sum; g *= 256;
  // Serial.println((int)g, HEX);
}

bool get_green(void) {
  return g > 100;
}

void read_blue(void) {
  tcs.setInterrupt(false);
  delay(60);
  tcs.getRawData(&red, &green, &blue, &clear);
  tcs.setInterrupt(true);
  sum = clear;
  b = blue; b /= sum; b *= 256;
  // Serial.println((int)b, HEX);
}

bool get_blue(void) {
  return b > 100;
}

void read_tof(void) {
  for (uint8_t i = 0; i < sensorCountL0; i++) {
    sensorValues[i] = sensorsL0[i].readRangeContinuousMillimeters();
  }
  for (uint8_t i = 0; i < sensorCountL1; i++) {
    sensorValues[i + sensorCountL0] = sensorsL1[i].readRangeContinuousMillimeters();
  }
}

int* get_tof(void) {
  return sensorValues;
}

void IMU_init(void)
{
  if (!bno.begin()) {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1e3);
}

bool get_prox(void)
{
  return analogRead(A9) < 150;
}

void read_IMU(void) {
  // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
}

float get_heading(void) {
  read_IMU();
  // Serial.println(orientationData.orientation.x, 4);
  float angle = orientationData.orientation.x;
  return angle;
}

void print_encoder(void)
{ 
  if ((lastReportedPos1 != encoderPos1)||(lastReportedPos2 != encoderPos2)) 
  {
    // Serial.print("Index:");
    // Serial.print(encoderPos1, DEC);
    // Serial.print(":");
    // Serial.print(encoderPos2, DEC);
    // Serial.println();
    lastReportedPos1 = encoderPos1;
    lastReportedPos2 = encoderPos2;
  }
}

void doEncoder1A(void) {
  A_set1 = digitalRead(encoder1PinA) == HIGH;
  encoderPos1 += (A_set1 != B_set1) ? +1 : -1;
  B_set1 = digitalRead(encoder1PinB) == HIGH;
  encoderPos1 += (A_set1 == B_set1) ? +1 : -1;
}

void doEncoder2A(void) {
  A_set2 = digitalRead(encoder2PinA) == HIGH;
  encoderPos2 += (A_set2 != B_set2) ? +1 : -1;
  B_set2 = digitalRead(encoder2PinB) == HIGH;
  encoderPos2 += (A_set2 == B_set2) ? +1 : -1;
}

bool get_limit_switch(void) {
  return analogRead(7);
}

void read_limit_switch(void) {
  next_switch_state = get_limit_switch();
  if (next_switch_state != current_switch_state) {
    lim_count++;
  } else {
    lim_count = 0;
  }
  if (lim_count > count_threshold_lim) {
    current_switch_state = next_switch_state;
  }
}

bool process_limit_switch(void) {
  return current_switch_state;
}

void read_prox(void) {
  next_prox_state = get_prox();
  if (next_prox_state != current_prox_state) {
    prox_count++;
  } else {
    prox_count = 0;
  }
  if (prox_count > count_threshold_prox) {
    current_prox_state = next_prox_state;
  }
}

bool process_prox(void) {
  return current_prox_state;
}

int global_position[2];
float last_heading = 0;
float last_odom = 0;

void calculate_position(void) {
  float current_heading = get_heading();
  global_position[0] = (g_odometry - last_odom) * cos(last_heading);
  global_position[1] = (g_odometry - last_odom) * sin(last_heading);
  last_heading = current_heading;
  last_odom = g_odometry;
}

float calculate_heading(void) {
  return atan2f(-global_position[1], -global_position[0]);
}