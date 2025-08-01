//************************************
//         sensors.h     
//************************************

#ifndef SENSORS_H_
#define SENSORS_H_

// Local definitions
#define VL53L0X_ADDRESS_START 0x35
#define VL53L1X_ADDRESS_START 0x26
#define SX1509_ADDRESS 0x3F
#define SX1509_AIO6 6

enum PinAssignments {
  encoder1PinA = 2,
  encoder1PinB = 3,
  encoder2PinA = 4,
  encoder2PinB = 5,
};

// Sensor initialisation
void colour_sensor_init(void);
void tof_init(void);
void IMU_init(void);
void encoder_init(void);

int get_encoder_one(void);
int get_encoder_two(void);

// Colour sensor functions
bool get_green(void);
bool get_blue(void);
void read_green(void);
void read_blue(void);

void read_tof(void);
int *get_tof(void);

bool get_prox(void);
void read_prox(void);
bool process_prox(void);

void read_IMU(void);
float get_heading(void);
void print_encoder(void);
void doEncoder1A(void);
void doEncoder2A(void);

bool get_limit_switch(void);
void read_limit_switch(void);
bool process_limit_switch(void);

void read_odometry(void);
void calculate_position(void);
float calculate_heading(void);

#endif /* SENSORS_H_ */
