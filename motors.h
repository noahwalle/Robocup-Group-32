//************************************
//         motors.h    
//************************************

#ifndef MOTORS_H_
#define MOTORS_H_

#include <Servo.h>

// Initialises all motor pins
void motors_init(void);

void get_to_speed(Servo servo, int speed);

// Collection belt drive functions
void load_belt(void);
void unload_belt(void);
void stop_belt(void);

// Lead screw drive functions
void forward_screw(void);
void reverse_screw(void);
void stop_screw(void);

// Robot drive functions
void forward(void);
void backward(void);
void turn_left(void);
void turn_right(void);
void hard_stop(void);
void soft_stop(void);

#endif /* MOTORS_H_ */