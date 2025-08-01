
/************************************
 *        movement.h       *
 *************************************/

 /* This header is for movement control  */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_


typedef enum {
  SEEKING = 0,
  COLLECTING,
  RETURNING
} State_t;

typedef enum {
  HOME = 0,
  ENEMY,
  ARENA,
  UNKNOWN
} Location_t;

typedef enum {
  BLUE = 0,
  GREEN,
  BLACK
} Colour_t;

typedef enum {
    FORWARD = 0,
    LEFT,
    RIGHT,
    BACKWARD,
    RAND
  } Turn_Condition_t;

typedef enum {
    FOLLOW_LEFT = 0,
    FOLLOW_RIGHT,
  } Wall_Following_t;

Wall_Following_t get_follow_dir(void);
Colour_t get_current_colour(void);
Colour_t get_base_colour(void);

void print_debug_data(void);

State_t get_current_state(void);
Location_t get_current_location(void);

// Colour sensor functions
// WARNING: Every col sensor
// read takes a 60ms delay
// So dont do this one often!
void update_colour(void);
void set_base_colour(void);
void update_location(void);

void finite_state_machine(void);
void seek_weights(void);

float calculate_base_heading(void);

#endif /* MOVEMENT_H_ */
