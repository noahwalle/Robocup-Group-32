/************************************
 *        movement.cpp       *
 *************************************/

 /* This is for movement control  */

#include "sensors.h"
#include "motors.h"
#include "return_to_base.h"
#include "weight_collection.h"
#include "Arduino.h"
#include "debug.h"
#include "movement.h"

#define VELOCITY_THRESHOLD 0.5
#define WEIGHT_THRESHOLD 200
#define WALL_FIND_THRESHOLD 250
#define WALL_FOLLOW_THRESHOLD 350
#define COLOUR_THRESHOLD 100

int count = 100;

int* tof_sensor_vals;
int w_left, w_right, nav_right, nav_left, nav_straight_left, nav_straight_right, nav_straight_middle;
static bool belt_flag = true;

State_t g_current_state = {SEEKING};
Location_t g_current_location = {HOME};
Colour_t g_current_colour;
Colour_t g_base_colour;

Turn_Condition_t g_turn_condition = FORWARD;
Wall_Following_t g_follow_direction;

State_t get_current_state(void) {
  return g_current_state;
}

Location_t get_current_location(void) {
  return g_current_location;
}

Colour_t get_current_colour(void) {
  return g_current_colour;
}

Colour_t get_base_colour(void) {
  return g_base_colour;
}

bool col_init_flag = false;

void set_base_colour(void) {
  if (!col_init_flag) {
    if (get_green()) {
      g_current_colour = GREEN;
    } else if (get_blue()) {
      g_current_colour = BLUE;
    }
    g_base_colour = (g_current_colour == GREEN) ? GREEN : BLUE;
    g_follow_direction = ((g_base_colour == GREEN) ? FOLLOW_LEFT : FOLLOW_RIGHT);
    col_init_flag = true;
  }
}

void update_colour(void) {
  if (get_green()) {
    g_current_colour = GREEN;
  } else if (get_blue()) {
    g_current_colour = BLUE;
  } else {
    g_current_colour = BLACK;
  }
}

Wall_Following_t get_follow_dir(void) {
  return g_follow_direction;
}

void update_location(void) {
  if ((get_green() && (g_base_colour == GREEN)) || (get_blue() && (g_base_colour == BLUE))) {
    g_current_location = HOME;
  } else if ((get_green() && (g_base_colour == BLUE)) || (get_blue() && (g_base_colour == GREEN))) {
    g_current_location = ENEMY;
  } else {
    g_current_location = ARENA;
  }
}

void finite_state_machine(void)
{
  // print_debug_data();
  set_base_colour();
  delay(1000);
  update_colour();
  update_location();
  debug_colour();
  if ((get_weight_count() >= 2) && (g_current_location == HOME)) {
    g_current_state = RETURNING;
    switch(g_follow_direction) {
      case FOLLOW_LEFT:
        turn_right();
        delay(1e3);
        g_follow_direction = FOLLOW_RIGHT;
        break;
      case FOLLOW_RIGHT:
        turn_left();
        delay(1e3);
        g_follow_direction = FOLLOW_LEFT;
        break;
    }
  } else if (get_prox()) {
    g_current_state = COLLECTING;
  } else if (get_weight_count() < 2) {
    if (g_current_state == COLLECTING) {
      increment_weight_count();
    } else if (g_current_state == RETURNING) {
      reset_weight_count();
    }
    g_current_state = SEEKING;
  }

  switch (g_current_state) {
    case SEEKING:
      seek_weights();
      break;
    case COLLECTING:
      collect_weight();
      delay(1e3);
      break;
    case RETURNING:
      drop_weights_at_base();
      break;
    }
}

void seek_weights(void) {
  tof_sensor_vals = get_tof();
  w_left = tof_sensor_vals[0];
  w_right = tof_sensor_vals[1];
  nav_left = tof_sensor_vals[3];
  nav_right = tof_sensor_vals[4];
  nav_straight_left = tof_sensor_vals[5];
  nav_straight_middle = tof_sensor_vals[2];
  nav_straight_right = tof_sensor_vals[6];

  // Conditions for detecting obstacles, walls, or weights in 3 directions
  bool r_stuff = (nav_right < WALL_FIND_THRESHOLD);
  bool l_stuff = (nav_left < WALL_FIND_THRESHOLD);
  bool f_stuff = (nav_straight_middle < WALL_FIND_THRESHOLD + 150);
  bool l_weight = (w_left < WEIGHT_THRESHOLD) && (nav_straight_left > WEIGHT_THRESHOLD);
  bool r_weight = (w_right < WEIGHT_THRESHOLD) && (nav_straight_right > WEIGHT_THRESHOLD);

  // float accel = 0;
  // static float accel_prev = 0;
  // read_IMU();
  
  // bool isCollision = abs(accel - accel_prev) > 5;
  // Serial.println(accel_prev - accel);

  // uint32_t currentTime = millis();
  // uint32_t prevTime = 0;
  // uint32_t dt = currentTime - prevTime;

  // uint32_t velocity = 0;
  // velocity += accel * dt;

  // prevTime = currentTime;

  // if (velocity/1000 < VELOCITY_THRESHOLD) {
  //   count++;
  // }
  // if (velocity > VELOCITY_THRESHOLD) {
  //   count = 0;
  // }

  // bool stall = count >= 10;  

  // Conditions for driving left, right, straight, and backwards
  g_turn_condition = !f_stuff ? FORWARD : g_turn_condition;
  g_turn_condition = (r_stuff && !l_stuff) || (!l_stuff && !r_weight && l_weight) || ((g_follow_direction == FOLLOW_LEFT) && (nav_left > WALL_FOLLOW_THRESHOLD)) ? LEFT : g_turn_condition;
  g_turn_condition = (l_stuff && !r_stuff) || (!(r_stuff || l_weight) && r_weight) || ((g_follow_direction == FOLLOW_RIGHT) && (nav_right > WALL_FOLLOW_THRESHOLD)) ? RIGHT : g_turn_condition;
  g_turn_condition = r_stuff && l_stuff && f_stuff ? BACKWARD : g_turn_condition;
  // g_turn_condition = f_stuff && !(r_stuff || l_stuff) ? RAND : g_turn_condition;

  switch (g_turn_condition) {
    case FORWARD:
      forward();
      break;
    case LEFT:
      turn_left();
      break;
    case RIGHT:
      turn_right();
      break;
    case BACKWARD:
      backward();
      delay(1e3);
      turn_right(); // should be turning wall-following direction
      delay(2e3);
      break;
    case RAND:
      turn_left();
      break;
  }

  if (!get_limit_switch()) {
    stop_screw();
  } else {
    reverse_screw();
  }

  if ((get_weight_count() == 1) && belt_flag) {
    load_belt();
    delay(1500);
    stop_belt();
    belt_flag = false;
  }
}