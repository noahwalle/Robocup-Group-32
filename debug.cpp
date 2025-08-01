/************************************
 *        movement.cpp       *
 *************************************/

 /* This is for debugging and printing data
 to the serial port  */

#include "debug.h"
#include "movement.h"
#include "sensors.h"
#include "weight_collection.h"
#include "Arduino.h"

void print_debug_data(void) {
  Serial.print("State: ");
  switch (get_current_state()) {
    case SEEKING:
      Serial.print("Seeking");
      break;
    case COLLECTING:
      Serial.print("Collecting");
      break;
    case RETURNING:
      Serial.print("Returning");
      break;
  }

  Serial.print("   |   ");

  Serial.print("Location: ");
  switch (get_current_location()) {
    case HOME:
      Serial.print("Home Base");
      break;
    case ENEMY:
      Serial.print("Enemy Base");
      break;
    case ARENA:
      Serial.print("Arena");
      break;
    case UNKNOWN:
      Serial.print("Unknown");
      break;
  }

  Serial.print("   |   ");

  Serial.print("Follow Direction: ");
  switch (get_follow_dir()) {
    case FOLLOW_LEFT:
      Serial.print("Follow left");
      break;
    case FOLLOW_RIGHT:
      Serial.print("Follow right");
      break;
  }
  Serial.print("   |   ");

  Serial.print("Weight count: ");
  Serial.print(get_weight_count());

  Serial.print("   |   ");

  Serial.print("Limit switch: ");
  switch (process_limit_switch()) {
    case 1:
      Serial.print("Open");
      break;
    case 0:
      Serial.print("Closed");
      break;
  }

  Serial.print("   |   ");
  Serial.print("Inductive Proximity: ");
  Serial.print(get_prox());
  Serial.print(process_prox());

  Serial.println();
}

void debug_colour(void) {
  Serial.print("Green Channel: ");
  Serial.print(get_green());
  Serial.print("   |   ");
  Serial.print("Blue Channel: ");
  Serial.print(get_blue());
  Serial.print("   |   ");

  Serial.print("Location: ");
  switch (get_current_location()) {
    case HOME:
      Serial.print("Home Base");
      break;
    case ENEMY:
      Serial.print("Enemy Base");
      break;
    case ARENA:
      Serial.print("Arena");
      break;
    case UNKNOWN:
      Serial.print("Unknown");
      break;
  }

  Serial.print("   |   ");

  Serial.print("Follow Direction: ");
  switch (get_follow_dir()) {
    case FOLLOW_LEFT:
      Serial.print("Follow left");
      break;
    case FOLLOW_RIGHT:
      Serial.print("Follow right");
      break;
  }
  Serial.print("   |   ");

  Serial.print("Over Colour: ");
  switch (get_current_colour()) {
    case GREEN:
      Serial.print("GREEN");
      break;
    case BLUE:
      Serial.print("BLUE");
      break;
    case BLACK:
      Serial.print("BLACK");
      break;
  }

  Serial.print("   |   ");

  Serial.print("Base Colour: ");
  switch (get_base_colour()) {
    case GREEN:
      Serial.print("GREEN");
      break;
    case BLUE:
      Serial.print("BLUE");
      break;
    case BLACK:
      Serial.print("BLACK");
      break;
  }
  Serial.println();
}

void debug_tofs(void) {
  int w_left, w_right, nav_left, nav_right, nav_straight_left, nav_straight_right, nav_straight_middle;
  int* tof_values = get_tof();
  w_left = tof_values[0];
  w_right = tof_values[1];
  nav_straight_middle = tof_values[2];
  nav_left = tof_values[3];
  nav_right = tof_values[4];
  nav_straight_left = tof_values[5];
  nav_straight_right = tof_values[6];

  Serial.print(w_left);
  Serial.print("   |   ");
  Serial.print(w_right);
  Serial.print("   |   ");
  Serial.print(nav_left);
  Serial.print("   |   ");
  Serial.print(nav_right);
  Serial.print("   |   ");
  Serial.print(nav_straight_left);
  Serial.print("   |   ");
  Serial.print(nav_straight_right);
  Serial.print("   |   ");
  Serial.print(nav_straight_middle);
  Serial.println();
}