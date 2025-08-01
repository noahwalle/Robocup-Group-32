/************************************
 *        weight_collection.cpp       *
 *************************************/

 /* This is for functions and tasks for
  *  finding and collecting weights  */


#include "weight_collection.h"
#include "motors.h"
#include "sensors.h"
#include "Arduino.h"

int g_weight_counter = 0;

void collect_weight(void)
{
  hard_stop();
  forward_screw();
}

void collect_third_weight(void) {
  forward_screw();
  delay(500);
}

int get_weight_count(void) {
  return g_weight_counter;
}

void increment_weight_count(void) {
  g_weight_counter++;
}

void reset_weight_count(void) {
  g_weight_counter = 0;
}