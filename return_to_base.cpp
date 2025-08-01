//************************************
//         return_to_base.cpp       
//************************************

 // This file contains functions used to return to and
 // detect bases

#include "return_to_base.h"
#include "motors.h"
#include "sensors.h"
#include "weight_collection.h"
#include "Arduino.h"
#include "movement.h"

#define DEADBAND 5

void drop_weights_at_base(void) {
  float heading = calculate_heading() * 180 / 3.14;
  if (get_heading() < heading) {
    turn_left;
  }
  forward();
  if (get_current_location() == HOME) {
    hard_stop();
    turn_left();
    delay(2000);
    unload_belt();
    delay(3000);
    unload_belt();
    reset_weight_count();
    stop_belt();
  }
}
