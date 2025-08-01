
/********************************************************************************
 *                               ROBOCUP Group 32                              
 *        
 *  This is a template program design with modules for 
 *  different components of the robot, and a task scheduler
 *  for controlling how frequently tasks sholud run
 *  
 * Written by: Francesco Mayone, Noah Walle, John-Luke Fenn
 ******************************************************************************/

#include <Servo.h>                  //control the DC motors
#include <Wire.h>                   //for I2C and SPI
#include <TaskScheduler.h>          //scheduler 

// Custom headers
#include "movement.h"
#include "sensors.h"
#include "motors.h"

//**********************************************************************************
// Local Definitions
//**********************************************************************************

// Task period Definitions
// ALL OF THESE VALUES WILL NEED TO BE SET TO SOMETHING USEFUL
#define COLOUR_G_READ_TASK_PERIOD           200
#define COLOUR_B_READ_TASK_PERIOD           200
#define TOF_READ_TASK_PERIOD                40
#define IMU_READ_TASK_PERIOD                40
#define LIM_READ_TASK_PERIOD                40
#define PROX_READ_TASK_PERIOD               40
#define ODOM_READ_TASK_PERIOD               40
#define FSM_TASK_PERIOD                     20
#define POS_TASK_PERIOD                     20

// Task execution amount definitions
// -1 means indefinitely
#define COLOUR_G_READ_TASK_NUM_EXECUTE     -1
#define COLOUR_B_READ_TASK_NUM_EXECUTE     -1
#define TOF_READ_TASK_NUM_EXECUTE          -1
#define IMU_READ_TASK_NUM_EXECUTE          -1
#define LIM_READ_TASK_NUM_EXECUTE          -1
#define PROX_READ_TASK_NUM_EXECUTE         -1
#define ODOM_READ_TASK_NUM_EXECUTE         -1
#define FSM_TASK_NUM_EXECUTE               -1
#define POS_TASK_NUM_EXECUTE               -1

// Pin definitions
#define IO_POWER 49

// Serial definitions
#define BAUD_RATE 115200


//**********************************************************************************
// Task Scheduler and Tasks
//**********************************************************************************

/* The first value is the period, second is how many times it executes
   (-1 means indefinitely), third one is the callback function */

// Tasks for reading sensors 
Task tRead_green(COLOUR_G_READ_TASK_PERIOD,     COLOUR_G_READ_TASK_NUM_EXECUTE,    &read_green);
Task tRead_blue(COLOUR_B_READ_TASK_PERIOD,      COLOUR_B_READ_TASK_NUM_EXECUTE,    &read_blue);
Task tRead_tof(TOF_READ_TASK_PERIOD,            TOF_READ_TASK_NUM_EXECUTE,         &read_tof);
Task tRead_IMU(IMU_READ_TASK_PERIOD,            IMU_READ_TASK_NUM_EXECUTE,         &read_IMU);
Task tRead_prox(PROX_READ_TASK_PERIOD,          PROX_READ_TASK_NUM_EXECUTE,        &read_prox);
Task tRead_lim(LIM_READ_TASK_PERIOD,            LIM_READ_TASK_NUM_EXECUTE,         &read_limit_switch);
Task tRead_odom(ODOM_READ_TASK_PERIOD,          ODOM_READ_TASK_NUM_EXECUTE,        &read_odometry);
Task tRead_Pos(POS_TASK_PERIOD,                POS_TASK_NUM_EXECUTE,               &calculate_position);

// Task to do the finite state machine
Task tFSM(FSM_TASK_PERIOD,           FSM_TASK_NUM_EXECUTE,      &finite_state_machine);

// Tasks to check the 'watchdog' timer (These will need to be added in)
//Task tCheck_watchdog(CHECK_WATCHDOG_TASK_PERIOD, CHECK_WATCHDOG_TASK_NUM_EXECUTE, &check_watchdog);
//Task tVictory_dance(VICTORY_DANCE_TASK_PERIOD,   VICTORY_DANCE_TASK_NUM_EXECUTE,  &victory_dance);

Scheduler taskManager;

//**********************************************************************************
// Function Definitions
//**********************************************************************************
void pin_init();
void robot_init();
void task_init();

//**********************************************************************************
// SETUP
//**********************************************************************************
void setup() {
  Serial.begin(BAUD_RATE);
  pin_init();
  Wire.begin();
  Wire.setClock(4e5);
  Wire1.setSCL(16);
  Wire1.setSDA(17);
  Wire1.begin();
  robot_init();
  task_init();
}

//**********************************************************************************
// Initialise the pins as inputs and outputs (otherwise, they won't work) 
// Set as high or low
//**********************************************************************************
void pin_init(){
  pinMode(IO_POWER, OUTPUT);              //Pin 49 is used to enable IO power
  digitalWrite(IO_POWER, 1);              //Enable IO power on main CPU board

  // Encoder pins
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT); 
  pinMode(encoder2PinA, INPUT); 
  pinMode(encoder2PinB, INPUT); 
}

//**********************************************************************************
// Set default robot state
//**********************************************************************************
void robot_init() {
  encoder_init();
  motors_init();
  tof_init();
  IMU_init();
  colour_sensor_init();
}

//**********************************************************************************
// SCHEDULER
//**********************************************************************************
void task_init() {  
  
  // This is a class/library function. Initialise the task scheduler
  taskManager.init();     
 
  // Add tasks to the scheduler
  taskManager.addTask(tRead_green);
  taskManager.addTask(tRead_blue);
  taskManager.addTask(tRead_tof);
  taskManager.addTask(tRead_IMU);
  taskManager.addTask(tRead_prox);
  taskManager.addTask(tRead_lim);
  taskManager.addTask(tRead_odom);
  taskManager.addTask(tRead_Pos);
  taskManager.addTask(tFSM); 

  //taskManager.addTask(tCheck_watchdog);
  //taskManager.addTask(tVictory_dance);      

  //enable the tasks
  tRead_green.enable();
  tRead_blue.enable();
  tRead_tof.enable();
  tRead_IMU.enable();
  tRead_prox.enable();
  tRead_lim.enable();
  tRead_odom.enable();
  tRead_Pos.enable();
  tFSM.enable();
}



//**********************************************************************************
// MAIN LOOP
//**********************************************************************************
void loop() {
  taskManager.execute();
}
