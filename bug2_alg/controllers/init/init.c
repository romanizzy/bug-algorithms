/*
 * File:          init.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/lidar_point.h>
#include <webots/display.h>
#include <webots/motor.h>

#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define WHEEL_RADIUS 0.029

float wheel_cirum = 2*(3.14)*WHEEL_RADIUS
float encoder_unit = wheel_cirum / (2*3.14)

gps
compass
/*right_motor  // defined by right wheel motor
left_motor
right_sensor // this is the wheel sensor
left_sensor  // defined as right wheel sensor
sonar_2      // this is a distance sensor
sonar_3      // defined by s#
sonar_5
inf_1        // this is a distance sensor
inf_2        // defined by so#
inf_3
inf_4
inf_5
inf_6
inf_7*/
gps_values
compass_val
sonar_val
position_value
ir_value
robot_position // this will be an [0.0, 0.0, 0.0] array
robot_omega // this will be an [0.0, 0.0, 0.0] array

static WbDeviceTag right_motor, left_motor;
static WbDeviceTag right_sensor, left_sensor;
static WbDeviceTag sonar1, sonar2, sonar3;

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
init_robot(TIME_STEP) {
  
  //initialize robot
  wb_robot_init();

  //define robot motors
  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  //set position for robot motors
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_position(left_motor, INFINITY);

  //set velocity for robot motors
  wb_motor_set_velocity(right_motor, 0.0);
  wb_motor_set_velocity(left_motor, 0.0);

  //define distance sensors(sonar)
  sonar1 = wb_robot_get_device("s2");
  sonar2 = wb_robot_get_device("s3");
  sonar3 = wb_robot_get_device("s5");

  //enable distance sensors(sonar)
  wb_distance_sensor_enable(sonar1, TIME_STEP);
  wb_distance_sensor_enable(sonar2, TIME_STEP);
  wb_distance_sensor_enable(sonar3, TIME_STEP);
  
  //define position sensors
  right_sensor = wb_robot_get_device("right wheel sensor");
  left_sensor = wb_robot_get_device("left wheel sensor");

  //enable position sensors
  wb_position_sensor_enable(right_sensor, TIME_STEP);
  wb_position_sensor_enable(left_sensor, TIME_STEP);

  //define infared sensors
  inf_1 = wb_position_get_device("so0");
  inf_2 = wb_position_get_device("so1");
  inf_3 = wb_position_get_device("so2");
  inf_4 = wb_position_get_device("so3");
  inf_5 = wb_position_get_device("so4");
  inf_6 = wb_position_get_device("so5");
  inf_7 = wb_position_get_device("so6");
  inf_8 = wb_position_get_device("so7");

  //enable infared sensors
  wb_distance_sensor_enable(inf_1, TIME_STEP);
  wb_distance_sensor_enable(inf_2, TIME_STEP);
  wb_distance_sensor_enable(inf_3, TIME_STEP);
  wb_distance_sensor_enable(inf_4, TIME_STEP);
  wb_distance_sensor_enable(inf_5, TIME_STEP);
  wb_distance_sensor_enable(inf_6, TIME_STEP);
  wb_distance_sensor_enable(inf_7, TIME_STEP);
  wb_distance_sensor_enable(inf_8, TIME_STEP);

  //wheel cirum again
  //encoder unit

  // define and enable gps

  //define and enable compass

}

void read_sensors_values(){
  // read GPS values
  // read compass and rotate arrow accordingly
  // read sonar values
  // read position sensor values
  // read infrared sensors values

  // store all values in thier respective global variable
}

void init_robot_state(){

}

void update_robot_state(){

}

void update_motor_speed(){

}

void get_bearing_in_degrees(){
  
}