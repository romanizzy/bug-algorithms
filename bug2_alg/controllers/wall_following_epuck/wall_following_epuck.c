/*
 * File:          wall_following_epuck.c
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
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
static WbDeviceTag backGPS, centerGPS;
static WbDeviceTag left_motor, right_motor;
const double *point1;
const double *point2;

 
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  
  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;
  static float targetX=0;
  static float targetZ=4.684;
  float initalAngle;
  
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  
  backGPS = wb_robot_get_device("frontGps"); //name is wrong, it is actually at back
  centerGPS = wb_robot_get_device("centerGps");
  wb_gps_enable(backGPS,TIME_STEP);
  wb_gps_enable(centerGPS,TIME_STEP);
  
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  wb_robot_step(TIME_STEP);
  
  WbDeviceTag prox_sensors[8];
  /*char prox_sensor_name[50];
  for (int i = 0; i < 8; ++i)
  {
    sprintf(prox_sensor_name, "ps%d", i);
    prox_sensors[i] = wb_robot_get_device(prox_sensor_name);
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
  }*/
  
  //same as: 
  
  prox_sensors[0] = wb_robot_get_device("so0");
  prox_sensors[1] = wb_robot_get_device("so1");
  prox_sensors[2] = wb_robot_get_device("so2");
  prox_sensors[3] = wb_robot_get_device("so3");
  prox_sensors[4] = wb_robot_get_device("so4");
  prox_sensors[5] = wb_robot_get_device("ps5");
  prox_sensors[6] = wb_robot_get_device("so6");
  prox_sensors[7] = wb_robot_get_device("so7");
  wb_distance_sensor_enable(prox_sensors[0],TIME_STEP);
  wb_distance_sensor_enable(prox_sensors[1],TIME_STEP);
  wb_distance_sensor_enable(prox_sensors[2],TIME_STEP);
  wb_distance_sensor_enable(prox_sensors[3],TIME_STEP);
  wb_distance_sensor_enable(prox_sensors[4],TIME_STEP);
  wb_distance_sensor_enable(prox_sensors[5],TIME_STEP);
  wb_distance_sensor_enable(prox_sensors[6],TIME_STEP);
  wb_distance_sensor_enable(prox_sensors[7],TIME_STEP);
  

  
 
  
  
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
   
  /*
   * Read the sensors :
   * Enter here functions to read sensor data, like:
   *  double val = wb_distance_sensor_get_value(my_sensor);
   */
   
   
  while (wb_robot_step(TIME_STEP) != -1) {
  
    
    bool left_wall = wb_distance_sensor_get_value(prox_sensors[5]) > 80;
    bool left_corner = wb_distance_sensor_get_value(prox_sensors[6]) > 80;
    bool front_wall = wb_distance_sensor_get_value(prox_sensors[7]) > 80;
    //when sensor is more than 80 a wall has been detected
    
    /* Process sensor data here */
    
    //if wall is in front, drive right to follow wall on left
    if (front_wall == true)
    {
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    }
    else
    {
      //if wall is on left, drive straight
      if (left_wall == true)
      {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      }
      //no wall around. driven too far ahead, turn left to find wall
      else
      {
          left_speed = MAX_SPEED/8; //this is the turn 
          right_speed = MAX_SPEED;
      }
      
      //dont get too close to the wall
      if(left_corner == true)
      {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED/8;
      }
    }

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     wb_motor_set_velocity(left_motor, left_speed);
     wb_motor_set_velocity(right_motor, right_speed);
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
