#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/device.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>


#define DISTANCE_SENSORS_NUMBER 8
static WbDeviceTag distance_sensors[DISTANCE_SENSORS_NUMBER];
static double distance_sensors_values[DISTANCE_SENSORS_NUMBER];
static const char *distance_sensors_names[DISTANCE_SENSORS_NUMBER] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};

#define GROUND_SENSORS_NUMBER 3
static WbDeviceTag ground_sensors[GROUND_SENSORS_NUMBER];
static double ground_sensors_values[GROUND_SENSORS_NUMBER] = {0.0, 0.0, 0.0};
static const char *ground_sensors_names[GROUND_SENSORS_NUMBER] = {"gs0", "gs1", "gs2"};

#define LEDS_NUMBER 10
static WbDeviceTag leds[LEDS_NUMBER];
static bool leds_values[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};

static WbDeviceTag left_motor, right_motor;

#define LEFT 0
#define RIGHT 1
#define TIME_STEP 64
#define MAX_SPEED 6.28
static double speeds[2];

int main(int argc, char **argv) {

  wb_robot_init();

  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  for (i = 0; i < 8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  while (wb_robot_step(TIME_STEP) != -1) {
    double ps_values[8];
    for (i = 0; i < 8 ; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);

    bool right_obstacle =
      ps_values[0] > 100.0 ||
      ps_values[1] > 100.0 ||
      ps_values[2] > 100.0;
    bool left_obstacle =
      ps_values[5] > 100.0 ||
      ps_values[6] > 100.0 ||
      ps_values[7] > 100.0;

    double left_speed  = 0.5 * MAX_SPEED;
    double right_speed = 0.5 * MAX_SPEED;

    if (left_obstacle) {
      // turn right
      left_speed  = 0.5 * MAX_SPEED;
      right_speed = -0.5 * MAX_SPEED;
    }
    else if (right_obstacle) {
      // turn left
      left_speed  = -0.5 * MAX_SPEED;
      right_speed = 0.5 * MAX_SPEED;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
