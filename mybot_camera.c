#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/gps.h>
#include <stdio.h>

#define SPEED 11
#define TIME_STEP 64

int main() {
  wb_robot_init();
   
  // Get and enable the distance sensors
  WbDeviceTag ds0 = wb_robot_get_device("ds0");
  WbDeviceTag ds1 = wb_robot_get_device("ds1");
  wb_distance_sensor_enable(ds0, TIME_STEP);
  wb_distance_sensor_enable(ds1, TIME_STEP);

  // Get and enable the camera and recognition
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 2 * TIME_STEP);
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  wb_camera_recognition_enable(camera, TIME_STEP);

  // Get a handler to the motors and set target position to infinity (speed control)
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  // Get and enable the GPS
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    // Get distance sensor values
    double ds0_value = wb_distance_sensor_get_value(ds0);
    double ds1_value = wb_distance_sensor_get_value(ds1);

    // Refresh the camera
    wb_camera_get_image(camera);

    // Compute the motor speeds
    double left_speed, right_speed;
    if (ds1_value > 500) {
      if (ds0_value > 500) {
        // If both distance sensors are detecting something, we move backwards
        left_speed = -SPEED;
        right_speed = -SPEED / 2;
      } else {
        // Turn proportionally to the sensor values
        left_speed = -ds1_value / 100;
        right_speed = (ds0_value / 100) + 0.5;
      }
    } else if (ds0_value > 500) {
      left_speed = (ds1_value / 100) + 0.5;
      right_speed = -ds0_value / 100;
    } else {
      // Move forward at maximal speed if nothing was detected
      left_speed = SPEED;
      right_speed = SPEED;
    }
    
    // Get GPS coordinates
    const double* coords = wb_gps_get_values(gps);
    double x = coords[0];
    double y = coords[1];
    double z = coords[2];
    
    // Get camera image and detect object size
    const unsigned char* image = wb_camera_get_image(camera);
    int object_size = 0;
    for (int i = 0; i < width * height; i++) {
      if (image[i] > 127) {
        object_size++;
      }
    }
    
    // Print object size
    printf("Object size: %d\n", object_size);
    
     // Print GPS coordinates
    printf("Current coordinates: x=%f, y=%f, z=%f\n", x, y, z);


    // Set the motor speeds
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  return 0;
}