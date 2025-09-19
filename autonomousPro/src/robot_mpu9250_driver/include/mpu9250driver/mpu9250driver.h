#ifndef mpu9250driver_H
#define mpu9250driver_H

#include "mpu9250sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class mpu9250driver : public rclcpp::Node {
 public:
  mpu9250driver();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::unique_ptr<MPU9250Sensor> mpu9250_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleInput();
  void declareParameters();
  void calculateOrientation(sensor_msgs::msg::Imu& imu_message);
};

#endif  // mpu9250driver_H