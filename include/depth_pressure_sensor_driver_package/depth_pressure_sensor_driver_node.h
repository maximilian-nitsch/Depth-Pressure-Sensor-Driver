/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "keller_sensor_driver.h"

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace keller {

class DpthSensorDriverNode : public rclcpp::Node {
 public:
  // Constructor and destructor
  DpthSensorDriverNode();
  ~DpthSensorDriverNode();

 private:
  // Serial interface and driver
  std::shared_ptr<KellerSerialInterface> pSerial_;
  std::shared_ptr<KellerSensorDriver> pDriver_;

  // Node publisher
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr
      pFluidPressurePublisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr
      pTemperaturePublisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      pDepthPublisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pLatencyPublisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      pDiagnosticPublisher_;

  // Static tf2 broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> pStaticTf2Broadcaster_;

  // Timers
  rclcpp::TimerBase::SharedPtr pTimer_;

  // Last pressure sensor measurement
  double last_pressure_measurement_;

  // Sequence ID
  uint32_t sequence_id_;

  // Component settings
  std::string component_name_;
  std::string component_type_;
  std::string component_vendor_;
  std::string component_id_;

  // Serial device settings
  std::string port_;
  int baud_rate_;

  // Modbus device settings
  uint8_t modbus_device_address_;
  float response_time_fix_;
  float response_time_per_byte_;
  bool repeat_if_error_;

  // Pressure sensor settings
  double sample_time_;
  double pressure_sensor_std_;
  double pressure_per_metre_;
  double pressure_bias_;
  std::vector<double> lever_arm_body_to_sensor_;  // Lever arm vector from the
  // sensor to body frame

  // Enable flags
  bool enable_publish_pressure_;
  bool enable_publish_depth_;
  bool enable_publish_temperature_;
  bool enable_publish_diagnostic_;
  bool enable_constant_pressure_to_meter_conversion_;
  bool enable_constant_pressure_bias_;

  // Timer callback function
  void loopCallback();

  // Declaration and retrieval for parameters from YAML file
  void declareAndRetrieveSettings();

  // Setting printing function
  std::stringstream printSettings();

  // tf2 static broadcaster callback function
  void publishStaticTf2Transforms();
};  // class DpthSensorDriverNode

}  // namespace keller
