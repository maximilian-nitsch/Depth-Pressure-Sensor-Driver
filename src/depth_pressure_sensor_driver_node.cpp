/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "depth_pressure_sensor_driver_node.h"

namespace keller {

DpthSensorDriverNode::DpthSensorDriverNode()
    : Node("depth_sensor_driver_node"),
      last_pressure_measurement_(0.0),
      sequence_id_(0),
      port_(""),
      baud_rate_(115200),
      modbus_device_address_(1),
      response_time_fix_(15.0),
      response_time_per_byte_(0.75),
      repeat_if_error_(false),
      sample_time_(0.1),
      pressure_sensor_std_(0.0),
      pressure_per_metre_(10110.8212387044),
      pressure_bias_(0.0),
      lever_arm_body_to_sensor_({0.0, 0.0, 0.0}),
      enable_publish_pressure_(true),
      enable_publish_depth_(true),
      enable_publish_temperature_(true),
      enable_publish_diagnostic_(true),
      enable_constant_pressure_to_meter_conversion_(true),
      enable_constant_pressure_bias_(false),
      component_name_(""),
      component_type_(""),
      component_vendor_(""),
      component_id_("") {
  // Write the settings from YAML file to the parameter server and load them
  // into the pressure sensor driver class
  declareAndRetrieveSettings();

  RCLCPP_INFO(rclcpp::get_logger("sensor." + component_name_),
              ("Configuring " + component_name_ + "_node...").c_str());

  RCLCPP_INFO(rclcpp::get_logger("sensor." + component_name_),
              "Parameters from YAML config loaded successfully.");

  // Print the settings from YAML file
  std::stringstream yamlSettings = printSettings();
  RCLCPP_INFO(rclcpp::get_logger("sensor." + component_name_), "%s",
              yamlSettings.str().c_str());

  // Retrieve the namespace from the node
  std::string nsStr = get_namespace();

  // Declare the value of the topic_name parameter
  this->declare_parameter("topic_name", rclcpp::PARAMETER_STRING);

  // Open the serial port and create the depth sensor driver
  pSerial_ = std::make_shared<KellerSerialInterface>(port_, baud_rate_);
  pDriver_ = std::make_shared<KellerSensorDriver>(
      pSerial_, modbus_device_address_, response_time_fix_,
      response_time_per_byte_, repeat_if_error_);

  // Initialize the fluid pressure publisher
  pFluidPressurePublisher_ =
      this->create_publisher<sensor_msgs::msg::FluidPressure>("fluid_pressure",
                                                              10);

  // Initialize the temperature publisher
  pTemperaturePublisher_ =
      this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);

  // Initialize the depth publisher
  pDepthPublisher_ =
      this->create_publisher<geometry_msgs::msg::PointStamped>("depth", 10);

  // Initialize the latency publisher
  pLatencyPublisher_ =
      this->create_publisher<std_msgs::msg::Float64>("latency", 10);

  // Initialize the diagnostic publisher
  pDiagnosticPublisher_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "diagnostic", 10);

  // Create a timer to call the depth driver loop callback function
  pTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(sample_time_ * 1000)),
      std::bind(&DpthSensorDriverNode::loopCallback, this));

  // Create diagnostic message to track serial port opening
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

  rclcpp::Time currentTimestamp = now();

  // Check if serial port is open, otherwise return and stop the node
  if (pDriver_->checkConnection()) {
    RCLCPP_INFO(rclcpp::get_logger("sensor." + component_name_),
                "Serial port opened successfully.");

    // Print OK diagnostic message when port successfully openend
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnosticMsg.name = component_name_;
    diagnosticMsg.message = "Serial port opened successfully";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Publish the diagnostic array message
    pDiagnosticPublisher_->publish(diagnosticArrayMsg);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("sensor." + component_name_),
                 "Serial port could not be opened!");

    // Print OK diagnostic message when port successfully openend
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    diagnosticMsg.name = component_name_;
    diagnosticMsg.message = "Serial port could not be opened!";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Publish the diagnostic array message
    pDiagnosticPublisher_->publish(diagnosticArrayMsg);
  }

  RCLCPP_INFO(rclcpp::get_logger("sensor." + component_name_),
              (component_name_ + "_node initialized.").c_str());
}

/**
 * @brief Destructor for the depth sensor driver node. Print shutdown message.
 *
 * This function is called when the depth sensor driver node is destroyed.
 *
 * @param[in] None
 *
 * @return None
 */
DpthSensorDriverNode::~DpthSensorDriverNode() {
  RCLCPP_INFO(rclcpp::get_logger("sensor." + component_name_),
              "Shutting down depth sensor driver node...");
}

/**
 * @brief Depth pressure sensor driver loop callback function.
 *
 * This function is called when the driver loop timer is triggered.
 * The depth sensor driver yields fluid pressure and temperature measurements,
 * if the serial connection is healthy. Fluid pressure, depth
 * (calculated from fluid pressure), and temperature messages are published. The
 * diagnostic message array is published to indicate the status of the driver
 * and individual messages.
 *
 * @param[in] None
 *
 * @return None
 */
void DpthSensorDriverNode::loopCallback() {
  // Get current timestamp
  rclcpp::Time currentTimestamp = now();
  sequence_id_++;

  // Create ROS messages to be published
  sensor_msgs::msg::FluidPressure fluidPressureMsg;
  sensor_msgs::msg::Temperature temperatureMsg;
  geometry_msgs::msg::PointStamped depthMsg;
  std_msgs::msg::Float64 latencyMsg;
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

  //   // Remove the static bias from the pressure measurement
  //   dpthSensorMeasurement.pressure -= pressure_bias;
  //   dpthSensorMeasurement.depth -=
  //       pressure_bias /
  //       pDpthSensorSimulator_->getSimParams().pressure_per_metre;

  float pressure;
  float temperature;
  float latency;

  bool pressure_measurement_valid;
  bool temperature_measurement_valid;
  bool latency_measurement_valid;

  bool sensor_available = true;

  // Check if sensor connection is available
  if (sensor_available) {
    // Read out measurements from core driver
    pressure = pDriver_->getPressureMeasurement();
    temperature = pDriver_->getTemperatureMeasurement();
    latency = pDriver_->getLatency();

    // Check if one of the measurements is invalid (NaN)
    pressure_measurement_valid = !(std::isnan(pressure));
    temperature_measurement_valid = !(std::isnan(temperature));
    latency_measurement_valid = !(std::isnan(latency));

    // Check communication status from core driver
    sensor_available = pDriver_->checkConnection();

    // Read and publish pressure measurement
    if (pressure_measurement_valid) {
      // Fill the fluid pressure message
      fluidPressureMsg.header.stamp = currentTimestamp;
      fluidPressureMsg.header.frame_id = component_name_ + "_link";
      fluidPressureMsg.fluid_pressure =
          pressure * 100000 - pressure_bias_;  // 1 bar = 100,000 Pa
      fluidPressureMsg.variance = pressure_sensor_std_ * pressure_sensor_std_;

      // Publish the fluid pressure message
      pFluidPressurePublisher_->publish(fluidPressureMsg);

      // Fill the depth meassage
      depthMsg.header.stamp = currentTimestamp;
      depthMsg.header.frame_id = component_name_ + "_link";
      depthMsg.point.x = 0.0;
      depthMsg.point.y = 0.0;
      depthMsg.point.z = fluidPressureMsg.fluid_pressure / pressure_per_metre_;

      // Publish the depth meassage
      pDepthPublisher_->publish(depthMsg);

      // Print ERROR diagnostic message when measurement is invalid
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diagnosticMsg.name = component_name_;
      diagnosticMsg.message = component_name_ + " nominal";
      diagnosticMsg.hardware_id = "pressure";

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);

    } else {
      // Get measurement status from core driver and print it
      std::stringstream measStatus = pDriver_->explainMeasStatus();

      RCLCPP_ERROR(rclcpp::get_logger("sensor." + component_name_),
                   "Pressure measurement invalid! %s",
                   measStatus.str().c_str());

      // Print ERROR diagnostic message when measurement is invalid
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnosticMsg.name = component_name_;
      diagnosticMsg.message = "Pressure measurement invalid!";
      diagnosticMsg.hardware_id = "pressure";

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);
    }

    // Read and publish temperature measurement
    if (temperature_measurement_valid) {
      // Fill the temperature message
      temperatureMsg.header.stamp = currentTimestamp;
      temperatureMsg.header.frame_id = component_name_ + "_link";
      temperatureMsg.temperature = temperature;
      temperatureMsg.variance = 0.0;

      // Publish the temperature message
      pTemperaturePublisher_->publish(temperatureMsg);

      // Print ERROR diagnostic message when measurement is invalid
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diagnosticMsg.name = component_name_;
      diagnosticMsg.message = component_name_ + " nominal";
      diagnosticMsg.hardware_id = "temperature";

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);

    } else {
      // Get measurement status from core driver and print it
      std::stringstream measStatus = pDriver_->explainMeasStatus();

      RCLCPP_ERROR(rclcpp::get_logger("sensor." + component_name_),
                   "Temperature measurement invalid! %s",
                   measStatus.str().c_str());

      // Print ERROR diagnostic message when measurement is invalid
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnosticMsg.name = component_name_;
      diagnosticMsg.message = "Temperature measurement invalid!";
      diagnosticMsg.hardware_id = "temperature";

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);
    }

    // Read and publish latency measurement
    if (latency_measurement_valid) {
      // Fill the temperature message
      latencyMsg.data = latency;

      // Publish the temperature message
      pLatencyPublisher_->publish(latencyMsg);

      // Print ERROR diagnostic message when measurement is invalid
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      diagnosticMsg.name = component_name_;
      diagnosticMsg.message = component_name_ + " nominal";
      diagnosticMsg.hardware_id = "latency";

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);

    } else {
      // Get measurement status from core driver and print it
      std::stringstream measStatus = pDriver_->explainMeasStatus();

      RCLCPP_ERROR(rclcpp::get_logger("sensor." + component_name_),
                   "Latency measurement invalid! %s", measStatus.str().c_str());

      // Print ERROR diagnostic message when measurement is invalid
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      diagnosticMsg.name = component_name_;
      diagnosticMsg.message = "Latency measurement invalid!";
      diagnosticMsg.hardware_id = "latency";

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);
    }

  } else {
    RCLCPP_WARN(rclcpp::get_logger("sensor." + component_name_),
                "Trying to establish connection...");

    // Try to reconnect
    sensor_available = pDriver_->connect();

    // Get measurement status from core driver and print it
    std::stringstream measStatus = pDriver_->explainMeasStatus();

    RCLCPP_WARN(rclcpp::get_logger("sensor." + component_name_), "%s",
                measStatus.str().c_str());

    // Print STALE diagnostic message when measurement is invalid
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = component_name_;
    diagnosticMsg.message =
        "Sensor not available! Trying to establish connection...";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Publish the diagnostic array message
    pDiagnosticPublisher_->publish(diagnosticArrayMsg);
  }
}

void DpthSensorDriverNode::declareAndRetrieveSettings() {
  // Component settings
  std::string prev_hierarchies =
      "depth_pressure_sensor_driver.component_settings.";
  this->declare_parameter(prev_hierarchies + "name", rclcpp::PARAMETER_STRING);
  this->declare_parameter(prev_hierarchies + "type", rclcpp::PARAMETER_STRING);
  this->declare_parameter(prev_hierarchies + "vendor",
                          rclcpp::PARAMETER_STRING);
  this->declare_parameter(prev_hierarchies + "id", rclcpp::PARAMETER_STRING);

  component_name_ = this->get_parameter(prev_hierarchies + "name").as_string();
  component_type_ = this->get_parameter(prev_hierarchies + "type").as_string();
  component_vendor_ =
      this->get_parameter(prev_hierarchies + "vendor").as_string();
  component_id_ = this->get_parameter(prev_hierarchies + "id").as_string();

  // Serial device settings
  prev_hierarchies = "depth_pressure_sensor_driver.serial_device_settings.";
  this->declare_parameter(prev_hierarchies + "port", rclcpp::PARAMETER_STRING);
  this->declare_parameter(prev_hierarchies + "baud_rate",
                          rclcpp::PARAMETER_INTEGER);

  port_ = this->get_parameter(prev_hierarchies + "port").as_string();
  baud_rate_ = this->get_parameter(prev_hierarchies + "baud_rate").as_int();

  // Modbus device settings
  prev_hierarchies = "depth_pressure_sensor_driver.modbus_device_settings.";
  this->declare_parameter(prev_hierarchies + "modbus_device_address",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(prev_hierarchies + "response_timefix",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "response_time_per_byte",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "repeat_if_error",
                          rclcpp::PARAMETER_BOOL);

  modbus_device_address_ =
      this->get_parameter(prev_hierarchies + "modbus_device_address").as_int();
  response_time_fix_ =
      this->get_parameter(prev_hierarchies + "response_timefix").as_double();
  response_time_per_byte_ =
      this->get_parameter(prev_hierarchies + "response_time_per_byte")
          .as_double();
  repeat_if_error_ =
      this->get_parameter(prev_hierarchies + "repeat_if_error").as_bool();

  // Pressure sensor settings
  prev_hierarchies = "depth_pressure_sensor_driver.pressure_sensor_settings.";
  this->declare_parameter(prev_hierarchies + "sample_time",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "pressure_noise_std",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "pressure_per_metre",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "pressure_bias",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "lever_arm_body_to_sensor",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);

  sample_time_ =
      this->get_parameter(prev_hierarchies + "sample_time").as_double();
  pressure_sensor_std_ =
      this->get_parameter(prev_hierarchies + "pressure_noise_std").as_double();
  pressure_per_metre_ =
      this->get_parameter(prev_hierarchies + "pressure_per_metre").as_double();
  pressure_bias_ =
      this->get_parameter(prev_hierarchies + "pressure_bias").as_double();
  lever_arm_body_to_sensor_ =
      this->get_parameter(prev_hierarchies + "lever_arm_body_to_sensor")
          .as_double_array();

  // Enable flags
  prev_hierarchies = "depth_pressure_sensor_driver.enable_flags_settings.";
  this->declare_parameter(prev_hierarchies + "enable_publish_pressure",
                          rclcpp::PARAMETER_BOOL);  // (-)
  this->declare_parameter(prev_hierarchies + "enable_publish_depth",
                          rclcpp::PARAMETER_BOOL);  // (-)
  this->declare_parameter(prev_hierarchies + "enable_publish_temperature",
                          rclcpp::PARAMETER_BOOL);  // (-)
  this->declare_parameter(prev_hierarchies + "enable_publish_diagnostic",
                          rclcpp::PARAMETER_BOOL);  // (-)
  this->declare_parameter(
      prev_hierarchies + "enable_constant_pressure_to_meter_conversion",
      true);  // (-)
  this->declare_parameter(prev_hierarchies + "enable_constant_pressure_bias",
                          rclcpp::PARAMETER_BOOL);  // (Pa)

  enable_publish_pressure_ =
      this->get_parameter(prev_hierarchies + "enable_publish_pressure")
          .as_bool();
  enable_publish_depth_ =
      this->get_parameter(prev_hierarchies + "enable_publish_depth").as_bool();
  enable_publish_temperature_ =
      this->get_parameter(prev_hierarchies + "enable_publish_temperature")
          .as_bool();
  enable_publish_diagnostic_ =
      this->get_parameter(prev_hierarchies + "enable_publish_diagnostic")
          .as_bool();
  enable_constant_pressure_to_meter_conversion_ =
      this->get_parameter(prev_hierarchies +
                          "enable_constant_pressure_to_meter_conversion")
          .as_bool();
  enable_constant_pressure_bias_ =
      this->get_parameter(prev_hierarchies + "enable_constant_pressure_bias")
          .as_bool();
}

std::stringstream DpthSensorDriverNode::printSettings() {
  // Create stringstream to store the output
  std::stringstream ss;

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";
  ss << std::left << std::setw(50) << component_name_ + "_node Settings"
     << "\n";
  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Component settings
  ss << std::left << "Component Settings:\n";

  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50) << "Name:" << component_name_ << "\n";

  ss << std::left << std::setw(50) << "Type:" << component_type_ << "\n";

  ss << std::left << std::setw(50) << "Vendor:" << component_vendor_ << "\n";

  ss << std::left << std::setw(50) << "ID:" << component_id_ << "\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Serial device settings
  ss << std::left << "Serial Device Settings:\n";

  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50) << "Serial port:" << port_ << "\n";

  ss << std::left << std::setw(50) << "Baud rate:" << baud_rate_ << "\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Modbus settings
  ss << std::left << "Modbus Settings:\n";

  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50)
     << "Modbus device address:" << static_cast<int>(modbus_device_address_)
     << "\n";

  ss << std::left << std::setw(50) << "Response time fix:" << response_time_fix_
     << " ms\n";

  ss << std::left << std::setw(50)
     << "Response time per byte:" << response_time_per_byte_ << " ms\n";

  ss << std::left << std::setw(50) << "Repeat if error:" << repeat_if_error_
     << "\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Pressure sensor settings
  ss << std::left << "Pressure Sensor Settings:\n";

  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50) << "Sample time:" << sample_time_ << " s\n";

  ss << std::left << std::setw(50)
     << "Pressure sensor standard deviation:" << pressure_sensor_std_
     << " Pa\n";

  ss << std::left << std::setw(50)
     << "Pressure-to-meter conversion:" << pressure_per_metre_ << " Pa/mH20\n";

  ss << std::left << std::setw(50) << "Pressure bias:" << pressure_bias_
     << " Pa\n";

  ss << std::left << std::setw(50)
     << "Lever arm:" << lever_arm_body_to_sensor_[0] << " "
     << lever_arm_body_to_sensor_[1] << " " << lever_arm_body_to_sensor_[2]
     << " m\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Depth sensor model enable settings
  ss << std::left << "Enable settings:\n";

  ss << std::left << std::setw(50)
     << "Enable fluid pressure publisher:" << enable_publish_pressure_ << "\n";

  ss << std::left << std::setw(50)
     << "Enable depth publisher:" << enable_publish_depth_ << "\n";

  ss << std::left << std::setw(50)
     << "Enable temperature publisher:" << enable_publish_temperature_ << "\n";

  ss << std::left << std::setw(50)
     << "Enable diagnostic publisher:" << enable_publish_diagnostic_ << "\n";

  ss << std::left << std::setw(50)
     << "Enable constant pressure-to-meter conversion:"
     << enable_constant_pressure_to_meter_conversion_ << "\n";

  ss << std::left << std::setw(50)
     << "Enable constant pressure bias:" << enable_constant_pressure_bias_
     << "\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  return ss;
}

}  // namespace keller

/**
 * @brief Main function for the depth sensor driver node.
 *
 * This function is the entry point for the depth sensor driver node.
 *
 * @param[in] argc Number of command line arguments
 * @param[in] argv Pointer to the command line arguments
 *
 * @return 0
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<keller::DpthSensorDriverNode>());

  rclcpp::shutdown();

  return 0;
}
