/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

// NOLINTNEXTLINE(build/c++11)
#include <chrono>
#include <cmath>
#include <iostream>
// NOLINTNEXTLINE(build/c++11)
#include <thread>

#include "keller_sensor_driver.h"

int main() {
  const std::string port = "/dev/ttyUSB5";

  std::shared_ptr<keller::KellerSerialInterface> pSerial =
      std::make_shared<keller::KellerSerialInterface>(port, 115200);
  keller::KellerSensorDriver driver =
      keller::KellerSensorDriver(pSerial, 5, 15, 0.75);

  // Address request
  // int address = 0;
  // driver.getHardwareAddress(&address);
  // std::cout << "Device address: "  << address << std::endl;
  // driver.setSoftwareAddress(address);

  // Read and print coefficients
  // uint8_t coeff_address;
  // float coeff_value;
  // for (uint8_t i = 64; i <= 79; i++) {
  //   coeff_address = i;
  //   if (driver.F30(coeff_address, &coeff_value) ==
  // CommunicationStatus::ok) {
  //     std::cout
  //       << "Coeff No. " << static_cast<int>(coeff_address)
  //       << " : " << coeff_value << "\n";
  //   } else {
  //     driver.explainCommStatus();
  //   }
  // }

  // Disconnection experiment
  // Measurement rate in Hz
  double loopRateHz = 10.0;
  // keller::FullStatus status;
  float pressure;
  bool sensor_available = true;
  bool measurement_valid;

  // Calculate the duration of each loop iteration in milliseconds
  std::chrono::milliseconds loopDuration(static_cast<int>(1000.0 / loopRateHz));

  while (true) {
    auto startTime = std::chrono::steady_clock::now();

    if (sensor_available) {
      pressure = driver.getPressureMeasurement();
      measurement_valid = !(std::isnan(pressure));
      sensor_available = driver.checkConnection();
      if (measurement_valid) {
        std::cout << "Pressure: " << pressure << " bar" << std::endl;
      } else {
        std::cout << "Measurement Invalid. ";
        std::cout << driver.explainMeasStatus().str().c_str() << std::endl;
      }
    } else {
      std::cout << "Trying to establish connection" << std::endl;
      sensor_available = driver.connect();
      std::cout << driver.explainCommStatus().str().c_str() << std::endl;
    }

    // Calculate the time elapsed since the start of the loop iteration
    auto elapsedTime = std::chrono::steady_clock::now() - startTime;

    // If the loop iteration took less time than the desired duration,
    // sleep for the remaining time to maintain the loop rate
    auto sleepDuration =
        loopDuration -
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsedTime);
    if (sleepDuration > std::chrono::milliseconds(0)) {
      std::this_thread::sleep_for(sleepDuration);
    }
  }

  return 0;
}
