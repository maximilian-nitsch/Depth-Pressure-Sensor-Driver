/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#pragma once

// NOLINTNEXTLINE(build/c++11)
#include <chrono>
#include <memory>
#include <string>
// NOLINTNEXTLINE(build/c++11)
#include <thread>

#include "keller_sensor_driver.h"

#include "gtest/gtest.h"

/**
 * @brief Parameterized test fixture for KellerSensorDriver Class
 * 
 * The main idea is to create tests which can be conducted for both the emulated
 * serial interface and the real hardware.
 * This has the following advantages:
 *  - allow unit testing for both the hardware and its emulation
 *  - Avoid code duplication and curation of both test suites
 *  - Ensure that automatic testing still occures using the emulated interface
 * in the CI/CD process
 *  - Ensure that exactly same tests are done for the hardware,
 * whenever it is available
 *  - The differences between testing hardware and emulator can be kept
 *  MINIMAL and they forced to always be EXPLICIT (using an if statement)
 * 
 * These are parameterized tests.
 * The parameter is the object of KellerSensorDriver class
 * They're created with TEST_P and need to be explicitly instantiated
 * for each value of the parameter (kinda like templates) (see below).
 */
class KellerSensorDriverTest
    : public ::testing::TestWithParam<
          std::shared_ptr<keller::KellerSensorDriver>> {
 protected:
  KellerSensorDriverTest() {
    pKellerSensorDriver_ = GetParam();
    dev_address_ = 5;
  }
  std::shared_ptr<keller::KellerSensorDriver> pKellerSensorDriver_;
  int dev_address_;
};

/**
 * @brief classifies the tests based on whether they test an emulated sensor or
 * the real one
 * 
 * @param info test parameter info object
 * @return std::string test type
 */
std::string testClassification(
    const testing::TestParamInfo<std::shared_ptr<keller::KellerSensorDriver>>&
        info) {
  std::string test_type;
  if (info.param->checkIfSerialIsEmulated()) {
    test_type = std::string("emulated");
  } else {
    test_type = std::string("hardware");
  }
  return test_type;
}

/**
 * @brief Test if "not initialized" exception is thrown and handled correctly
 * in case any operation (here: getting the address) is performed before
 * before initialization
 * 
 * The device should be uninitialized in order for this test to pass. Restart
 * the real device before conducting this test on it.
 */
TEST_P(KellerSensorDriverTest, CatchNotInitializedTest) {
  int dummy;
  pKellerSensorDriver_->restartDevice();
  pKellerSensorDriver_->getHardwareAddress(&dummy);
  EXPECT_EQ(pKellerSensorDriver_->getStatus().comm,
            keller::CommunicationStatus::exception_32);
}

/**
 * @brief Test if initialization is performed successfully
 */
TEST_P(KellerSensorDriverTest, InitSuccessfulTest) {
  EXPECT_TRUE(pKellerSensorDriver_->initializeHardware());
}

/**
 * @brief Test if disconnection and connection works
 * 
 */
TEST_P(KellerSensorDriverTest, SensorConnectTest) {
  EXPECT_TRUE(pKellerSensorDriver_->disconnect());
  EXPECT_TRUE(pKellerSensorDriver_->connect());
}

/**
 * @brief Test if connection checking works
 * 
 */
TEST_P(KellerSensorDriverTest, CheckConnectionTest) {
  EXPECT_TRUE(pKellerSensorDriver_->checkConnection());
}

/**
 * @brief Test the function for getting the device status. The connection status
 * should also be normal;
 * 
 */
TEST_P(KellerSensorDriverTest, GetStatusTest) {
  keller::FullStatus status = pKellerSensorDriver_->getStatus();
  EXPECT_EQ(status.comm, keller::CommunicationStatus::ok);
  EXPECT_EQ(status.meas, keller::MeasurementStatus::not_available);
}

/**
 * @brief Test if the latency can be retrieved
 * 
 */
TEST_P(KellerSensorDriverTest, GetLatencyTest) {
  float latency = pKellerSensorDriver_->getLatency();
  keller::FullStatus status = pKellerSensorDriver_->getStatus();
  EXPECT_GT(latency, 0);
  EXPECT_EQ(status.comm, keller::CommunicationStatus::ok);
  EXPECT_EQ(status.meas, keller::MeasurementStatus::not_available);
}

/**
 * @brief Test if the baud rate can be changed
 * 
 */
TEST_P(KellerSensorDriverTest, SetHardwareBaudTest) {
  int current_baud = pKellerSensorDriver_->getSoftwareBaud();
  int new_baud;
  switch (current_baud) {
    case 9600:
      new_baud = 115200;
      break;
    case 115200:
      new_baud = 9600;
      break;
  }
  std::cout << "New baud will be:" << new_baud << std::endl;
  EXPECT_TRUE(pKellerSensorDriver_->setHardwareBaud(new_baud));
  EXPECT_TRUE(pKellerSensorDriver_->checkConnection());
  std::cout << "Returning baud to" << current_baud << std::endl;
  EXPECT_TRUE(pKellerSensorDriver_->setHardwareBaud(current_baud));
}

/**
 * @brief Test if getting the hardware address returns the actual address
 */
TEST_P(KellerSensorDriverTest, GetHardwareAddressTest) {
  int address;
  EXPECT_TRUE(pKellerSensorDriver_->getHardwareAddress(&address));
  EXPECT_EQ(address, dev_address_);
}

/**
 * @brief Test if setting hardware address sets the desired address
 */
TEST_P(KellerSensorDriverTest, SetHardwareAddressTest) {
  int new_address = 10;
  EXPECT_NE(new_address, dev_address_);
  EXPECT_TRUE(pKellerSensorDriver_->setHardwareAddress(new_address));
  int soft_address = pKellerSensorDriver_->getSoftwareAddress();
  int hard_address;
  EXPECT_TRUE(pKellerSensorDriver_->getHardwareAddress(&hard_address));
  EXPECT_EQ(new_address, soft_address);
  EXPECT_EQ(new_address, hard_address);
  // Set the address back (for the future use of real hardware)
  EXPECT_TRUE(pKellerSensorDriver_->setHardwareAddress(dev_address_));
  soft_address = pKellerSensorDriver_->getSoftwareAddress();
  EXPECT_TRUE(pKellerSensorDriver_->getHardwareAddress(&hard_address));
  EXPECT_EQ(dev_address_, soft_address);
  EXPECT_EQ(dev_address_, hard_address);
}

/**
 * @brief Test if the expected pressure measurement is returned
 */
TEST_P(KellerSensorDriverTest, GetSinglePressureMeasurement) {
  float expected_pressure = -0.0042;  // bar
  float read_pressure;
  read_pressure = pKellerSensorDriver_->getPressureMeasurement();
  if (pKellerSensorDriver_->checkIfSerialIsEmulated()) {
    EXPECT_EQ(read_pressure, expected_pressure);
  } else {
    EXPECT_NEAR(read_pressure, expected_pressure, 1);
  }
}

/**
 * @brief Test if the expected pressure measurement is returned using the
 * quantized register
 */
TEST_P(KellerSensorDriverTest, GetSinglePressureMeasurementQuantized) {
  float expected_pressure = -0.0042;  // bar
  float read_pressure;
  read_pressure = pKellerSensorDriver_->getPressureMeasurementQuant();
  if (pKellerSensorDriver_->checkIfSerialIsEmulated()) {
    EXPECT_EQ(read_pressure, expected_pressure);
  } else {
    EXPECT_NEAR(read_pressure, expected_pressure, 1);
  }
}

/**
 * @brief Test if the expected temperature measurement is returned
 */
TEST_P(KellerSensorDriverTest, GetSingleTemperatureMeasurement) {
  float expected_temperature = 24.42;  // deg C
  float read_temperature;
  read_temperature = pKellerSensorDriver_->getTemperatureMeasurement();
  if (pKellerSensorDriver_->checkIfSerialIsEmulated()) {
    EXPECT_EQ(read_temperature, expected_temperature);
  } else {
    EXPECT_NEAR(read_temperature, expected_temperature, 15);
  }
}

/**
 * @brief Test if the expected pressure measurement is returned using the
 * quantized register
 */
TEST_P(KellerSensorDriverTest, GetSingleTemperatureMeasurementQuantized) {
  float expected_temperature = 24.42;  // deg C
  float read_temperature;
  read_temperature = pKellerSensorDriver_->getTemperatureMeasurementQuant();
  if (pKellerSensorDriver_->checkIfSerialIsEmulated()) {
    EXPECT_EQ(read_temperature, expected_temperature);
  } else {
    EXPECT_NEAR(read_temperature, expected_temperature, 15);
  }
}
