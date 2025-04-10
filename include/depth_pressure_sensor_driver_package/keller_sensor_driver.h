/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#pragma once

// NOLINTNEXTLINE(build/c++11)
#include <condition_variable>
#include <memory>
// NOLINTNEXTLINE(build/c++11)
#include <mutex>
#include <sstream>
#include <string>

#include "depth_pressure_sensor_driver_constants.h"
#include "keller_serial_interface.h"

namespace keller {

class KellerSensorDriver {
 public:
  // Constructor and destructor
  KellerSensorDriver();
  explicit KellerSensorDriver(
      std::shared_ptr<SerialInterface> pSerial, uint8_t dev_address = 1,
      float response_time_fix = constants::OPTIMAL_TIME_FIX_B115200,
      float response_time_per_byte = constants::OPTIMAL_TIME_PER_BYTE_B115200,
      bool repeat_if_error = false);

  // Functions that interact with hardware
  bool connect();
  bool disconnect();
  bool checkConnection();
  float getLatency();
  FullStatus getStatus();
  bool initializeHardware();
  bool getHardwareAddress(int* cur_address);
  bool setHardwareAddress(uint8_t new_address);
  bool setHardwareBaud(int new_baud);
  float getPressureMeasurement();
  float getPressureMeasurementQuant();
  float getTemperatureMeasurement();
  float getTemperatureMeasurementQuant();

  // Functions that don't interact with hardware
  bool checkIfSerialIsEmulated();
  void restartDevice();
  std::stringstream explainCommStatus();
  std::stringstream explainMeasStatus();
  int getSoftwareBaud();
  uint8_t getSoftwareAddress();
  void setSoftwareAddress(uint8_t address);

 private:
  std::shared_ptr<SerialInterface> pSerial_;
  uint8_t dev_address_;
  FullStatus status_;
  bool repeat_if_error_;
  int max_repetitions_;
  int repeat_counter_;

  // TX and RX buffers
  uint8_t tx_buffer_[constants::MAX_TX_LENGTH];
  uint8_t rx_buffer_[constants::MAX_TX_LENGTH + constants::MAX_RX_LENGTH];

  // to calculate the wait time before reading the slave response, [ms]
  float response_time_fix_;
  float response_time_per_byte_;

  // Conditional variable and mutex necessary for communication cancellation
  std::shared_ptr<std::condition_variable> pCancelVar_;
  std::shared_ptr<std::mutex> pCancelMutex_;

  // Aux functions
  void transferData(uint16_t nTX, uint16_t nRX);
  CommunicationStatus transferDataDirect(uint16_t nTX, uint16_t nRX);
  uint16_t calcCrc16(const uint8_t* Data, uint16_t nCnt);
  CommunicationStatus parseException();
  float getMeasurement(MeasurementType measurement_type);
  // CommunicationStatus handleException();

  // Keller bus functions
  void F30(uint8_t coeff_address, float* coeff_value);
  void F31(uint8_t coeff_address, float coeff_value);
  void F32(uint8_t coeff_address, uint8_t* coeff_value);
  void F33(uint8_t coeff_address, uint8_t coeff_value);
  void F48(uint8_t* device_class, uint8_t* group, uint8_t* year, uint8_t* week,
           uint8_t* buffer, uint8_t* state);
  void F66(uint8_t new_address, uint8_t* act_address);
  void F69(uint32_t* serial_number);
  void F73(uint8_t channel, float* value, uint8_t* status);
  void F74(uint8_t channel, int32_t* value, uint8_t* status);
  void F95(uint8_t command);
  void F95_Val(uint8_t command, float value);
};

}  // namespace keller
