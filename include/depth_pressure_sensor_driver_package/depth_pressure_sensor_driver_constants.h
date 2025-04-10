/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#pragma once

namespace keller {

enum class CommunicationStatus {
  ok = 0,

  error,  // generic error
  bad_address,
  bad_crc,
  bad_response,
  no_response,
  bad_rx_length,
  fail_read,
  bad_tx_length,
  fail_write,
  fail_config,
  fail_open_port,
  cancel,
  invalid_baud,

  exception,  // generic exception
  exception_1,
  exception_2,
  exception_3,
  exception_4,
  exception_32,
  bad_exception
};

enum class MeasurementStatus {
  valid = 0,
  not_available,
  bad_status_powerup,
  bad_status_saturation,
  bad_status_temperature,
  bad_status_pressure,
};

struct FullStatus {
  CommunicationStatus comm;
  MeasurementStatus meas;
};

enum class MeasurementType {
  pressure = 0,
  pressure_quant = 1,
  temperature = 2,
  temperature_quant = 3
};

}  // namespace keller

namespace constants {

constexpr uint16_t MAX_TX_LENGTH = 20;   // Length of send-buffer
constexpr uint16_t MAX_RX_LENGTH = 275;  // Length of receive-buffer
constexpr float OPTIMAL_TIME_FIX_B115200 = 15.0;
constexpr float OPTIMAL_TIME_PER_BYTE_B115200 = 0.75;
constexpr float OPTIMAL_TIME_FIX_B9600 = 100.0;
constexpr float OPTIMAL_TIME_PER_BYTE_B9600 = 1.5;

}  // namespace constants
