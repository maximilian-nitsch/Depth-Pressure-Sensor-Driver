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

#include "endianness.h"
#include "keller_sensor_driver.h"

namespace keller {

/**
 * @brief Default constructor
 */
KellerSensorDriver::KellerSensorDriver() {
  // this->initializeHardware();
  this->repeat_counter_ = 0;
  this->max_repetitions_ = 4;
  this->pCancelVar_ = std::make_shared<std::condition_variable>();
  this->pCancelMutex_ = std::make_shared<std::mutex>();
  this->status_.meas = MeasurementStatus::not_available;
  this->status_.comm = CommunicationStatus::ok;
}

/**
 * @brief Construct a new Communication Interface object
 *
 * Constructor for an unintialized hardware piece. Device address and baud rate
 * must be known and passed to the constructor. The default values are
 *
 * @param port_name communication port name, e.g. /dev/ttyUSB0
 * @param dev_address current device address, should be in range [1, 255],
 * but not 250. Default: 1
 * @param baud_rate current baud rate. 9600 or 115200. Default: 9600
 * @param response_time_fix time in ms to wait before reading the RX buffer,
 *  fixed component
 * @param response_time_per_byte time in ms to wait before reading the RX
 *  buffer, will be multiplied with the expected response length in bytes
 * @param repeat_if_error if false, the first timeout will result in
 *  cancellation of the communication channel. If true, will keep repeating
 *  sending the signal.
 */
KellerSensorDriver::KellerSensorDriver(std::shared_ptr<SerialInterface> pSerial,
                                       uint8_t dev_address,
                                       float response_time_fix,
                                       float response_time_per_byte,
                                       bool repeat_if_error)
    : pSerial_(pSerial),
      dev_address_(dev_address),
      repeat_if_error_(repeat_if_error),
      response_time_fix_(response_time_fix),
      response_time_per_byte_(response_time_per_byte) {
  // this->initializeHardware();
  this->repeat_counter_ = 0;
  this->max_repetitions_ = 4;
  this->pCancelVar_ = std::make_shared<std::condition_variable>();
  this->pCancelMutex_ = std::make_shared<std::mutex>();
  this->status_.meas = MeasurementStatus::not_available;
  this->status_.comm = CommunicationStatus::ok;
  this->connect();
}

/**
 * @brief Make the associated serial open the port
 *
 * @return bool success flag
 */
bool KellerSensorDriver::connect() {
  status_.comm = pSerial_->openCommPort();
  if (status_.comm == CommunicationStatus::ok) {
    checkConnection();
  }
  return status_.comm == CommunicationStatus::ok;
}

/**
 * @brief Make the associated serial close the port
 *
 * @return bool success flag
 */
bool KellerSensorDriver::disconnect() {
  pSerial_->closeCommPort();
  return status_.comm == CommunicationStatus::ok;
}

/**
 * @brief ping the sensor
 *
 * @return bool success flag
 */
bool KellerSensorDriver::checkConnection() {
  uint8_t deviceClass, deviceGroup, year, week, buffer, state;
  F48(&deviceClass, &deviceGroup, &year, &week, &buffer, &state);
  return status_.comm == CommunicationStatus::ok;
}

/**
 * @brief ping the sensor and and save the latency
 *
 * @return float device latency
 */
float KellerSensorDriver::getLatency() {
  uint8_t deviceClass, deviceGroup, year, week, buffer, state;
  auto start = std::chrono::steady_clock::now();
  F48(&deviceClass, &deviceGroup, &year, &week, &buffer, &state);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> duration = end - start;

  if (status_.comm != CommunicationStatus::ok)
    return std::nanf("");

  return duration.count();
}

/**
 * @brief Return the device status
 *
 * @return FullStatus device status
 */
FullStatus KellerSensorDriver::getStatus() {
  return status_;
}

/**
 * @brief Run the initialization bus function, print hardware information
 *
 * Essentially, a wrapper around the keller bus function F48
 *
 * @return bool success flag
 */
bool KellerSensorDriver::initializeHardware() {
  uint8_t deviceClass, deviceGroup, year, week, buffer, state;
  F48(&deviceClass, &deviceGroup, &year, &week, &buffer, &state);
  if (status_.comm == CommunicationStatus::ok) {
    std::cout << "Device Information:"
              << "\n";
    std::cout << "\tClass: " << static_cast<int>(deviceClass) << "\n";
    std::cout << "\tGroup: " << static_cast<int>(deviceGroup) << "\n";
    std::cout << "\tYear: " << static_cast<int>(year) << "\n";
    std::cout << "\tWeek: " << static_cast<int>(week) << "\n";
    std::cout << "\tBuffer size: " << static_cast<int>(buffer) << "\n";
    std::cout << "\tstate: "
              << (state ? "was already initialized" : "just initialized")
              << "\n";
  } else {
    std::cout << "Failed to read device information."
              << "\n";
    std::cout << explainCommStatus().str().c_str();
  }
  return status_.comm == CommunicationStatus::ok;
}

/**
 * @brief get the current hardware address
 *
 * Warning: only one slave should be connected on the communication line
 *
 * @param cur_address current address (output)
 * @return bool success flag
 */
bool KellerSensorDriver::getHardwareAddress(int* cur_address) {
  dev_address_ = 250;
  uint8_t address = 0;
  F66(0, &address);
  *cur_address = static_cast<int>(address);
  return status_.comm == CommunicationStatus::ok;
}

/**
 * @brief Change the device's (hardware) address
 *
 * @param new_address desired device's address, should be in range [1, 255]
 * but not 250
 * @return bool success flag
 */
bool KellerSensorDriver::setHardwareAddress(uint8_t new_address) {
  uint8_t dummy_address;
  F66(new_address, &dummy_address);
  if (status_.comm == CommunicationStatus::ok) {
    dev_address_ = new_address;
    std::cout << "Device address has been set to: " << new_address << "\n";
  } else {
    std::cout << "Setting the device address failed"
              << "\n";
    std::cout << explainCommStatus().str().c_str();
  }
  return status_.comm == CommunicationStatus::ok;
}

/**
 * @brief Change the device's baud rate
 *
 * @param new_baud desired baud rate, 9600 or 115200
 * @return bool success flag
 */
bool KellerSensorDriver::setHardwareBaud(int new_baud) {
  speed_t converted_baud = pSerial_->convertBaudForTermios(new_baud);
  if (converted_baud == 0) {
    status_.comm = CommunicationStatus::invalid_baud;
    return false;
  }
  if (pSerial_->getBaud() != converted_baud) {
    uint8_t uart_config;
    uint8_t new_uart_config;
    F32(10, &uart_config);
    new_uart_config = uart_config ^ ((uint8_t)1);
    F33(10, new_uart_config);
    // reinitialise after changing baud rate
    pSerial_->closeCommPort();
    if (!checkIfSerialIsEmulated()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      std::cout << "Waiting for restart..."
                << "\n";
    }
    switch (converted_baud) {
      case B9600:
        response_time_fix_ = constants::OPTIMAL_TIME_FIX_B9600;
        response_time_per_byte_ = constants::OPTIMAL_TIME_PER_BYTE_B9600;
        break;
      case B115200:
        response_time_fix_ = constants::OPTIMAL_TIME_FIX_B115200;
        response_time_per_byte_ = constants::OPTIMAL_TIME_PER_BYTE_B115200;
        break;
    }
    pSerial_->setBaud(new_baud);
    pSerial_->openCommPort();
    initializeHardware();
  } else {
    std::cout << "Not setting the baud, since it is already " << new_baud
              << "\n";
  }
  return status_.comm == CommunicationStatus::ok;
}

/**
 * @brief Return the the meaning of the current status_.comm variable as
 * stringstream.
 *
 * @return Stringstream with current communication status.
 */
std::stringstream KellerSensorDriver::explainCommStatus() {
  std::stringstream ss;
  ss << "Communication status: ";
  switch (status_.comm) {
    case CommunicationStatus::ok:
      ss << "No Errors\n";
      break;
    case CommunicationStatus::error:
      ss << "RX Error\n";
      break;
    case CommunicationStatus::bad_exception:
      ss << "Bad exception\n";
      break;
    case CommunicationStatus::bad_address:
      ss << "Bad address\n";
      break;
    case CommunicationStatus::bad_crc:
      ss << "Bad CRC\n";
      break;
    case CommunicationStatus::bad_rx_length:
      ss << "Bad length\n";
      break;
    case CommunicationStatus::bad_response:
      ss << "Bad response\n";
      break;
    case CommunicationStatus::no_response:
      ss << "No response\n";
      break;
    case CommunicationStatus::fail_read:
      ss << "Couldn't read\n";
      break;
    case CommunicationStatus::bad_tx_length:
      ss << "Wrong number of bytes written\n";
      break;
    case CommunicationStatus::fail_write:
      ss << "Couldn't write\n";
      break;
    case CommunicationStatus::fail_config:
      ss << "Port configuration failed\n";
      break;
    case CommunicationStatus::fail_open_port:
      ss << "Couldn't open serial port\n";
      break;
    case CommunicationStatus::cancel:
      ss << "Communication cancel (timeout)\n";
      break;
    case CommunicationStatus::invalid_baud:
      ss << "Baud rate invalid\n";
      break;
    case CommunicationStatus::exception:
      ss << "Unrecognized exception\n";
      break;
    case CommunicationStatus::exception_1:
      ss << "Illegal non-implemented function exception\n";
      break;
    case CommunicationStatus::exception_2:
      ss << "Illegal data address exception\n";
      break;
    case CommunicationStatus::exception_3:
      ss << "Illegal data value exception\n";
      break;
    case CommunicationStatus::exception_4:
      ss << "Slave device failure exception\n";
      break;
    case CommunicationStatus::exception_32:
      ss << "Not initialized exception\n";
      break;
  }
  ss << "\n";

  return ss;
}

/**
 * @brief Return the meaning of the current status_.meas variable as
 * stringstream.
 *
 * @return Stringstream with current measurement status.
 */
std::stringstream KellerSensorDriver::explainMeasStatus() {
  std::stringstream ss;
  ss << "Measurement status: ";

  switch (status_.meas) {
    case MeasurementStatus::valid:
      ss << "No Errors, valid measurement\n";
      break;
    case MeasurementStatus::not_available:
      ss << "No measurement received! ";
      ss << explainCommStatus().str().c_str();
      break;
    case MeasurementStatus::bad_status_powerup:
      ss << "Sensor was in power-up state when "
         << "measurement request was made\n";
      break;
    case MeasurementStatus::bad_status_saturation:
      ss << "Analogue signal saturated\n";
      break;
    case MeasurementStatus::bad_status_temperature:
      ss << "Temperature Channel error\n";
      break;
    case MeasurementStatus::bad_status_pressure:
      ss << "Pressure channel error\n";
      break;
  }

  return ss;
}

float KellerSensorDriver::getMeasurement(MeasurementType measurement_type) {
  uint8_t channel;
  uint8_t status_byte;
  uint8_t power_up_check = 0b10000000;
  uint8_t saturation_check = 0b01000000;
  uint8_t temp_check = 0b00010000;
  uint8_t pressure_check = 0b00000010;
  float preliminary_measurement;
  int32_t preliminary_measurement_int;

  // Prepare the channel and status checks
  switch (measurement_type) {
    case MeasurementType::pressure:
    case MeasurementType::pressure_quant:
      channel = 1;
      break;
    case MeasurementType::temperature:
    case MeasurementType::temperature_quant:
      // when only measuring temperature, pressure status is irrelevant
      pressure_check = 0;
      channel = 4;
      break;
    default:
      std::cout << "Unknown Measurement Type"
                << "\n";
  }

  // Perform the measurement
  switch (measurement_type) {
    case MeasurementType::pressure:
    case MeasurementType::temperature:
      F73(channel, &preliminary_measurement, &status_byte);
      break;
    case MeasurementType::pressure_quant:
    case MeasurementType::temperature_quant:
      F74(channel, &preliminary_measurement_int, &status_byte);
      break;
    default:
      std::cout << "Unknown Measurement Type"
                << "\n";
  }
  if (status_.comm != CommunicationStatus::ok) {
    status_.meas = MeasurementStatus::not_available;
    return std::nanf("");
  }

  // Check the status and identify problems
  if ((status_byte & power_up_check) != 0) {
    status_.meas = MeasurementStatus::bad_status_powerup;
    return std::nanf("");
  }

  if ((status_byte & saturation_check) != 0) {
    status_.meas = MeasurementStatus::bad_status_saturation;
    return std::nanf("");
  }

  if ((status_byte & temp_check) != 0) {
    status_.meas = MeasurementStatus::bad_status_temperature;
    return std::nanf("");
  }

  if ((status_byte & pressure_check) != 0) {
    status_.meas = MeasurementStatus::bad_status_pressure;
    return std::nanf("");
  }

  // Scale the quantized measurements
  switch (measurement_type) {
    case MeasurementType::pressure:
      break;
    case MeasurementType::pressure_quant:
      preliminary_measurement = preliminary_measurement_int / 1e5f;
      break;
    case MeasurementType::temperature:
      break;
    case MeasurementType::temperature_quant:
      preliminary_measurement = preliminary_measurement_int / 100.0f;
      break;
    default:
      std::cout << "Unknown Measurement Type"
                << "\n";
  }
  status_.meas = MeasurementStatus::valid;

  return preliminary_measurement;
}

/**
 * @brief Get one pressure measurement from the device
 *
 * @return float measurement
 */
float KellerSensorDriver::getPressureMeasurement() {
  return getMeasurement(MeasurementType::pressure);
}

/**
 * @brief Get one pressure measurement from the device by reading it as an int
 *
 * Pressure is then quantized down to 1 Pa
 *
 * @return float measurement
 */
float KellerSensorDriver::getPressureMeasurementQuant() {
  return getMeasurement(MeasurementType::pressure_quant);
}

/**
 * @brief Get one temperature measurement from the device
 *
 * @return float measurement
 */
float KellerSensorDriver::getTemperatureMeasurement() {
  return getMeasurement(MeasurementType::temperature);
}

/**
 * @brief Get one temperature measurement from device by reading it as an int
 *
 * Temperature is then quantized down to 0.01 degC
 *
 * @return float measurement
 */
float KellerSensorDriver::getTemperatureMeasurementQuant() {
  return getMeasurement(MeasurementType::temperature_quant);
}

/**
 * @brief checks is the used serial port is emulated or real
 *
 * @return true serial port is emulated
 * @return false serial port is real
 */
bool KellerSensorDriver::checkIfSerialIsEmulated() {
  return pSerial_->isEmulated();
}

/**
 * @brief Requst to restart the sensor or mimic the restart
 * results in an uninitialized sensor
 */
void KellerSensorDriver::restartDevice() {
  pSerial_->restartDevice();
}

/**
 * @brief getter for the baud rate stored in the associated serial
 *
 * @return int baud rate (human readable)
 */
int KellerSensorDriver::getSoftwareBaud() {
  return pSerial_->convertBaudForHuman(pSerial_->getBaud());
}

/**
 * @brief getter for the class member dev_address
 *
 * @return uint8_t current address
 */
uint8_t KellerSensorDriver::getSoftwareAddress() {
  return dev_address_;
}

/**
 * @brief setter for the class member dev_address
 *
 * @param address desired address
 */
void KellerSensorDriver::setSoftwareAddress(uint8_t address) {
  dev_address_ = address;
}

/**
 * @brief calculates the CRC16 checksum for the provided data
 *
 * @param Data data buffer address
 * @param nCnt number of bytes to analyze for the checksum
 * @return uint16_t CRC16 checksum
 */
uint16_t KellerSensorDriver::calcCrc16(const uint8_t* Data, uint16_t nCnt) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < nCnt; ++i) {
    crc ^= *Data++;
    for (int n = 0; n < 8; ++n) {
      bool b = crc & 1;
      crc >>= 1;
      if (b) {
        crc ^= 0xA001;
      }
    }
  }
  return crc;
}

/**
 * @brief wrapper around data tranfer routine
 *
 * @param nTX number of bytes to transfer from the TX buffer
 * @param nRX number of bytes to read from the RX buffer
 */
void KellerSensorDriver::transferData(uint16_t nTX, uint16_t nRX) {
  std::unique_lock<std::mutex> lock(*pCancelMutex_);
  if (pCancelVar_->wait_for(lock, std::chrono::milliseconds(0)) !=
      std::cv_status::timeout)
    status_.comm = CommunicationStatus::cancel;
  else
    status_.comm = transferDataDirect(nTX, nRX);

  if (pCancelVar_->wait_for(lock, std::chrono::milliseconds(0)) !=
      std::cv_status::timeout)
    status_.comm = CommunicationStatus::cancel;
}

/**
 * @brief core data transfer and reception routine.
 *
 * @param nTX number of bytes to transfer from the TX buffer
 * @param nRX number of bytes to read from the RX buffer
 * @return CommunicationStatus comm_status
 */
CommunicationStatus KellerSensorDriver::transferDataDirect(uint16_t nTX,
                                                           uint16_t nRX) {
  int16_t bytes_written, bytes_read;
  uint16_t crc;

  // 1. Add CRC16-checksum
  crc = calcCrc16(tx_buffer_, nTX);
  tx_buffer_[nTX++] = crc >> 8;    // HIBYTE(crc);
  tx_buffer_[nTX++] = crc & 0xFF;  // LOBYTE(crc);

  // // debug logging
  // std::cout << "\ntx_buffer_ with " << nTX << " bytes: \n";
  // for (int i = 0; i < nTX; i++)
  //   std::cout << static_cast<int>(tx_buffer_[i]) << " ";
  // std::cout << "\n";

  // 2. Send
  bytes_written = pSerial_->sendData(tx_buffer_, nTX);
  if (bytes_written < 0)
    return CommunicationStatus::fail_write;
  if (bytes_written != nTX)
    return CommunicationStatus::bad_tx_length;

  // Wait for the slave to respond
  if (response_time_fix_ > 0 || response_time_per_byte_ > 0)
    std::this_thread::sleep_for(std::chrono::milliseconds(
        static_cast<int>(response_time_fix_ + nRX * response_time_per_byte_)));

  // 3. Receive
  bytes_read = pSerial_->receiveData(rx_buffer_, nRX);
  if (bytes_read < 0)
    return CommunicationStatus::fail_read;

  // // debug logging
  // std::cout << "rx_buffer_ with " << bytes_read << " bytes: \n";
  // for (int i = 0; i < bytes_read; i++)
  //   std::cout << static_cast<int>(rx_buffer_[i]) << " ";
  // std::cout << "\n";

  // 4. Check for response
  if (bytes_read == 0)
    return CommunicationStatus::no_response;

  // 6. Exception?
  if (tx_buffer_[1] + 128 == rx_buffer_[1]) {
    return parseException();
  }

  // 7. Check length of answer
  if (bytes_read != nRX)
    return CommunicationStatus::bad_rx_length;

  // 8. Check CRC16
  crc = calcCrc16(&rx_buffer_[0], bytes_read - 2);
  if ((rx_buffer_[bytes_read - 2] != (crc >> 8)) ||
      (rx_buffer_[bytes_read - 1] != (crc & 0xFF)))
    return CommunicationStatus::bad_crc;

  // 9. Check device address
  if (tx_buffer_[0] != rx_buffer_[0])
    return CommunicationStatus::bad_address;

  // 10. Function ok?
  if (tx_buffer_[1] == rx_buffer_[1])
    return CommunicationStatus::ok;

  // 11. General error
  return CommunicationStatus::error;
}

/**
 * @brief Checks which kind of exception was received in RX buffer
 *
 * @return CommunicationStatus communication status
 */
CommunicationStatus KellerSensorDriver::parseException() {
  CommunicationStatus exception_type;

  // Check CRC16, reply with bad exception in case of error to avoid confusion
  uint16_t crc = calcCrc16(&rx_buffer_[0], 3);
  if ((rx_buffer_[3] != (crc >> 8)) || (rx_buffer_[4] != (crc & 0xFF))) {
    std::cout << "Bad CRC in Exception"
              << "\n";
    return CommunicationStatus::bad_exception;
  }

  // Check device address, reply with bad exception in case of error to
  // avoid confusion
  if (tx_buffer_[0] != rx_buffer_[0]) {
    std::cout << "Bad Address in Exception"
              << "\n";
    return CommunicationStatus::bad_exception;
  }

  switch (rx_buffer_[2]) {
    case 1:
      exception_type = CommunicationStatus::exception_1;
      break;
    case 2:
      exception_type = CommunicationStatus::exception_2;
      break;
    case 3:
      exception_type = CommunicationStatus::exception_3;
      break;
    case 4:
      exception_type = CommunicationStatus::exception_4;
      break;
    case 32:
      exception_type = CommunicationStatus::exception_32;
      break;
    default:
      exception_type = CommunicationStatus::exception;
      break;
  }

  return exception_type;
}

/**
 * @brief Read the coefficient at a given address
 *
 * Coefficients provide a way to access device information (read-only) and
 * calibration state (read and write)
 *
 * @param coeff_address address (No.) of the coefficient
 * @param coeff_value desired storage variable
 */
void KellerSensorDriver::F30(uint8_t coeff_address, float* coeff_value) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 30;
  tx_buffer_[2] = coeff_address;
  transferData(3, 8);
  if (status_.comm == CommunicationStatus::ok) {
    // memcpy(coeff_value, &rx_buffer_[7], sizeof(float));
    endianness::copyFromBigEndian(coeff_value, &rx_buffer_[2], sizeof(float));
    // *coeff_value = ReadFloatToLittleEndian(&rx_buffer_[7]);
  }
}

/**
 * @brief Write the coefficient at a given address
 *
 * Coefficients provide a way to access device information (read-only) and
 * calibration state (read and write)
 *
 * @param coeff_address address (No.) of the coefficient
 * @param coeff_value desired value
 */
void KellerSensorDriver::F31(uint8_t coeff_address, float coeff_value) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 31;
  tx_buffer_[2] = coeff_address;

  memcpy(&tx_buffer_[3], &coeff_value, sizeof(float));

  transferData(7, 5);
}

/**
 * @brief Read the configuration at a given address
 *
 * Configuration values provide a way to access the configuration and
 * reconfigure the device
 *
 * @param config_address address (No.) of the configuration
 * @param config_value desired storage variable
 */
void KellerSensorDriver::F32(uint8_t config_address, uint8_t* config_value) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 32;
  tx_buffer_[2] = config_address;
  transferData(3, 5);
  if (status_.comm == CommunicationStatus::ok) {
    *config_value = rx_buffer_[2];
  }
}

/**
 * @brief Write the configuration at a given address
 *
 * Configuration values provide a way to access the configuration and
 * reconfigure the device
 *
 * @param config_address address (No.) of the configuration
 * @param config_value desired storage variable
 */
void KellerSensorDriver::F33(uint8_t config_address, uint8_t config_value) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 33;
  tx_buffer_[2] = config_address;
  tx_buffer_[3] = config_value;

  transferData(4, 5);
}

/**
 * @brief Initialize the hardware after a break in power supply and read the
 * initialization information
 *
 * @param device_class device ID code
 * @param group subdivision within a device class
 * @param year fimware version, year
 * @param week firmware version, week
 * @param buffer length of the internal RX buffer
 * @param state status information (1 if already initialised before, 0
 * otherwise)
 */
void KellerSensorDriver::F48(uint8_t* device_class, uint8_t* group,
                             uint8_t* year, uint8_t* week, uint8_t* buffer,
                             uint8_t* state) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 48;
  transferData(2, 10);
  if (status_.comm == CommunicationStatus::ok) {
    *device_class = rx_buffer_[2];
    *group = rx_buffer_[3];
    *year = rx_buffer_[4];
    *week = rx_buffer_[5];
    *buffer = rx_buffer_[6];
    *state = rx_buffer_[7];
  }
}

/**
 * @brief Change the device address or read the actual address
 *
 * After setting the device address, it is returned as act_address. To read the
 * current unknown address, set dev_address_ to 250 (transparent address) prior
 * to calling the function and pass new_address = 0. act_address will return the
 * current address.
 *
 * @param new_address desired device address (1..255, not 250)
 * @param act_address storage variable for the current address
 */
void KellerSensorDriver::F66(uint8_t new_address, uint8_t* act_address) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 66;
  tx_buffer_[2] = new_address;
  transferData(3, 5);
  if (status_.comm == CommunicationStatus::ok) {
    *act_address = rx_buffer_[2];
  }
}

/**
 * @brief Read the device serial number
 *
 * @param serial_number storage variable for the serial number
 */
void KellerSensorDriver::F69(uint32_t* serial_number) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 69;
  transferData(2, 8);
  if (status_.comm == CommunicationStatus::ok) {
    memcpy(serial_number, &rx_buffer_[2], sizeof(uint32_t));
  }
}

/**
 * @brief Read the value of Channel as a float
 *
 * @param channel channel
 * @param value storage variable
 * @param status status byte reporting errors during the measurement
 */
void KellerSensorDriver::F73(uint8_t channel, float* value, uint8_t* status) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 73;
  tx_buffer_[2] = channel;
  transferData(3, 9);
  if (status_.comm == CommunicationStatus::ok) {
    endianness::copyFromBigEndian(value, &rx_buffer_[2], sizeof(float));
    *status = rx_buffer_[6];
  }
}

/**
 * @brief Read the value of Channel as a 32bit integer
 *
 * @param channel channel
 * @param value storage variable
 * @param status status byte reporting errors during the measurement
 */
void KellerSensorDriver::F74(uint8_t channel, int32_t* value, uint8_t* status) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 74;
  tx_buffer_[2] = channel;
  transferData(3, 9);
  if (status_.comm == CommunicationStatus::ok) {
    endianness::copyFromBigEndian(value, &rx_buffer_[2], sizeof(int32_t));
    *status = rx_buffer_[6];
  }
}

/**
 * @brief Set zero point such that the current measured value is 0
 *
 * @param command command to specify the channel and setting/resetting
 */
void KellerSensorDriver::F95(uint8_t command) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 95;
  tx_buffer_[2] = command;

  transferData(3, 5);
}

/**
 * @brief Set zero point such that the current measured value is as specified
 *
 * @param command command to specify the channel and setting/resetting
 * @param value desired value to be measured
 */
void KellerSensorDriver::F95_Val(uint8_t command, float value) {
  tx_buffer_[0] = dev_address_;
  tx_buffer_[1] = 95;
  tx_buffer_[2] = command;

  memcpy(&tx_buffer_[3], &value, sizeof(float));

  transferData(7, 5);
}

}  // namespace keller
