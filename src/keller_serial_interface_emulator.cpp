/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#include <cstring>
#include <iostream>

#include "endianness.h"
#include "keller_serial_interface_emulator.h"

namespace keller {

/**
 * @brief Construct a new KellerSerialInterfaceEmulator object
 *
 * @param port_name communication port name, e.g. /dev/ttyUSB0
 * @param baud_rate current baud rate. 9600 or 115200. Default: 9600
 */
KellerSerialInterfaceEmulator::KellerSerialInterfaceEmulator(
    const std::string& port_name, int baud_rate) {
  this->setPortName(port_name);
  this->setBaud(baud_rate);
  this->dev_address_ = 5;
  this->device_initialized_ = true;
}

/**
 * @brief return if the serial interface is emulated
 *
 * @return true
 */
bool KellerSerialInterfaceEmulator::isEmulated() {
  return true;
}

/**
 * @brief set device_initialized_ to false,
 * mimicking restart (power off, then back on)
 */
void KellerSerialInterfaceEmulator::restartDevice() {
  device_initialized_ = false;
}

/**
 * @brief opens a mock version of a communication port
 *
 * @return CommunicationStatus comm_status
 */
CommunicationStatus KellerSerialInterfaceEmulator::openCommPort() {
  std::cout << "Opening a comm port " << port_name_ << " at a baud rate "
            << convertBaudForHuman(baud_rate_) << std::endl;

  // Close the port if already open
  closeCommPort();

  // Open the serial port
  port_file_descriptor_ = 0;
  std::cout << "Opening a port emulator with a descriptor "
            << port_file_descriptor_ << std::endl;

  return CommunicationStatus::ok;
}

/**
 * @brief closes the current communication port
 */
void KellerSerialInterfaceEmulator::closeCommPort() {
  if (port_file_descriptor_ >= 0) {
    port_file_descriptor_ = -1;
  }
}

/**
 * @brief emulate sending data to the sensor
 *
 * Emulation means no actual sending is happening.
 * Instead, extract the function code from the tx_buffer and save it.
 * Also, save the entire tx buffer.
 * Perform checks of device address, crc16 and message length.
 * Return nTX
 *
 * @param tx_buffer tx buffer
 * @param nTX number of bytes to be sent
 * @return int16_t number of bytes sent = nTX
 */
int16_t KellerSerialInterfaceEmulator::sendData(const uint8_t* tx_buffer,
                                                uint16_t nTX) {
  // Extract device address, function code, save the tx message if needed later
  dev_address_correct_ = tx_buffer[0] == dev_address_ || tx_buffer[0] == 250;
  func_code_ = tx_buffer[1];
  memcpy(tx_buffer_, tx_buffer, nTX);
  nTX_ = nTX;

  // Check function code and message length
  func_code_correct_ = true;
  length_correct_ = false;
  switch (func_code_) {
    case 30:
      length_correct_ = (nTX == 5);
      break;
    case 31:
      length_correct_ = (nTX == 9);
      break;
    case 32:
      length_correct_ = (nTX == 5);
      break;
    case 33:
      length_correct_ = (nTX == 6);
      break;
    case 48:
      length_correct_ = (nTX == 4);
      break;
    case 66:
      length_correct_ = (nTX == 5);
      break;
    case 69:
      length_correct_ = (nTX == 4);
      break;
    case 73:
      length_correct_ = (nTX == 5);
      break;
    case 74:
      length_correct_ = (nTX == 5);
      break;
    case 95:
      length_correct_ = (nTX == 5 || nTX == 9);
      break;
    default:
      func_code_correct_ = false;
      break;
  }

  // Check CRC
  uint16_t crc_calculated = calcCrc16(tx_buffer, nTX - 2);
  uint16_t crc_sent = 0;
  endianness::copyFromBigEndian(&crc_sent, tx_buffer + nTX - 2, 2);
  crc_correct_ = crc_sent == crc_calculated;

  int16_t bytes_written = static_cast<int16_t>(nTX);
  return bytes_written;
}

/**
 * @brief emulate receiving data from the sensor
 *
 * @param rx_buffer rx buffer
 * @param nRX expected length of received message
 * @return int16_t number of received bytes
 */
int16_t KellerSerialInterfaceEmulator::receiveData(uint8_t* rx_buffer,
                                                   uint16_t nRX) {
  // unused variable; suppress compiler warning
  (void)nRX;

  // Message analysis: cases where no response should be given
  if (!length_correct_)
    return 0;
  if (!dev_address_correct_)
    return 0;
  if (!crc_correct_)
    return 0;

  // Exception analysis
  // response starts with the transmitted address, in case it was 250
  rx_buffer[0] = tx_buffer_[0];
  rx_buffer[1] = func_code_;
  uint16_t rx_length = 2;
  if (!func_code_correct_)
    rx_length += throwException(rx_buffer, 1);
  if (!device_initialized_ && func_code_ != 48)
    rx_length += throwException(rx_buffer, 32);

  // no exception occured
  if (rx_length == 2) {
    // Creating the response payload
    switch (func_code_) {
      case 30:
        rx_length += handleF30(rx_buffer);
        break;
      case 31:
        rx_length += handleF31(rx_buffer);
        break;
      case 32:
        rx_length += handleF32(rx_buffer);
        break;
      case 33:
        rx_length += handleF33(rx_buffer);
        break;
      case 48:
        rx_length += handleF48(rx_buffer);
        break;
      case 66:
        rx_length += handleF66(rx_buffer);
        break;
      case 69:
        rx_length += handleF69(rx_buffer);
        break;
      case 73:
        rx_length += handleF73(rx_buffer);
        break;
      case 74:
        rx_length += handleF74(rx_buffer);
        break;
      case 95:
        rx_length += handleF95(rx_buffer);
        break;
    }

    // Add crc16
    uint16_t crc = calcCrc16(rx_buffer, rx_length);
    rx_buffer[rx_length++] = crc >> 8;    // HIBYTE(crc);
    rx_buffer[rx_length++] = crc & 0xFF;  // LOBYTE(crc);
  }

  int16_t bytes_read = static_cast<int16_t>(rx_length);
  return bytes_read;
}

/**
 * @brief setter for dev_address_
 *
 * @param dev_address emulated device address
 */
void KellerSerialInterfaceEmulator::setDeviceAddress(uint8_t dev_address) {
  dev_address_ = dev_address;
}

/**
 * @brief getter for dev_address_
 *
 * @return uint8_t emulated device address
 */
uint8_t KellerSerialInterfaceEmulator::getDeviceAddress() {
  return dev_address_;
}

/**
 * @brief calculates the CRC16 checksum for the provided data
 *
 * @param Data data buffer address
 * @param nCnt number of bytes to analyze for the checksum
 * @return uint16_t CRC16 checksum
 */
uint16_t KellerSerialInterfaceEmulator::calcCrc16(const uint8_t* Data,
                                                  uint16_t nCnt) {
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
 * @brief fill the rx buffer with the exception message
 *
 * @param rx_buffer rx buffer
 * @param ex_code exception code
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::throwException(uint8_t* rx_buffer,
                                                       uint8_t ex_code) {
  rx_buffer[1] += 128;
  rx_buffer[2] = ex_code;
  uint16_t crc = calcCrc16(rx_buffer, 3);
  rx_buffer[3] = crc >> 8;    // HIBYTE(crc);
  rx_buffer[4] = crc & 0xFF;  // LOBYTE(crc);
  return 3;
}

/**
 * @brief Return a dummy coefficient
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF30(uint8_t* rx_buffer) {
  rx_buffer[2] = 0;
  rx_buffer[3] = 0;
  rx_buffer[4] = 0;
  rx_buffer[5] = 0;
  return 4;
}

/**
 * @brief Write a dummy coefficient = do nothing
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF31(uint8_t* rx_buffer) {
  rx_buffer[2] = 0;
  return 1;
}

/**
 * @brief Return a dummy configuration
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF32(uint8_t* rx_buffer) {
  if (tx_buffer_[2] == 10) {  // UART settings
    uint8_t uart_config = 0;
    uart_config |= ((uint8_t)(baud_rate_ == B115200 ? 1 : 0));
    rx_buffer[2] = uart_config;
  } else {
    rx_buffer[2] = 0;
  }
  return 1;
}

/**
 * @brief Write a dummy configuration = do nothing
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF33(uint8_t* rx_buffer) {
  if (tx_buffer_[2] == 10) {             // UART settings
    if (tx_buffer_[3] & ((uint8_t)1)) {  // Setting baud to 115200
      baud_rate_ = convertBaudForTermios(115200);
    } else {  // Setting baud to 9600
      baud_rate_ = convertBaudForTermios(9600);
    }
    restartDevice();
  }
  rx_buffer[2] = 0;
  return 1;
}

/**
 * @brief Fill the rx buffer with hardcoded device settings and set the device
 * as initialized
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF48(uint8_t* rx_buffer) {
  rx_buffer[2] = 5;
  rx_buffer[3] = 24;
  rx_buffer[4] = 22;
  rx_buffer[5] = 27;
  rx_buffer[6] = 255;
  rx_buffer[7] = device_initialized_;
  device_initialized_ = 1;
  return 6;
}

/**
 * @brief Set a new emulated device address or return the current address
 *
 * @param rx_buffer rx buffer
 * @return uint16_t  number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF66(uint8_t* rx_buffer) {
  uint8_t new_address = tx_buffer_[2];
  // return current address
  if (tx_buffer_[0] == 250 && new_address == 0) {
    rx_buffer[2] = dev_address_;
    return 1;
  }
  // set new address
  if (new_address >= 1 && new_address != 250) {
    dev_address_ = new_address;
    rx_buffer[2] = dev_address_;
    return 1;
  }
  // bad input -> no response
  return 0;
}

/**
 * @brief Return a dummy serial number
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF69(uint8_t* rx_buffer) {
  rx_buffer[2] = 0;
  rx_buffer[3] = 0;
  rx_buffer[4] = 0;
  rx_buffer[5] = 0;
  return 4;
}

/**
 * @brief Read a dummy value of a channel
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF73(uint8_t* rx_buffer) {
  float value;
  switch (tx_buffer_[2]) {
    case 1:  // P1 channel
      value = -0.0042;
      endianness::storeAsBigEndian(&rx_buffer[2], &value, sizeof(float));
      break;
    case 4:  // TOB1 channel
      value = 24.42;
      endianness::storeAsBigEndian(&rx_buffer[2], &value, sizeof(float));
      break;
    case 0:
    case 2:
    case 3:
    case 5:
    case 6:
    case 7:
    case 8:  // channel doesn't exist -> output NaN
      rx_buffer[2] = 0xFF;
      rx_buffer[3] = 0xFF;
      rx_buffer[4] = 0xFF;
      rx_buffer[5] = 0xFF;
      break;
    case 9:  // conductivity channel? -> send 0s
      rx_buffer[2] = 0x00;
      rx_buffer[3] = 0x00;
      rx_buffer[4] = 0x00;
      rx_buffer[5] = 0x00;
      break;
    default:  // channel invalid -> throw exception
      return throwException(rx_buffer, 2);
  }
  rx_buffer[6] = 0;
  return 5;
}

/**
 * @brief Read a dummy value of a channel
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF74(uint8_t* rx_buffer) {
  uint32_t value;
  switch (tx_buffer_[2]) {
    case 1:          // P1 channel
      value = -420;  // -0.0042 * 10000
      endianness::storeAsBigEndian(&rx_buffer[2], &value, sizeof(uint32_t));
      break;
    case 4:          // TOB1 channel
      value = 2442;  // 24.24 * 100
      endianness::storeAsBigEndian(&rx_buffer[2], &value, sizeof(uint32_t));
      break;
    case 0:
    case 2:
    case 3:
    case 5:
    case 6:  // channel doesn't exist -> output max value
      value = 0x7fffffff;
      endianness::storeAsBigEndian(&rx_buffer[2], &value, sizeof(uint32_t));
      break;
    default:  // channel invalid -> throw exception
      return throwException(rx_buffer, 2);
  }
  rx_buffer[6] = 0;
  return 5;
}

/**
 * @brief Return a dummy confirmation that some non-existing zero point
 * has been set
 *
 * @param rx_buffer rx buffer
 * @return uint16_t number of bytes added to rx
 */
uint16_t KellerSerialInterfaceEmulator::handleF95(uint8_t* rx_buffer) {
  rx_buffer[2] = 0;
  return 1;
}

}  // namespace keller
