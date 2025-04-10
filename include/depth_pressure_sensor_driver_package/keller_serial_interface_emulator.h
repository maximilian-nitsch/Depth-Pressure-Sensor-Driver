/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <cstdint>
#include <string>

#include "serial_interface.h"

namespace keller {

/**
 * @brief This class emulates the behavior of a keller 10LHPX pressure sensor
 * connected to the current computer via a serial interface
 *
 */
class KellerSerialInterfaceEmulator : public SerialInterface {
 public:
  explicit KellerSerialInterfaceEmulator(const std::string& port,
                                         int baud_rate = 9600);
  bool isEmulated() override;
  void restartDevice();
  CommunicationStatus openCommPort() override;
  void closeCommPort() override;
  int16_t sendData(const uint8_t* tx_buffer, uint16_t nTX) override;
  int16_t receiveData(uint8_t* rx_buffer, uint16_t nRX) override;
  void setDeviceAddress(uint8_t dev_address);
  uint8_t getDeviceAddress();

 private:
  uint16_t calcCrc16(const uint8_t* Data, uint16_t nCnt);
  uint16_t throwException(uint8_t* rx_buffer, uint8_t ex_code);
  uint16_t handleF30(uint8_t* rx_buffer);
  uint16_t handleF31(uint8_t* rx_buffer);
  uint16_t handleF32(uint8_t* rx_buffer);
  uint16_t handleF33(uint8_t* rx_buffer);
  uint16_t handleF48(uint8_t* rx_buffer);
  uint16_t handleF66(uint8_t* rx_buffer);
  uint16_t handleF69(uint8_t* rx_buffer);
  uint16_t handleF73(uint8_t* rx_buffer);
  uint16_t handleF74(uint8_t* rx_buffer);
  uint16_t handleF95(uint8_t* rx_buffer);

  uint8_t tx_buffer_[constants::MAX_TX_LENGTH];
  uint8_t nTX_;
  uint8_t dev_address_;
  uint8_t func_code_;
  bool device_initialized_;
  bool dev_address_correct_;
  bool func_code_correct_;
  bool length_correct_;
  bool crc_correct_;
};

}  // namespace keller
