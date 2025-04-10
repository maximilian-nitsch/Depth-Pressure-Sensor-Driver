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

class KellerSerialInterface : public SerialInterface {
 public:
  explicit KellerSerialInterface(const std::string& port, int baud_rate = 9600);
  ~KellerSerialInterface();
  bool isEmulated() override;
  void restartDevice() override;
  CommunicationStatus openCommPort() override;
  void closeCommPort() override;
  int16_t sendData(const uint8_t* tx_buffer, uint16_t nTX) override;
  int16_t receiveData(uint8_t* rx_buffer, uint16_t nRX) override;
};

}  // namespace keller
