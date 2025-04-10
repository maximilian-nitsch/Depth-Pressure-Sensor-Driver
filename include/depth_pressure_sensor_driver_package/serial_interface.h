/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <termios.h>
#include <cstdint>
#include <string>

#include "depth_pressure_sensor_driver_constants.h"

namespace keller {

class SerialInterface {
 public:
  virtual bool isEmulated() = 0;
  virtual void restartDevice() = 0;
  virtual CommunicationStatus openCommPort() = 0;
  virtual void closeCommPort() = 0;
  virtual int16_t sendData(const uint8_t* tx_buffer, uint16_t nTX) = 0;
  virtual int16_t receiveData(uint8_t* rx_buffer, uint16_t nRX) = 0;

  speed_t convertBaudForTermios(int baud);
  int convertBaudForHuman(speed_t baud);
  speed_t getBaud();
  void setBaud(int baud);
  std::string getPortName();
  void setPortName(const std::string& port_name);

 protected:
  std::string port_name_;
  int port_file_descriptor_;
  speed_t baud_rate_;
};

}  // namespace keller
