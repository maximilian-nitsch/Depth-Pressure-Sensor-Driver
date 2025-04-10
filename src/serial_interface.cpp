/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#include "serial_interface.h"

namespace keller {

/**
 * @brief function to convert baud rates from human readable format to the
 * definitions of termios.h. For unsupported baud rates, returns 0.
 * 
 * @param baud human-readable baud rate, e.g. 9600
 * @return speed_t corresponding baud rate, understandable for termios.h
 */
speed_t SerialInterface::convertBaudForTermios(int baud) {
  switch (baud) {
    case 9600:
      return B9600;
      break;
    case 115200:
      return B115200;
      break;
    default:
      return 0;
  }
}

/**
 * @brief function to convert baud rates from termios format to human readable
 * format. For unsupported baud rates, returns 0.
 * 
 * @param baud baud rate in termios format, e.g. B9600
 * @return speed_t corresponding baud rate in human readable form
 */
int SerialInterface::convertBaudForHuman(speed_t baud) {
  switch (baud) {
    case B9600:
      return 9600;
      break;
    case B115200:
      return 115200;
      break;
    default:
      return 0;
  }
}

/**
 * @brief getter for the baud rate
 * 
 * @return speed_t baud rate
 */
speed_t SerialInterface::getBaud() {
  return baud_rate_;
}

/**
 * @brief setter for the baud rate
 * 
 * @param baud baud rate
 */
void SerialInterface::setBaud(int baud) {
  baud_rate_ = convertBaudForTermios(baud);
}

/**
 * @brief getter for the port name
 * 
 * @return std::string port name
 */
std::string SerialInterface::getPortName() {
  return port_name_;
}

/**
 * @brief setter for the port name
 * 
 * @param port_name port name
 */
void SerialInterface::setPortName(const std::string& port_name) {
  port_name_ = port_name;
}

}  // namespace keller
