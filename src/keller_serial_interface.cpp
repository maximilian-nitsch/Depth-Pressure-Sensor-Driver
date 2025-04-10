/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Dmitrii Likhachev (dmitrii.likhachev@rwth-aachen.de)
All rights reserved.
*/

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
// NOLINTNEXTLINE(build/c++11)
#include <thread>

#include "keller_serial_interface.h"

namespace keller {

/**
 * @brief Construct a new KellerSerialInterface object
 *
 * @param port_name communication port name, e.g. /dev/ttyUSB0
 * @param baud_rate current baud rate. 9600 or 115200. Default: 9600
 */
KellerSerialInterface::KellerSerialInterface(const std::string& port_name,
                                             int baud_rate) {
  this->setPortName(port_name);
  this->setBaud(baud_rate);
}

/**
 * @brief Destroy the serial interface. Close the communication port before it.
 * 
 */
KellerSerialInterface::~KellerSerialInterface() {
  closeCommPort();
}

/**
 * @brief return if the serial interface is emulated
 * 
 * @return false
 */
bool KellerSerialInterface::isEmulated() {
  return false;
}

/**
 * @brief request the user to restart the sensor
 */
void KellerSerialInterface::restartDevice() {
  std::cout << "\nRestart the device now!\n";
  std::cout << "Turn the power supply off and then back on\n";
  std::cout << "Before the timer runs out\n";
  for (int i = 5; i > 0; i--) {
    std::cout << "\tSeconds left: " << i << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
  return;
}

/**
 * @brief opens a communication port with default settings
 * 
 * @param port communication port name, e.g. /dev/ttyUSB0
 * @return CommunicationStatus comm_status
 */
CommunicationStatus KellerSerialInterface::openCommPort() {
  std::cout << "Opening a comm port " << port_name_ << " at a baud rate "
            << convertBaudForHuman(baud_rate_) << std::endl;

  // Close the port if already open
  closeCommPort();

  // Open the serial port
  // blocking version
  port_file_descriptor_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  // non-blocking version
  // port_file_descriptor_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (port_file_descriptor_ < 0) {
    return CommunicationStatus::fail_open_port;
  }
  std::cout << "Opened port with a descriptor " << port_file_descriptor_
            << std::endl;

  // Configure port settings
  std::cout << "Configuring the port settings" << std::endl;
  struct termios tty;
  memset(&tty, 0, sizeof tty);
  if (tcgetattr(port_file_descriptor_, &tty) != 0) {
    close(port_file_descriptor_);
    return CommunicationStatus::fail_config;
  }

  // Set baud rate
  std::cout << "Setting the baud rate" << std::endl;
  cfsetospeed(&tty, baud_rate_);
  cfsetispeed(&tty, baud_rate_);

  tty.c_cflag &= ~PARENB;  // No parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;             // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;        // No hardware flow control
  tty.c_cflag |= CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
  tty.c_oflag &= ~ONLCR;  // Prevent conversion of NL to CR-NL

  tty.c_cc[VTIME] = 10;  // Wait for up to 1s (10 deciseconds)
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(port_file_descriptor_, TCSANOW, &tty) != 0) {
    close(port_file_descriptor_);
    return CommunicationStatus::fail_config;
  }

  return CommunicationStatus::ok;
}

/**
 * @brief closes the current communication port
 */
void KellerSerialInterface::closeCommPort() {
  if (port_file_descriptor_ >= 0) {
    close(port_file_descriptor_);
    port_file_descriptor_ = -1;
  }
}

/**
 * @brief Write the bytes from tx buffer to the file descriptor port
 * 
 * @param tx_buffer tx buffer
 * @param nTX number of bytes to be sent
 * @return int16_t number of bytes successfully written to the port
 */
int16_t KellerSerialInterface::sendData(const uint8_t* tx_buffer,
                                        uint16_t nTX) {
  // std::cout << "Writing " << nTX << " bytes" << std::endl;
  int16_t bytes_written = write(port_file_descriptor_, tx_buffer, nTX);
  return bytes_written;
}

/**
 * @brief Read the bytes from the serial port
 * 
 * @param rx_buffer rx buffer
 * @param nRX expected of bytes
 * @return int16_t number of bytes successfully read from the port
 */
int16_t KellerSerialInterface::receiveData(uint8_t* rx_buffer, uint16_t nRX) {
  // std::cout << "Expecting " << nRX << " bytes " << std::endl;
  int16_t bytes_read = read(port_file_descriptor_, rx_buffer, nRX);
  // std::cout << "Received " << bytes_read << " bytes " << std::endl;
  return bytes_read;
}

}  // namespace keller
