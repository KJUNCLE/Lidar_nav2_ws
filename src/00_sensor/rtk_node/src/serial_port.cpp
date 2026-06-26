// Copyright 2026 KJUNCLE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rtk_node/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>

namespace rtk_node
{
namespace
{

speed_t baud_to_constant(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return 0;
  }
}

void set_error(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & port, int baud_rate, std::string * error)
{
  close();

  const speed_t baud = baud_to_constant(baud_rate);
  if (baud == 0) {
    set_error(error, "unsupported baud rate " + std::to_string(baud_rate));
    return false;
  }

  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    set_error(error, std::strerror(errno));
    return false;
  }

  termios options{};
  if (tcgetattr(fd, &options) != 0) {
    set_error(error, std::strerror(errno));
    ::close(fd);
    return false;
  }

  cfmakeraw(&options);
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    set_error(error, std::strerror(errno));
    ::close(fd);
    return false;
  }

  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  port_ = port;
  set_error(error, "");
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

ssize_t SerialPort::read(std::uint8_t * data, std::size_t size, std::string * error)
{
  if (fd_ < 0) {
    set_error(error, "serial port is not open");
    return -1;
  }

  const ssize_t bytes_read = ::read(fd_, data, size);
  if (bytes_read < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      set_error(error, "");
      return 0;
    }
    set_error(error, std::strerror(errno));
    return -1;
  }

  set_error(error, "");
  return bytes_read;
}

const std::string & SerialPort::port() const
{
  return port_;
}

}  // namespace rtk_node
