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

#ifndef RTK_NODE__SERIAL_PORT_HPP_
#define RTK_NODE__SERIAL_PORT_HPP_

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <string>

namespace rtk_node
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool open(const std::string & port, int baud_rate, std::string * error);
  void close();
  bool is_open() const;
  ssize_t read(std::uint8_t * data, std::size_t size, std::string * error);

  const std::string & port() const;

private:
  int fd_ = -1;
  std::string port_;
};

}  // namespace rtk_node

#endif  // RTK_NODE__SERIAL_PORT_HPP_
