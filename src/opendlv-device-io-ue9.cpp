/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <vector>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "ue9.hpp"
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("ip")) {
    std::cerr << argv[0] << " interfaces to a LabJack UE9  I/O board using TCP." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --ip=<IP address for the LabJack UE9> --porta=<The port A for the LabJack UE9, default 52360> --portb=<The port B for the LabJack UE9, default 52361> --cid=<OpenDaVINCI session> [--id=<ID if more than one sensor>] [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --ip=192.168.0.100 --cid=111" << std::endl;
    retCode = 1;
  } else {
    //uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    
    uint32_t const PORTA{(commandlineArguments["porta"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["porta"])) : 52360};
    uint32_t const PORTB{(commandlineArguments["portb"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["portb"])) : 52361};

    std::string const IP = commandlineArguments["ip"];

    uint32_t channelCount = 8; // 1, 2, 4, 8, 16
    uint32_t resolution = 12;
    uint32_t scanInterval = 4000;
    uint32_t readMultiples = 10;

    Ue9 ue9(IP, PORTA, PORTB);
    ue9.StartStream(channelCount, resolution, scanInterval);

    cluon::OD4Session od4{CID};
    while (od4.isRunning()) {
      std::vector<std::vector<double>> data = ue9.ParseStream(readMultiples);
      std::vector<double> sums(channelCount);
      for (uint32_t i = 0; i < data.size(); i++) {
        std::vector<double> samples = data[i];

        for (uint32_t j = 0; j < samples.size(); j++) {
          double value = samples[j];
          sums[j] += value;
        }
      }

      for (uint32_t i = 0; i < channelCount; i++) {
        double mean = sums[i] / data.size();

        opendlv::proxy::VoltageReading voltageReading;
        voltageReading.voltage(static_cast<float>(mean));
          
        od4.send(voltageReading, cluon::time::now(), i);

        if (VERBOSE) {
          std::cout << i << ":" << mean << "\t";
        }
      }
      if (VERBOSE) {
        std::cout << std::endl;
      }
    }

    ue9.StopStream();
  }
  return retCode;
}
