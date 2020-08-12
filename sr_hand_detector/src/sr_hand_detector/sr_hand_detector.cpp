/*
* Copyright 2020 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <string>
#include <utility>
#include <algorithm>
#include <ifaddrs.h>
#include <unistd.h>
#include "sr_hand_detector/sr_hand_detector.h"
#include <sys/capability.h>


namespace sr_hand_detector
{
SrHandDetector::SrHandDetector()
{
  std::cout << "Starting hand detector..." << std::endl;
}

SrHandDetector::~SrHandDetector()
{
}

void SrHandDetector::run()
{
  get_available_port_names();
  get_hands_ports_and_serials();
}

void SrHandDetector::get_available_port_names()
{
  struct ifaddrs *ifaddr, *ifa;
  int n;

  if (-1 == getifaddrs(&ifaddr))
  {
    perror("getifaddrs");
    exit(EXIT_FAILURE);
  }

  for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++)
  {
    if (ifa->ifa_addr == NULL || count(available_port_names_.begin(),
                                            available_port_names_.end(),
                                            ifa->ifa_name))
    {
      continue;
    }
    available_port_names_.push_back(ifa->ifa_name);
  }

  freeifaddrs(ifaddr);
}

void SrHandDetector::get_hands_ports_and_serials()
{
  for (auto const& port_name : available_port_names_)
  {
    if (NUM_OF_SLAVES_EXPECTED_FOR_HAND_ == count_slaves_on_port(port_name))
    {
      int hand_serial = get_hand_serial(port_name);
      hand_serial_and_port_map_.insert(std::pair<int, std::string>(hand_serial, port_name));
    }
  }
}

int SrHandDetector::count_slaves_on_port(std::string port_name)
{
  int  w;
  char *port_name_c_str = &port_name[0];
  if (ec_init(port_name_c_str))
  {
    return ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);
  }
  else
  {
    return 0;
  }
}

int SrHandDetector::get_hand_serial(std::string port_name)
{
  uint16 *wbuf;
  int hand_serial;
  char *port_name_c_str = &port_name[0];
  if (ec_init(port_name_c_str))
  {
    read_eeprom(SLAVE_WITH_HAND_SERIAL_, 0x0000, 128);

    wbuf = reinterpret_cast<uint16 *>(&ebuf_[0]);
    hand_serial = *reinterpret_cast<uint32 *>(wbuf + 0x0E);

    ec_close();
    return hand_serial;
  }
  else
  {
    std::cout << "No socket connection on " << port_name_c_str << ". Excecute as root.";
    return 0;
  }
}

int SrHandDetector::read_eeprom(int slave, int start, int length)
{
  int i, wkc, ainc = 4;
  uint16 estat, aiadr;
  uint32 b4;
  uint64 b8;
  uint8 eepctl;

  aiadr = 1 - slave;
  eepctl = 2;
  // force Eeprom from PDI
  wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET);
  eepctl = 0;
  // set Eeprom to master
  wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET);

  estat = 0x0000;
  aiadr = 1 - slave;
  // read eeprom status
  wkc = ec_APRD(aiadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET);
  estat = etohs(estat);
  if (estat & EC_ESTAT_R64)
  {
    ainc = 8;
    for (i = start ; i < (start + length) ; i+=ainc)
    {
    b8 = ec_readeepromAP(aiadr, i >> 1 , EC_TIMEOUTEEP);
    ebuf_[i] = b8;
    ebuf_[i+1] = b8 >> 8;
    ebuf_[i+2] = b8 >> 16;
    ebuf_[i+3] = b8 >> 24;
    ebuf_[i+4] = b8 >> 32;
    ebuf_[i+5] = b8 >> 40;
    ebuf_[i+6] = b8 >> 48;
    ebuf_[i+7] = b8 >> 56;
    }
  }
  else
  {
    for (i = start ; i < (start + length) ; i+=ainc)
    {
    b4 = ec_readeepromAP(aiadr, i >> 1 , EC_TIMEOUTEEP);
    ebuf_[i] = b4;
    ebuf_[i+1] = b4 >> 8;
    ebuf_[i+2] = b4 >> 16;
    ebuf_[i+3] = b4 >> 24;
    }
  }

  return 1;
}
}  // namespace sr_hand_detector
