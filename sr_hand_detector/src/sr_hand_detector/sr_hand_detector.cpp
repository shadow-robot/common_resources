#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <time.h>
#include <iostream>
#include <algorithm>
#include "sr_hand_detector/sr_hand_detector.h"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <linux/if_link.h>

namespace sr_hand_detector
{
SrHandDetector::SrHandDetector()
{
  std::cout << "Starting hand detector..." << std::endl;
}

SrHandDetector::~SrHandDetector()
{
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
        if (ifa->ifa_addr == NULL || std::count(available_port_names_.begin(), available_port_names_.end(), ifa->ifa_name))
        {
            continue;
        }
        available_port_names_.push_back(ifa->ifa_name);
    }

    freeifaddrs(ifaddr);
}

int SrHandDetector::count_slaves_on_port(std::string port_name)
{
    int  w;
    char *port_name_c_str = &port_name[0];
    if (ec_init(port_name_c_str))
    {
       return ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);      /* detect number of slaves */
    }
    else
    {
        return 0;
    }   
}

void SrHandDetector::get_hands_ports_and_serials()
{
    for(auto const& available_port_name: available_port_names_)
    {
      if (number_of_slaves_expected_for_hand_ == count_slaves_on_port(available_port_name))
      {
        int hand_serial = get_hand_serial(available_port_name);
        hand_port_and_serial_map_.insert(std::pair<std::string, int>(available_port_name, hand_serial));
      }
    }
}

int SrHandDetector::get_hand_serial(std::string port_name)
{
   int rc = 0;
   uint16 *wbuf;
   int hand_serial;
   char *port_name_c_str = &port_name[0];
   if (ec_init(port_name_c_str))
   {   
        read_eeprom(slave_with_hand_serial_, 0x0000, 128); // read first 128 bytes

        wbuf = (uint16 *)&ebuf[0];
        hand_serial = *(uint32 *)(wbuf + 0x0E);

      /* stop SOEM, close socket */
      ec_close();
      return hand_serial;
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", port_name_c_str);
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
    wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
    eepctl = 0;
    wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */

    estat = 0x0000;
    aiadr = 1 - slave;
    wkc=ec_APRD(aiadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET); /* read eeprom status */
    estat = etohs(estat);
    if (estat & EC_ESTAT_R64)
    {
        ainc = 8;
        for (i = start ; i < (start + length) ; i+=ainc)
        {
        b8 = ec_readeepromAP(aiadr, i >> 1 , EC_TIMEOUTEEP);
        ebuf[i] = b8;
        ebuf[i+1] = b8 >> 8;
        ebuf[i+2] = b8 >> 16;
        ebuf[i+3] = b8 >> 24;
        ebuf[i+4] = b8 >> 32;
        ebuf[i+5] = b8 >> 40;
        ebuf[i+6] = b8 >> 48;
        ebuf[i+7] = b8 >> 56;
        }
    }
    else
    {
        for (i = start ; i < (start + length) ; i+=ainc)
        {
        b4 = ec_readeepromAP(aiadr, i >> 1 , EC_TIMEOUTEEP);
        ebuf[i] = b4;
        ebuf[i+1] = b4 >> 8;
        ebuf[i+2] = b4 >> 16;
        ebuf[i+3] = b4 >> 24;
        }
    }
    
    return 1;
}

} // namespace sr_hand_detector
