/** \file
 * \brief EEprom tool for Simple Open EtherCAT master
 *
 * Usage : eepromtool ifname slave OPTION fname|alias
 * ifname is NIC interface, f.e. eth0
 * slave = slave number in EtherCAT order 1..n
 * -r      read EEPROM, output binary format
 * -ri     read EEPROM, output Intel Hex format
 * -w      write EEPROM, input binary format
 * -wi     write EEPROM, input Intel Hex format
 * -i      display EEPROM information
 * -walias write slave alias in EEPROM
 *
 * (c)Arthur Ketels 2010-2012 
 */

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

void SrHandDetector::get_port_names()
{
    struct ifaddrs *ifaddr, *ifa;
    int n;

    for (int i = 0; i<MAX_PORTS; i++)
    {
        available_port_names_[i] = (char*)malloc(10);
        available_port_names_[i][0] = 0;
        hand_port_names_[i] = (char*)malloc(10);
        hand_port_names_[i][0] = 0;
    }

    if (getifaddrs(&ifaddr) == -1)
    {
        perror("getifaddrs");
        exit(EXIT_FAILURE);
    }

    for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++)
    {
        if (ifa->ifa_addr == NULL || std::count(available_ports_names_.begin(), available_ports_names_.end(), ifa->ifa_name))
        {
            continue;
        }
        available_ports_names_.push_back(ifa->ifa_name);
        add_port_name(ifa->ifa_name);
    }

    freeifaddrs(ifaddr);
}

void SrHandDetector::add_port_name(char* port_name)
{
    for (int i=0; i<num_ports_; i++)
    {
        if (strncmp(available_port_names_[i], port_name, strlen(port_name)) == 0)
        {
            return;
        }
    }
    strcpy(available_port_names_[num_ports_], port_name);
    num_ports_++;
}

int SrHandDetector::count_slaves(int port)
{
    int  w;

    if (port >= num_ports_)
	{
        return 0;
	}

    std::cout << "Checking port: " <<  available_ports_names_[port] << std::endl;

    char *available_port_name_c_str = &available_ports_names_[port][0];
    if (ec_init(available_port_name_c_str))
    {
       return ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);      /* detect number of slaves */
    }
    else
    {
        return 0;
    }   
}

void SrHandDetector::detect_hand_ports()
{
    for (int i=0; i<num_ports_; i++)
    {
      if (2 == count_slaves(i))
      {
          char *available_port_name_c_str = &available_ports_names_[i][0];
        strcpy(hand_port_names_[num_hands_], available_port_name_c_str);
        num_hands_++;
      }
    }
}

int SrHandDetector::get_hand_serial(char* port_name)
{
   int rc = 0, slave = 2;
   uint16 *wbuf;
   if (ec_init(port_name))
   {   
        read_eeprom(slave, 0x0000, 128); // read first 128 bytes

        wbuf = (uint16 *)&ebuf[0];
        printf(" Serial Number    : %8.8X (decimal = %u)\n",*(uint32 *)(wbuf + 0x0E),*(uint32 *)(wbuf + 0x0E));

      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n", port_name);
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

// #define MAXBUF 32768
// #define STDBUF 2048
// #define MINBUF 128
// #define CRCBUF 14

// #define MODE_NONE               0
// #define MODE_READBIN            1
// #define MODE_READINTEL          2
// #define MODE_WRITEBIN           3
// #define MODE_WRITEINTEL         4
// #define MODE_WRITEALIAS         5
// #define MODE_INFO               6
// #define MODE_WRITE_SERIAL_ONLY  7

// #define MAXSLENGTH        256

// uint8 ebuf[MAXBUF];
// uint8 ob;
// uint16 ow;
// int os;
// int slave;
// int alias;
// struct timeval tstart,tend, tdif;
// int wkc;
// int mode;
// char sline[MAXSLENGTH];
// uint32 serial_number = 0x00000000;

// #define MAX_PORTS 8
// char* port_names[MAX_PORTS];
// int num_ports = 0;

// #define IHEXLENGTH 0x20


// void add_port_name(char* port_name)
// {
// 	int i;

// 	for (i=0; i<num_ports; i++)
// 	{
// 		if (strncmp(port_names[i], port_name, strlen(port_name)) == 0)
// 		{
// 			return;
// 		}
// 	}

// 	strcpy(port_names[num_ports], port_name);
// 	num_ports++;
// }

// void get_ifaddrs()
// {
//     struct ifaddrs *ifaddr, *ifa;
//     int family, s, n;
//     char host[NI_MAXHOST];
// 	int i;

// 	for (i=0; i<MAX_PORTS; i++)				// allocate memory to hold the port names
// 	{
// 		port_names[i] = (char*)malloc(10);
// 		port_names[i][0] = 0;
// 	}

//     if (getifaddrs(&ifaddr) == -1)
//     {
//         perror("getifaddrs");
//         exit(EXIT_FAILURE);
//     }

//     // Walk through linked list, maintaining head pointer so we
//     // can free list later

//     for (ifa = ifaddr, n = 0; ifa != NULL; ifa = ifa->ifa_next, n++)
//     {
//         if (ifa->ifa_addr == NULL)
//         {
//             continue;
//         }

//         family = ifa->ifa_addr->sa_family;
//         add_port_name(ifa->ifa_name);
//     }

//     freeifaddrs(ifaddr);
// }


// void calc_crc(uint8 *crc, uint8 b)
// {
//    int j;
//    *crc ^= b;
//    for(j = 0; j <= 7 ; j++ )
//    {
//      if(*crc & 0x80)
//         *crc = (*crc << 1) ^ 0x07;
//      else
//         *crc = (*crc << 1);
//    }  
// }

// uint16 SIIcrc(uint8 *buf)
// {
//    int i; 
//    uint8 crc;
    
//    crc = 0xff; 
//    for( i = 0 ; i <= 13 ; i++ )
//    {
//       calc_crc(&crc , *(buf++));  
//    } 
//    return (uint16)crc;
// }

// int input_bin(char *fname, int *length)
// {
//    FILE *fp;
 
//    int cc = 0, c;

//    fp = fopen(fname, "rb");
//    if(fp == NULL) 
//       return 0;
//    while (((c = fgetc(fp)) != EOF) && (cc < MAXBUF))
//       ebuf[cc++] = (uint8)c;
//    *length = cc;
//    fclose(fp);
   
//    return 1;
// }

// int input_intelhex(char *fname, int *start, int *length)
// {
//    FILE *fp;
 
//    int c, sc, retval = 1;
//    int ll, ladr, lt, sn, i, lval;
//    int hstart, hlength, sum;

//    fp = fopen(fname, "r");
//    if(fp == NULL) 
//       return 0;
//    hstart = MAXBUF;
//    hlength = 0;
//    sum = 0;
//    do
//    {
//       memset(sline, 0x00, MAXSLENGTH);
//       sc = 0;
//       while (((c = fgetc(fp)) != EOF) && (c != 0x0A) && (sc < (MAXSLENGTH -1)))
//          sline[sc++] = (uint8)c;
//       if ((c != EOF) && ((sc < 11) || (sline[0] != ':')))
//       {
//          c = EOF;
//          retval = 0;
//          printf("Invalid Intel Hex format.\n");
//       }
//       if (c != EOF)
//       {
//          sn = sscanf(sline , ":%2x%4x%2x", &ll, &ladr, &lt);
//          if ((sn == 3) && ((ladr + ll) <= MAXBUF) && (lt == 0))
//          {
//             sum = ll + (ladr >> 8) + (ladr & 0xff) + lt;
//             if(ladr < hstart) hstart = ladr;
//             for(i = 0; i < ll ; i++)
//             {
//                sn = sscanf(&sline[9 + (i << 1)], "%2x", &lval);
//                ebuf[ladr + i] = (uint8)lval;
//                sum += (uint8)lval;
//             }
//             if(((ladr + ll) - hstart) > hlength)
//                hlength = (ladr + ll) - hstart;
//             sum = (0x100 - sum) & 0xff;
//             sn = sscanf(&sline[9 + (i << 1)], "%2x", &lval);
//             if (!sn || ((sum - lval) != 0))
//             {
//                c = EOF;
//                retval = 0;
//                printf("Invalid checksum.\n");
//             }
//          }
//       }      
//    }
//    while (c != EOF);
//    if (retval)
//    {
//       *length = hlength;
//       *start = hstart;
//    }
//    fclose(fp);
   
//    return retval;
// }

// int output_bin(char *fname, int length)
// {
//    FILE *fp;
 
//    int cc;

//    fp = fopen(fname, "wb");

//    if(fp == NULL) 
//       return 0;

//    for (cc = 0 ; cc < length ; cc++)
//       fputc( ebuf[cc], fp);

//    fclose(fp);
   
//    return 1;
// }

// int output_intelhex(char *fname, int length)
// {
//    FILE *fp;
 
//    int cc = 0, ll, sum, i;

//    fp = fopen(fname, "w");
//    if(fp == NULL) 
//       return 0;
//    while (cc < length)
//    {
//       ll = length - cc;
//       if (ll > IHEXLENGTH) ll = IHEXLENGTH;
//       sum = ll + (cc >> 8) + (cc & 0xff);
//       fprintf(fp, ":%2.2X%4.4X00", ll, cc);
//       for (i = 0; i < ll; i++)
//       {
//          fprintf(fp, "%2.2X", ebuf[cc + i]);
//          sum += ebuf[cc + i];
//       }
//       fprintf(fp, "%2.2X\n", (0x100 - sum) & 0xff);
//       cc += ll;
//    }
//    fprintf(fp, ":00000001FF\n");   
//    fclose(fp);
   
//    return 1;
// }

// int eeprom_read(int slave, int start, int length)
// {
//    int i, wkc, ainc = 4;
//    uint16 estat, aiadr;
//    uint32 b4;
//    uint64 b8;
//    uint8 eepctl;
   
//    if((ec_slavecount >= slave) && (slave > 0) && ((start + length) <= MAXBUF))
//    {
//       aiadr = 1 - slave;
//       eepctl = 2;
//       wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
//       eepctl = 0;
//       wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */

//       estat = 0x0000;
//       aiadr = 1 - slave;
//       wkc=ec_APRD(aiadr, ECT_REG_EEPSTAT, sizeof(estat), &estat, EC_TIMEOUTRET); /* read eeprom status */
//       estat = etohs(estat);
//       if (estat & EC_ESTAT_R64)
//       {
//          ainc = 8;
//          for (i = start ; i < (start + length) ; i+=ainc)
//          {
//             b8 = ec_readeepromAP(aiadr, i >> 1 , EC_TIMEOUTEEP);
//             ebuf[i] = b8;
//             ebuf[i+1] = b8 >> 8;
//             ebuf[i+2] = b8 >> 16;
//             ebuf[i+3] = b8 >> 24;
//             ebuf[i+4] = b8 >> 32;
//             ebuf[i+5] = b8 >> 40;
//             ebuf[i+6] = b8 >> 48;
//             ebuf[i+7] = b8 >> 56;
//          }
//       }
//       else
//       {
//          for (i = start ; i < (start + length) ; i+=ainc)
//          {
//             b4 = ec_readeepromAP(aiadr, i >> 1 , EC_TIMEOUTEEP);
//             ebuf[i] = b4;
//             ebuf[i+1] = b4 >> 8;
//             ebuf[i+2] = b4 >> 16;
//             ebuf[i+3] = b4 >> 24;
//          }
//       }
      
//       return 1;
//    }
   
//    return 0;
// }

// #define NUM_STRINGS 16

// char* strings[NUM_STRINGS] = {"\e[2;37mPDI Interface\e[0m",      // 00
//                               "\e[2;37mSPI Mode\e[0m",           // 01
//                               "",                   // 02
//                               "",                   // 03
//                               "",                   // 04
//                               "",                   // 05
//                               "",                   // 06
//                               "\e[1;37mCalculated CRC\e[0m",     // 07
//                               "\e[2;37mVendor ID low\e[0m",      // 08
//                               "\e[2;37mVendor ID high\e[0m",     // 09
//                               "\e[2;37mProduct Code low\e[0m",   // 0A
//                               "\e[2;37mProduct Code high\e[0m",  // 0B
//                               "\e[2;37mRevision low\e[0m",   // 0C
//                               "\e[2;37mRevision high\e[0m",  // 0D
//                               "\e[1;37mSerial Number low\e[0m",  // 0E
//                               "\e[1;37mSerial Number high\e[0m"  // 0F
//                      };


// int eeprom_write(int slave, int start, int length)
// {
//    int i, wkc, dc = 0;
//    uint16 aiadr, *wbuf;
//    uint8 eepctl;
//    int ret;
//    int reps_countdown;
   
//    //printf("eeprom_write(%d, %d, %d)\n", slave, start, length);

//    if((ec_slavecount >= slave) && (slave > 0) && ((start + length) <= MAXBUF))
//    {
//       aiadr = 1 - slave;
//       eepctl = 2;
//       wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
//       eepctl = 0;
//       wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */

//       aiadr = 1 - slave;
//       wbuf = (uint16 *)&ebuf[0];

//       wbuf[0x000E/2] = SIIcrc(&ebuf[0]);

//       for (i = start ; i < (start + length) ; i+=2)
//       {
//          printf("\nWriting[0x%04x] = %04x", i, *(wbuf + (i >> 1)));

//          //if (i == 0x000E)
// 	 //   printf(" (calculated CRC 0x%04x) ", SIIcrc(&ebuf[0]));

//          reps_countdown = 10;

//          while(reps_countdown--)
//          {
// 	    ret = ec_writeeepromAP(aiadr, i >> 1 , *(wbuf + (i >> 1)), EC_TIMEOUTEEP);
// 	    if (ret)
//                break;
//             usleep(1000);
//          }

//          if (reps_countdown == -1)
//             printf(" - \e[1;31mfailed\e[0m ");
//          else
//             printf(" - \e[1;32mOK\e[0m     ");

// 	 if (i>>1 < NUM_STRINGS)
//          	printf(strings[i>>1]);
//       }
//       return 1;
//    }
   
//    return 0;
// }

// int eeprom_writealias(int slave, int alias, uint16 crc)
// {
//    int wkc;
//    uint16 aiadr;
//    uint8 eepctl;
//    int ret;
   
//    if((ec_slavecount >= slave) && (slave > 0) && (alias <= 0xffff))
//    {
//       aiadr = 1 - slave;
//       eepctl = 2;
//       wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* force Eeprom from PDI */
//       eepctl = 0;
//       wkc = ec_APWR(aiadr, ECT_REG_EEPCFG, sizeof(eepctl), &eepctl , EC_TIMEOUTRET); /* set Eeprom to master */

//       ret = ec_writeeepromAP(aiadr, 0x04 , alias, EC_TIMEOUTEEP);
//       if (ret)
//         ret = ec_writeeepromAP(aiadr, 0x07 , crc, EC_TIMEOUTEEP);
        
//       return ret;
//    }
   
//    return 0;
// }

// void eepromtool(char *ifname, int slave, int mode, char *fname)
// {
//    int w, rc = 0, estart, esize;
//    uint16 *wbuf;
   
//    /* initialise SOEM, bind socket to ifname */
//    if (ec_init(ifname))
//    {   
//       printf("ec_init on %s succeeded.\n",ifname);

//       w = 0x0000;
//        wkc = ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);      /* detect number of slaves */
//        if (wkc > 0)
//        {
//          ec_slavecount = wkc;

//          printf("%d slaves found.\n",ec_slavecount);
//          if((ec_slavecount >= slave) && (slave > 0))
//          {

//             if ((mode == MODE_INFO) || (mode == MODE_READBIN) || (mode == MODE_READINTEL))
//             {
//                rc =  gettimeofday(&tstart, NULL);
//                eeprom_read(slave, 0x0000, MINBUF); // read first 128 bytes

//                wbuf = (uint16 *)&ebuf[0];
//                printf("Slave %d data\n", slave);
//                printf(" PDI Control      : %4.4X\n",*(wbuf + 0x00));
//                printf(" PDI Config       : %4.4X\n",*(wbuf + 0x01));
//                printf(" Config Alias     : %4.4X\n",*(wbuf + 0x04));
//                printf(" Checksum         : %4.4X\n",*(wbuf + 0x07));
//                printf("   calculated     : %4.4X\n",SIIcrc(&ebuf[0])); 
//                printf(" Vendor ID        : %8.8X\n",*(uint32 *)(wbuf + 0x08));
//                printf(" Product Code     : %8.8X\n",*(uint32 *)(wbuf + 0x0A));
//                printf(" Revision Number  : %8.8X\n",*(uint32 *)(wbuf + 0x0C));
//                printf(" Serial Number    : %8.8X (decimal = %u)\n",*(uint32 *)(wbuf + 0x0E),*(uint32 *)(wbuf + 0x0E));
//                printf(" Mailbox Protocol : %4.4X\n",*(wbuf + 0x1C));
//                esize = (*(wbuf + 0x3E) + 1) * 128;
//                if (esize > MAXBUF) esize = MAXBUF;
//                printf(" Size             : %4.4X = %d bytes\n",*(wbuf + 0x3E), esize);
//                printf(" Version          : %4.4X\n",*(wbuf + 0x3F));
//             }

//             if ((mode == MODE_READBIN) || (mode == MODE_READINTEL))
//             {
//                if (esize > MINBUF)
//                   eeprom_read(slave, MINBUF, esize - MINBUF); // read reminder

//                rc =  gettimeofday(&tend, NULL);
//                timersub(&tend, &tstart, &tdif);
//                if (mode == MODE_READINTEL) output_intelhex(fname, esize);
//                if (mode == MODE_READBIN)   output_bin(fname, esize);

//                printf("\nTotal EEPROM read time :%ldms\n", (tdif.tv_usec+(tdif.tv_sec*1000000L)) / 1000);
//             }

//             if ((mode == MODE_WRITEBIN) || (mode == MODE_WRITEINTEL))
//             {
//                estart = 0;
//                if (mode == MODE_WRITEINTEL) rc = input_intelhex(fname, &estart, &esize);
//                if (mode == MODE_WRITEBIN)   rc = input_bin(fname, &esize);

//                if (serial_number != 0xFFFFFFFF)
//                {
//                   ebuf[0x1F] = (serial_number >> 24) & 0x000000FF;
//                   ebuf[0x1E] = (serial_number >> 16) & 0x000000FF;
//                   ebuf[0x1D] = (serial_number >>  8) & 0x000000FF;
//                   ebuf[0x1C] = (serial_number      ) & 0x000000FF;
//                }

//                if (rc > 0)
//                {               
//                   wbuf = (uint16 *)&ebuf[0];
//                   printf("Slave %d\n", slave);
//                   printf(" Vendor ID        : %8.8X\n",*(uint32 *)(wbuf + 0x08));
//                   printf(" Product Code     : %8.8X\n",*(uint32 *)(wbuf + 0x0A));
//                   printf(" Revision Number  : %8.8X\n",*(uint32 *)(wbuf + 0x0C));
//                   printf(" Serial Number    : %8.8X\n",*(uint32 *)(wbuf + 0x0E));

//                   printf("Busy");
//                   fflush(stdout);
//                   rc =  gettimeofday(&tstart, NULL);
//                   eeprom_write(slave, estart, esize);
//                   rc =  gettimeofday(&tend, NULL);               
//                   timersub(&tend, &tstart, &tdif);

//                   printf("\nTotal EEPROM write time :%ldms\n", (tdif.tv_usec+(tdif.tv_sec*1000000L)) / 1000);
//                }
//                else
//                   printf("Error reading file, abort.\n"); 
//             }

//             if (mode == MODE_WRITEALIAS)
//             {
//                if( eeprom_read(slave, 0x0000, CRCBUF) ) // read first 14 bytes
//                {
//                   wbuf = (uint16 *)&ebuf[0];
//                   *(wbuf + 0x04) = alias;  
//                   if(eeprom_writealias(slave, alias, SIIcrc(&ebuf[0])))
//                   {
//                      printf("Alias %4.4X written successfully to slave %d\n", alias, slave);
//                   } 
//                   else
//                   {
//                      printf("Alias not written\n");
//                   }
//                }  
//                else
//                {
//                   printf("Could not read slave EEPROM");    
//                }
//             }

//             if (mode == MODE_WRITE_SERIAL_ONLY)
//             {
//                uint32 current_serial_number;
//                int i;

//                eeprom_read(slave, 0x001c, 4);

//                current_serial_number = (((uint32)ebuf[0x1F]) << 24) |
//                                        (((uint32)ebuf[0x1E]) << 16) |
//                                        (((uint32)ebuf[0x1D]) <<  8) |
//                                        (((uint32)ebuf[0x1C])      );

//                printf("Current Serial number = 0x%08x (decimal = %u)\n", current_serial_number, current_serial_number);

//                ebuf[0x1F] = (serial_number >> 24) & 0x000000FF;
//                ebuf[0x1E] = (serial_number >> 16) & 0x000000FF;
//                ebuf[0x1D] = (serial_number >>  8) & 0x000000FF;
//                ebuf[0x1C] = (serial_number      ) & 0x000000FF;

//                printf("Writing Serial number = 0x%08x (decimal = %u)\n", serial_number, serial_number);

//                eeprom_write(slave, 0x001c, 4);

//                for (i=0; i<10; i++)
//                {
//                        usleep(100);
// 		       eeprom_read(slave, 0x001c, 4);

// 		       current_serial_number = (((uint32)ebuf[0x1F]) << 24) |
// 		                               (((uint32)ebuf[0x1E]) << 16) |
// 		                               (((uint32)ebuf[0x1D]) <<  8) |
// 		                               (((uint32)ebuf[0x1C])      );

//                   if (current_serial_number == serial_number)
//                      break;
//                }

//                printf("\n");
//                if (current_serial_number == serial_number)
//                   printf("Verifying \e[1;32mOK\e[0m\n");
//                else
//                   printf("Verifying \e[1;31mfailed\e[0m\n");
//             }
//          }
//          else
//          {
//             printf("Slave number outside range.\n");
//          }
//       }
//       else
//       {
//          printf("No slaves found!\n");
//       }
//       printf("End, close socket\n");
//       /* stop SOEM, close socket */
//       ec_close();
//    }
//    else
//    {
//       printf("No socket connection on %s\nExcecute as root\n",ifname);
//    }
// }   


// // Count the number of slaves on this port
// int count_slaves(int port)
// {
//     int  w;

//     if (port >= num_ports)
// 	{
//         return 0;
// 	}

 
//     printf("Checking port %s\n", port_names[port]);

//     if (ec_init(port_names[port]))
//     {
//        return ec_BRD(0x0000, ECT_REG_TYPE, sizeof(w), &w, EC_TIMEOUTSAFE);      /* detect number of slaves */
//     }
//     else
//     {
//         return 0;
//     }   

// }


// // count the number of slaves on attached ports
// // print which ports have slaves
// void find_print_slaves()
// {
//     int port;
//     int numSlaves;
//     int success = 0;

// 	get_ifaddrs();		// Fetch a list of the port names, so we can search them for EtherCAT slaves

//     for (port=0; port<num_ports; port++)
//     {
//         numSlaves = count_slaves(port);

//         if (numSlaves > 0)
//         {
//             printf("\e[1;32m%s: \e[1;37m %d slaves\e[0m\n", port_names[port], numSlaves);
//             success = 1;
//         }
//     }

//     if (!success)
//     {
//         printf("\e[1;31mNo slaves are connected to this PC.\e[0m\n");
//     }
// }


// int create_bin_file(int product_id)
// {
//     uint16 *wbuf = (uint16 *)&ebuf[0];
    
//     wbuf[0x00] = 0x0107;
//     wbuf[0x01] = 0x0000;
//     wbuf[0x02] = 0x0000;
//     wbuf[0x03] = 0x0000;
//     wbuf[0x04] = 0xC001;
//     wbuf[0x05] = 0x0000;
//     wbuf[0x06] = 0x0000;
//     wbuf[0x07] = 0x0067;
//     wbuf[0x08] = 0x0530;				// Vendor ID
//     wbuf[0x09] = 0x0000;				// Vendor ID
//     wbuf[0x0A] = product_id & 0xFFFF;
//     wbuf[0x0B] = product_id >> 16;
//     wbuf[0x0C] = 0x0000;				// Revision
//     wbuf[0x0D] = 0x0000;				// Revision
//     wbuf[0x0E] = 0x0000;				// Serial No
//     wbuf[0x0F] = 0x0000;				// Serial No

// 	return 32;	// 32 bytes
// }



// int main(int argc, char *argv[])
// {
// 	printf("\n");

//     mode = MODE_NONE;
//     if (argc > 3)
//     {      
//         slave = atoi(argv[2]);
//         // if ((strncmp(argv[3], "-i", sizeof("-i")) == 0))   mode = MODE_INFO;
//         // if (argc > 4)
//         // {
//         //     if ((strncmp(argv[3], "-r",      sizeof("-r"))      == 0))  mode = MODE_READBIN;
//         //     if ((strncmp(argv[3], "-ri",     sizeof("-ri"))     == 0))  mode = MODE_READINTEL;

//         //     if ((strncmp(argv[3], "-w",      sizeof("-w"))      == 0))
//         //     {
//         //         mode = MODE_WRITEBIN;

//         //         if (argc > 5)                                      // -w also accepts a serial number to be inserted into the EEPROM
//         //             serial_number = strtol(argv[5], (char*)0, 0);
//         //         else
//         //            serial_number = 0xFFFFFFFF;
//         //     }

//         //     if ((strncmp(argv[3], "-wi",     sizeof("-wi"))     == 0))  mode = MODE_WRITEINTEL;
//         //     if ((strncmp(argv[3], "-walias", sizeof("-walias")) == 0))
//         //     {
//         //         mode = MODE_WRITEALIAS;
//         //         alias = atoi(argv[4]);
//         //     }
//         //     if ((strncmp(argv[3], "-sn", sizeof("-sn")) == 0))
//         //     {   
//         //         if ((strncmp(argv[4], "0x", strlen("0x")) == 0) || (strncmp(argv[4], "0X", strlen("0X")) == 0))
//         //         {
//         //             serial_number = strtol(argv[4], (char*)0, 0);
//         //         }
//         //         else
//         //         {
//         //             serial_number = strtol(argv[4], (char*)0, 10);
//         //         }
//         //         mode = MODE_WRITE_SERIAL_ONLY;
//         //     }

//         //     if ((strncmp(argv[3], "-c", sizeof("-c")) == 0))
// 		// 	{
// 		// 		int product_id = strtol(argv[4], (char*)0, 0);
// 		// 		int num_bytes = create_bin_file(product_id);
//     	// 		output_bin(argv[5], num_bytes);	// 24 bytes
// 		// 	}

//         //     if ((strncmp(argv[3], "-ci", sizeof("-ci")) == 0))
// 		// 	{
// 		// 		int product_id = strtol(argv[4], (char*)0, 0);
// 		// 		int num_bytes = create_bin_file(product_id);
//     	// 		output_intelhex(argv[5], num_bytes);	// 24 bytes
// 		// 	}
//         // }
//         // /* start tool */
//         // eepromtool(argv[1],slave,mode,argv[4]);
//     }
//     else
//     {
//         printf("Usage: eepromtool ifname slave OPTION fname|alias [serial]\n");
//         printf("ifname = eth0 for example\n");
//         printf("slave = slave number in EtherCAT order 1..n\n");
//         printf("    -i      display EEPROM information\n");
//         printf("    -walias write slave alias\n");
//         printf("    -r      read EEPROM, output binary format\n");
//         printf("    -ri     read EEPROM, output Intel Hex format\n");
//         printf("    -w      write EEPROM, input binary format. Accepts serial number as additional parameter\n");
//         printf("    -wi     write EEPROM, input Intel Hex format\n");
//         printf("    -sn     write Serial Number only. Number format in decimal or Hex (0xNNNNNNNN)\n");
//         printf("    -c      Create .bin file. <product_id> <filename>   (e.g. eepromtool eth0 1 -c  0x02000105 palm_H.bin)\n");
//         printf("    -ci     Create .hex file. <product_id> <filename>   (e.g. eepromtool eth0 1 -ci 0x02000105 palm_H.hex)\n");
//         printf("\n");

//         find_print_slaves();
//         printf("\n");
//     }   

//     printf("End program\n");
   
//     return (0);
// }