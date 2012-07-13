/*
* Copyright (c) 2008, AMT – The Association For Manufacturing Technology (“AMT”)
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the AMT nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* DISCLAIMER OF WARRANTY. ALL MTCONNECT MATERIALS AND SPECIFICATIONS PROVIDED
* BY AMT, MTCONNECT OR ANY PARTICIPANT TO YOU OR ANY PARTY ARE PROVIDED "AS IS"
* AND WITHOUT ANY WARRANTY OF ANY KIND. AMT, MTCONNECT, AND EACH OF THEIR
* RESPECTIVE MEMBERS, OFFICERS, DIRECTORS, AFFILIATES, SPONSORS, AND AGENTS
* (COLLECTIVELY, THE "AMT PARTIES") AND PARTICIPANTS MAKE NO REPRESENTATION OR
* WARRANTY OF ANY KIND WHATSOEVER RELATING TO THESE MATERIALS, INCLUDING, WITHOUT
* LIMITATION, ANY EXPRESS OR IMPLIED WARRANTY OF NONINFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. 

* LIMITATION OF LIABILITY. IN NO EVENT SHALL AMT, MTCONNECT, ANY OTHER AMT
* PARTY, OR ANY PARTICIPANT BE LIABLE FOR THE COST OF PROCURING SUBSTITUTE GOODS
* OR SERVICES, LOST PROFITS, LOSS OF USE, LOSS OF DATA OR ANY INCIDENTAL,
* CONSEQUENTIAL, INDIRECT, SPECIAL OR PUNITIVE DAMAGES OR OTHER DIRECT DAMAGES,
* WHETHER UNDER CONTRACT, TORT, WARRANTY OR OTHERWISE, ARISING IN ANY WAY OUT OF
* THIS AGREEMENT, USE OR INABILITY TO USE MTCONNECT MATERIALS, WHETHER OR NOT
* SUCH PARTY HAD ADVANCE NOTICE OF THE POSSIBILITY OF SUCH DAMAGES.
*/

#ifndef LABJACKU3_ADAPTER_HPP
#define LABJACKU3_ADAPTER_HPP

#include "u3.hh"
#include "adapter.hpp"
#include "device_datum.hpp"
#include "time_series.hpp"
#include "service.hpp"
#include "condition.hpp"
#include <sys/time.h>


class LabJackU3Adapter : public Adapter, public MTConnectService
{
protected:
  /* Define all the data values here */
  
  /* Events */
  Availability mAvail;
  EmergencyStop mEstop;
  PowerState mPower;
  Message mMessage;
  
  /* Samples */
  HANDLE hDevice;
  u3CalibrationInfo caliInfo;
  int dac1Enabled;
 
  double mStartTime;


bool mConnected;
protected:
  void disconnect();

public:
  TimeSeries manalog01; 
  TimeSeries manalog02;
  TimeSeries manalog03; 
  TimeSeries manalog04; 
  TimeSeries manalog05; 
  TimeSeries manalog06; 
  LabJackU3Adapter(int aPort);
  ~LabJackU3Adapter();

  void initialize(int aArgc, const char *aArgv[]);
  void start();
  void stop();
  void gatherDeviceData();


int ConfigIO_example(HANDLE hDevice, int *isDAC1Enabled);
int StreamConfig_example(HANDLE hDevice);
int StreamStart(HANDLE hDevice);
int StreamData_example(HANDLE hDevice, u3CalibrationInfo *caliInfo, int isDAC1Enabled);
int StreamStop(HANDLE hDevice);



  int recordCallback(const void *inputBuffer, void *outputBuffer,
		     unsigned long framesPerBuffer,
		     const time_t timeInfo);
  
};

#endif

