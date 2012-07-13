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

#define SAMPLE_FREQUENCY 100

#include "u3.hh"
#include "internal.hpp"
#include "logger.hpp"
#include "labjack_adapter.hpp"
#include <stdlib.h>
#include  <stdio.h>


long  count;
   div_t dcount;

const uint8 NumChannels = 6;  //SamplesPerPacket needs to be a multiple of
                              //NumChannels.
const uint8 SamplesPerPacket = 25;  //Needs to be 25 to read multiple StreamData
                                    //responses in one large packet, otherwise
                                    //can be any value between 1-25 for 1
                                    //StreamData response per packet.



LabJackU3Adapter::LabJackU3Adapter(int aPort)
  : Adapter(aPort, 10), mAvail("avail"), mEstop("estop"), mPower("power"), manalog02("system_pressure"),
    manalog01("coolant_pressure"), manalog03("tailstock_pressure"),manalog04("pneumatic_pressure"),
    manalog05("linear_vibration"), manalog06("other_vibration"), mMessage("message")
{
  addDatum(mAvail);
  addDatum(mEstop);
  addDatum(mPower);
  addDatum(mMessage);  

  addDatum(manalog01);
  addDatum(manalog02);
  addDatum(manalog03);
  addDatum(manalog04);
  addDatum(manalog05);
  addDatum(manalog06); 
  mConnected = false;
 // mEstop = false;
}

void LabJackU3Adapter::initialize(int aArgc, const char *aArgv[])
{
  MTConnectService::initialize(aArgc, aArgv);
  if (aArgc > 1) {
    mPort = atoi(aArgv[1]);
  }
 hDevice = openUSBConnection(-1);
getCalibrationInfo(hDevice, &caliInfo);
ConfigIO_example(hDevice, &dac1Enabled);
StreamConfig_example(hDevice);
StreamStop(hDevice);
StreamStart(hDevice);    
StreamData_example(hDevice, &caliInfo, dac1Enabled);
gLogger->warning("It starts here");

}

////u3 functions are 

//Sends a ConfigIO low-level command that configures the FIOs, DAC, Timers and Counters
int LabJackU3Adapter::ConfigIO_example(HANDLE hDevice, int *isDAC1Enabled)
{
    uint8 sendBuff[12], recBuff[12];
    uint16 checksumTotal;
    int sendChars, recChars;

    sendBuff[1] = (uint8)(0xF8);  //Command byte
    sendBuff[2] = (uint8)(0x03);  //Number of data words
    sendBuff[3] = (uint8)(0x0B);  //Extended command number

    sendBuff[6] = 13;  //Writemask : Setting writemask for timerCounterConfig (bit 0),
                       //            FIOAnalog (bit 2) and EIOAnalog (bit 3)

    sendBuff[7] = 0;  //Reserved
    sendBuff[8] = 64;  //TimerCounterConfig: Disabling all timers and counters,
                       //                    set TimerCounterPinOffset to 4 (bits 4-7)
    sendBuff[9] = 0;  //DAC1Enable

    sendBuff[10] = 255;  //FIOAnalog : setting all FIOs as analog inputs
    sendBuff[11] = 255;  //EIOAnalog : setting all EIOs as analog inputs
    extendedChecksum(sendBuff, 12);

    //Sending command to U3
    if( (sendChars = LJUSB_Write(hDevice, sendBuff, 12)) < 12 )
    {
        if( sendChars == 0 )
            printf("ConfigIO error : write failed\n");
        else
            printf("ConfigIO error : did not write all of the buffer\n");
        return -1;
    }

    //Reading response from U3
    if( (recChars = LJUSB_Read(hDevice, recBuff, 12)) < 12 )
    {
        if( recChars == 0 )
            printf("ConfigIO error : read failed\n");
        else
            printf("ConfigIO error : did not read all of the buffer\n");
        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 12);
    if( (uint8)((checksumTotal / 256 ) & 0xFF) != recBuff[5] )
    {
        printf("ConfigIO error : read buffer has bad checksum16(MSB)\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xFF) != recBuff[4] )
    {
        printf("ConfigIO error : read buffer has bad checksum16(LBS)\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0] )
    {
        printf("ConfigIO error : read buffer has bad checksum8\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x03) || recBuff[3] != (uint8)(0x0B) )
    {
        printf("ConfigIO error : read buffer has wrong command bytes\n");
        return -1;
    }

    if( recBuff[6] != 0 )
    {
        printf("ConfigIO error : read buffer received errorcode %d\n", recBuff[6]);
        return -1;
    }

    if( recBuff[8] != 64 )
    {
        printf("ConfigIO error : TimerCounterConfig did not get set correctly\n");
        return -1;
    }

    if( recBuff[10] != 255 && recBuff[10] != (uint8)(0x0F) )
    {
        printf("ConfigIO error : FIOAnalog did not set get correctly\n");
        return -1;
    }

    if( recBuff[11] != 255 )
    {
        printf("ConfigIO error : EIOAnalog did not set get correctly (%d)\n", recBuff[11]);
        return -1;
    }


    *isDAC1Enabled = (int)recBuff[9];

    return 0;
}




//Sends a StreamConfig low-level command to configure the stream.
int LabJackU3Adapter::StreamConfig_example(HANDLE hDevice)
{
    uint8 sendBuff[64], recBuff[8];
    uint16 checksumTotal, scanInterval;
    int sendBuffSize, sendChars, recChars, i;

    sendBuffSize = 12+NumChannels*2;

    sendBuff[1] = (uint8)(0xF8);    //Command byte
    sendBuff[2] = 3 + NumChannels;  //Number of data words = NumChannels + 3
    sendBuff[3] = (uint8)(0x11);    //Extended command number
    sendBuff[6] = NumChannels;      //NumChannels
    sendBuff[7] = SamplesPerPacket; //SamplesPerPacket
    sendBuff[8] = 0;  //Reserved
    sendBuff[9] = 1;  //ScanConfig:
                      // Bit 7: Reserved
                      // Bit 6: Reserved
                      // Bit 3: Internal stream clock frequency = b0: 4 MHz
                      // Bit 2: Divide Clock by 256 = b0
                      // Bits 0-1: Resolution = b01: 11.9-bit effective

    scanInterval = 4000;
    sendBuff[10] = (uint8)(scanInterval & (0x00FF));  //Scan interval (low byte)
    sendBuff[11] = (uint8)(scanInterval / 256);  //Scan interval (high byte)

    for( i = 0; i < NumChannels; i++ )
    {
        sendBuff[12 + i*2] = i;  //PChannel = i
        sendBuff[13 + i*2] = 31;  //NChannel = 31: Single Ended
    }

    extendedChecksum(sendBuff, sendBuffSize);

    //Sending command to U3
    sendChars = LJUSB_Write(hDevice, sendBuff, sendBuffSize);
    if( sendChars < sendBuffSize )
    {
        if( sendChars == 0 )
            printf("Error : write failed (StreamConfig).\n");
        else
            printf("Error : did not write all of the buffer (StreamConfig).\n");
        return -1;
    }

    for( i = 0; i < 8; i++ )
        recBuff[i] = 0;

    //Reading response from U3
    recChars = LJUSB_Read(hDevice, recBuff, 8);
    if( recChars < 8 )
    {
        if( recChars == 0 )
            printf("Error : read failed (StreamConfig).\n");
        else
            printf("Error : did not read all of the buffer, %d (StreamConfig).\n", recChars);

        for( i = 0; i < 8; i++ )
            printf("%d ", recBuff[i]);

        return -1;
    }

    checksumTotal = extendedChecksum16(recBuff, 8);
    if( (uint8)((checksumTotal / 256) & 0xFF) != recBuff[5] )
    {
        printf("Error : read buffer has bad checksum16(MSB) (StreamConfig).\n");
        return -1;
    }

    if( (uint8)(checksumTotal & 0xFF) != recBuff[4] )
    {
        printf("Error : read buffer has bad checksum16(LBS) (StreamConfig).\n");
        return -1;
    }

    if( extendedChecksum8(recBuff) != recBuff[0] )
    {
        printf("Error : read buffer has bad checksum8 (StreamConfig).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xF8) || recBuff[2] != (uint8)(0x01) ||
        recBuff[3] != (uint8)(0x11) || recBuff[7] != (uint8)(0x00) )
    {
        printf("Error : read buffer has wrong command bytes (StreamConfig).\n");
        return -1;
    }

    if( recBuff[6] != 0 )
    {
        printf("Errorcode # %d from StreamConfig read.\n", (unsigned int)recBuff[6]);
        return -1;
    }

    return 0;
}



//Sends a StreamStart low-level command to start streaming.
int LabJackU3Adapter::StreamStart(HANDLE hDevice)
{
    uint8 sendBuff[2], recBuff[4];
    int sendChars, recChars;

    sendBuff[0] = (uint8)(0xA8);  //CheckSum8
    sendBuff[1] = (uint8)(0xA8);  //command byte

    //Sending command to U3
    sendChars = LJUSB_Write(hDevice, sendBuff, 2);
    if( sendChars < 2 )
    {
        if( sendChars == 0 )
            printf("Error : write failed.\n");
        else
            printf("Error : did not write all of the buffer.\n");
        return -1;
    }

    //Reading response from U3
    recChars = LJUSB_Read(hDevice, recBuff, 4);
    if( recChars < 4 )
    {
        if( recChars == 0 )
            printf("Error : read failed.\n");
        else
            printf("Error : did not read all of the buffer.\n");
        return -1;
    }

    if( normalChecksum8(recBuff, 4) != recBuff[0] )
    {
        printf("Error : read buffer has bad checksum8 (StreamStart).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xA9) || recBuff[3] != (uint8)(0x00) )
    {
        printf("Error : read buffer has wrong command bytes \n");
        return -1;
    }

    if( recBuff[2] != 0 )
    {
        printf("Errorcode # %d from StreamStart read.\n", (unsigned int)recBuff[2]);
        return -1;
    }

    return 0;
}

//Reads the StreamData low-level function response in a loop.  All voltages from
//the stream are stored in the voltages 2D array.
int LabJackU3Adapter::StreamData_example(HANDLE hDevice, u3CalibrationInfo *caliInfo, int isDAC1Enabled)
{
    uint16 voltageBytes, checksumTotal;
    long startTime, endTime;
    double hardwareVersion;
    int recBuffSize, recChars, backLog, autoRecoveryOn;
    int packetCounter, currChannel, scanNumber;
    int i, j, k, m;
    int totalPackets;  //The total number of StreamData responses read
    int numDisplay;  //Number of times to display streaming information
    int numReadsPerDisplay;  //Number of packets to read before displaying 
                             //streaming information
    int readSizeMultiplier;  //Multiplier for the StreamData receive buffer size
    int responseSize;  //The number of bytes in a StreamData response
                       //(differs with SamplesPerPacket)

    numDisplay = 10;
    numReadsPerDisplay = 25;
    readSizeMultiplier = 1;
    responseSize = 14 + SamplesPerPacket*2;

    /* Each StreamData response contains (SamplesPerPacket / NumChannels) * readSizeMultiplier
     * samples for each channel.
     * Total number of scans = (SamplesPerPacket / NumChannels) * readSizeMultiplier * numReadsPerDisplay * numDisplay
     */
    double voltages[(SamplesPerPacket/NumChannels)*readSizeMultiplier*numReadsPerDisplay*numDisplay][NumChannels];
    uint8 recBuff[responseSize*readSizeMultiplier];

    packetCounter = 0;
    currChannel = 0;
    scanNumber = 0;
    totalPackets = 0;
    recChars = 0;
    autoRecoveryOn = 0;
    recBuffSize = 14 + SamplesPerPacket*2;
    hardwareVersion = caliInfo->hardwareVersion;

    printf("Reading Samples...\n");

    startTime = getTickCount();

    //for( i = 0; ; i++ )
    //{
        for( j = 0; j < numReadsPerDisplay; j++ )
        {
            /* For USB StreamData, use Endpoint 3 for reads.  You can read the
             * multiple StreamData responses of 64 bytes only if 
             * SamplesPerPacket is 25 to help improve streaming performance.  In
             * this example this multiple is adjusted by the readSizeMultiplier
             * variable.
             */

            //Reading stream response from U3
            recChars = LJUSB_Stream(hDevice, recBuff, responseSize*readSizeMultiplier);
            if( recChars < responseSize*readSizeMultiplier )
            {
                if( recChars == 0 )
                    printf("Error : read failed (StreamData).\n");
                else
                    printf("Error : did not read all of the buffer, expected %d bytes but received %d(StreamData).\n", responseSize*readSizeMultiplier, recChars);
                return -1;
            }

            //Checking for errors and getting data out of each StreamData
            //response
            for( m = 0; m < readSizeMultiplier; m++ )
            {
                totalPackets++;

                checksumTotal = extendedChecksum16(recBuff + m*recBuffSize, recBuffSize);
                if( (uint8)((checksumTotal / 256) & 0xFF) != recBuff[m*recBuffSize + 5] )
                {
                    printf("Error : read buffer has bad checksum16(MSB) (StreamData).\n");
                    return -1;
                }

                if( (uint8)(checksumTotal & 0xFF) != recBuff[m*recBuffSize + 4] )
                {
                    printf("Error : read buffer has bad checksum16(LBS) (StreamData).\n");
                    return -1;
                }

                checksumTotal = extendedChecksum8(recBuff + m*recBuffSize);
                if( checksumTotal != recBuff[m*recBuffSize] )
                {
                    printf("Error : read buffer has bad checksum8 (StreamData).\n");
                    return -1;
                }

                if( recBuff[m*recBuffSize + 1] != (uint8)(0xF9) ||
                    recBuff[m*recBuffSize + 2] != 4 + SamplesPerPacket ||
                    recBuff[m*recBuffSize + 3] != (uint8)(0xC0) )
                {
                    printf("Error : read buffer has wrong command bytes (StreamData).\n");
                    return -1;
                }

                if( recBuff[m*recBuffSize + 11] == 59 )
                {
                    if( !autoRecoveryOn )
                    {
                        printf("\nU3 data buffer overflow detected in packet %d.\nNow using auto-recovery and reading buffered samples.\n", totalPackets);
                        autoRecoveryOn = 1;
                    }
                }
                else if( recBuff[m*recBuffSize + 11] == 60 )
                {
                    printf("Auto-recovery report in packet %d: %d scans were dropped.\nAuto-recovery is now off.\n", totalPackets, recBuff[m*recBuffSize + 6] + recBuff[m*recBuffSize + 7]*256);
                    autoRecoveryOn = 0;
                }
                else if( recBuff[m*recBuffSize + 11] != 0 )
                {
                    printf("Errorcode # %d from StreamData read.\n", (unsigned int)recBuff[11]);
                    return -1;
                }

                if( packetCounter != (int)recBuff[m*recBuffSize + 10] )
                {
                    printf("PacketCounter (%d) does not match with with current packet count (%d)(StreamData).\n", recBuff[m*recBuffSize + 10], packetCounter);
                    return -1;
                }

                backLog = (int)recBuff[m*48 + 12 + SamplesPerPacket*2];

                for( k = 12; k < (12 + SamplesPerPacket*2); k += 2 )
                {
                    voltageBytes = (uint16)recBuff[m*recBuffSize + k] + (uint16)recBuff[m*recBuffSize + k+1]*256;

                    if( hardwareVersion >= 1.30 )
                        getAinVoltCalibrated_hw130(caliInfo, currChannel, 31, voltageBytes, &(voltages[scanNumber][currChannel]));
                    else
                        getAinVoltCalibrated(caliInfo, isDAC1Enabled, 31, voltageBytes, &(voltages[scanNumber][currChannel]));

                    currChannel++;
                    if( currChannel >= NumChannels )
                    {
                        currChannel = 0;
                        scanNumber++;
                    }
                }

                if( packetCounter >= 255 )
                    packetCounter = 0;
                else
                    packetCounter++;
            }
        }
	
	manalog01.clear();
	manalog02.clear();
	manalog03.clear();
	manalog04.clear();
	manalog05.clear();
	manalog06.clear();
	//double **data = (double**) voltages;
	for(k=0; k< SamplesPerPacket;k++)
	{
	manalog01.addValue(voltages[k][1]);
	manalog02.addValue(voltages[k][2]);
	manalog03.addValue(voltages[k][3]);
	manalog04.addValue(voltages[k][4]);
	manalog05.addValue(voltages[k][5]);
	manalog06.addValue(voltages[k][6]);	
	}
        sendChangedData(); 	

    endTime = getTickCount();
    printf("\nRate of samples: %.0lf samples per second\n", ((scanNumber*NumChannels) / ((endTime-startTime) / 1000.0)));
    printf("Rate of scans: %.0lf scans per second\n\n", (scanNumber / ((endTime - startTime) / 1000.0)));

}

//Sends a StreamStop low-level command to stop streaming.
int LabJackU3Adapter::StreamStop(HANDLE hDevice)
{
    uint8 sendBuff[2], recBuff[4];
    int sendChars, recChars;

    sendBuff[0] = (uint8)(0xB0);  //CheckSum8
    sendBuff[1] = (uint8)(0xB0);  //Command byte

    //Sending command to U3
    sendChars = LJUSB_Write(hDevice, sendBuff, 2);
    if( sendChars < 2 )
    {
        if( sendChars == 0 )
            printf("Error : write failed (StreamStop).\n");
        else
            printf("Error : did not write all of the buffer (StreamStop).\n");
        return -1;
    }

    //Reading response from U3
    recChars = LJUSB_Read(hDevice, recBuff, 4);
    if( recChars < 4 )
    {
        if( recChars == 0 )
            printf("Error : read failed (StreamStop).\n");
        else
            printf("Error : did not read all of the buffer (StreamStop).\n");
        return -1;
    }

    if( normalChecksum8(recBuff, 4) != recBuff[0] )
    {
        printf("Error : read buffer has bad checksum8 (StreamStop).\n");
        return -1;
    }

    if( recBuff[1] != (uint8)(0xB1) || recBuff[3] != (uint8)(0x00) )
    {
        printf("Error : read buffer has wrong command bytes (StreamStop).\n");
        return -1;
    }

    if( recBuff[2] != 0 )
    {
        printf("Errorcode # %d from StreamStop read.\n", (unsigned int)recBuff[2]);
        return -1;
    }

    return 0;
}


///u3 functions end here


void LabJackU3Adapter::disconnect()
{
  if (mConnected)
  {
    unavailable();
    mConnected = false;
  }
StreamStop(hDevice);

}



void LabJackU3Adapter::start()
{
startServer();
}

void LabJackU3Adapter::stop()
{
stopServer();
}

LabJackU3Adapter::~LabJackU3Adapter()
{
disconnect(); 
}

void LabJackU3Adapter::gatherDeviceData()
{

  mAvail.available();
StreamStop(hDevice);
StreamStart(hDevice);    
StreamData_example(hDevice, &caliInfo, dac1Enabled);
//cleanup();
 
}

	
