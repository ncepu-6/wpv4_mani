/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <driver/RH56DF3_Driver.h>
#include <math.h>

static bool bFirstQuart = true;

CRH56DF3_Driver::CRH56DF3_Driver()
{
   	m_SendBuf = new unsigned char[1024];
	memset(m_SendBuf, 0, 1024);
	memset(m_ParseBuf, 0, 128);
	m_nRecvIndex = 0;
	m_lastRecv = 0;
	m_bFrameStart = false;
	m_nFrameLength = 9;

	nParseCount = 0;
	
	nID = 0;
	
	for (int i = 0; i < 7;i++)
	{
		nRecvJointPos[i] = 0;
		nRecvJointCurrent[i] = 0;
		arPosition[i] = 0;
	}
	nRecvJointPos[6] = 35000;
}
    
CRH56DF3_Driver::~CRH56DF3_Driver()
{
	delete []m_SendBuf;
}


void CRH56DF3_Driver::Parse(unsigned char inData)
{
	// m_ParseBuf[m_nRecvIndex] = inData;

	// if (m_lastRecv == 0x55 && inData == 0xAA && m_bFrameStart == 0)
	// {
	// 	m_bFrameStart = 1;
	// 	m_ParseBuf[0] = m_lastRecv;
	// 	m_ParseBuf[1] = inData;
	// 	m_nRecvIndex = 2;
	// 	m_lastRecv = 0x00;
	// 	return;
	// }

	// if (m_bFrameStart)
	// {
		
	// 	m_ParseBuf[m_nRecvIndex] = inData;
	// 	m_nRecvIndex++;

	// 	if (m_nRecvIndex >= m_nFrameLength)
	// 	{
	// 		m_DisRecv();
	// 		m_ParseFrame();
	// 		m_bFrameStart = false;
	// 	}

	// 	if (m_nRecvIndex >= 128)
	// 	{
	// 		m_bFrameStart = 0;
	// 	}
	// }
	// else
	// 	m_lastRecv = inData;
}

void CRH56DF3_Driver::m_Split2Bytes(unsigned char *inTarg, short inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned short temp;
	memcpy(&temp, &inSrc, sizeof(short));
	inTarg[0] = (unsigned char)temp & 0x00ff;

	temp >>= 8;

	inTarg[1] = (unsigned char)temp & 0x00ff;
}


void CRH56DF3_Driver::m_Split4Bytes(unsigned char *inTarg, int inSrc)
{
	if (inTarg == NULL)
	{
		return;
	}

	static unsigned int temp;
	memcpy(&temp, &inSrc, sizeof(int));
	inTarg[3] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[2] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[1] = (unsigned char)temp & 0x00ff;
	temp >>= 8;
	inTarg[0] = (unsigned char)temp & 0x00ff;
}


short CRH56DF3_Driver::m_WordFromChar(unsigned char *inBuf)
{
	static short wtemp;
	wtemp = 0;
	wtemp = *(inBuf);

	wtemp <<= 8;
	wtemp |= *(inBuf + 1);

	return wtemp;
}

int CRH56DF3_Driver::m_IntFromChar(unsigned char *inBuf)
{
	int itemp;
	itemp = 0;
	itemp = *(inBuf);

	itemp <<= 8;
	itemp |= *(inBuf + 1);

	itemp <<= 8;
	itemp |= *(inBuf + 2);

	itemp <<= 8;
	itemp |= *(inBuf + 3);

	return itemp;
}


void CRH56DF3_Driver::m_ParseFrame()
{
	nParseCount ++;
	for (int i = 0; i < 6; i++)
	{
		nRecvJointPos[i] = m_ParseBuf[3 + i];
	}
	// printf("Recv  1=%d  2=%d  3=%d  4=%d  5=%d  6=%d\n",
	// nRecvJointPos[0],
	// nRecvJointPos[1],
	// nRecvJointPos[2],
	// nRecvJointPos[3],
	// nRecvJointPos[4],
	// nRecvJointPos[5]
	// );
}


void CRH56DF3_Driver::m_DisRecv()
{
	
}


void CRH56DF3_Driver::m_CalSendSum()
{
	int nLen = m_SendBuf[3] + 5;

	m_SendBuf[nLen - 1] = 0x00;
	for (int i = 2; i < nLen - 1; i++)
	{
		m_SendBuf[nLen - 1] += m_SendBuf[i];
	}
}

void CRH56DF3_Driver::SetAngles(int* inAngle)
{
	m_SendBuf[0] = 0xEB;
	m_SendBuf[1] = 0x90;
	m_SendBuf[2] = nID;	//id
	m_SendBuf[3] = 12 + 3;	//len
	m_SendBuf[4] = 0x12;	//write
	m_SendBuf[5] = 0xCE;	//Address_L
	m_SendBuf[6] = 0x05;	//Address_H

	for (int i = 0; i < 6; i++)
	{
		m_Split2Bytes(&m_SendBuf[7+i*2], (short) inAngle[i]);
	}

	m_CalSendSum();

	Send(m_SendBuf, m_SendBuf[3] + 5);

	printf("[CRH56DF3] ");
	for(int i=0;i<m_SendBuf[3]+5;i++)
		printf("%.2X ",m_SendBuf[i]);
	printf("\n");
}
