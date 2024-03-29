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
#include "PlanPathRecv.h"

CPlanPathRecv::CPlanPathRecv()
{
    bFrameStart = 0;
    nParseIndex = 0;
    nLenToParse = sizeof(stPathMsg);
    m_byteLast = 0;
    arParseBuf = new unsigned char[1024];
    memset(&path_recv,0,sizeof(stPathMsg));
    nRobotNum = 0;
}

CPlanPathRecv::~CPlanPathRecv()
{
    delete[] arParseBuf;
    delete[] path_pub;
}

int CPlanPathRecv::Initial(int inRobotNum)
{
    nRobotNum = inRobotNum;
    path_pub = new ros::Publisher[nRobotNum];
    for(int i=0;i<nRobotNum;i++)
    {
        std::ostringstream stringStream;
        stringStream << "/wpr_warehousing/path_" << (i+1);
        std::string path_topic = stringStream.str();
        path_pub[i] = n.advertise<nav_msgs::Path>(path_topic, 10);
    }
}

int CPlanPathRecv::Receive(char* inBuf, int inLen)
{
    /*显示client端的网络地址*/
    //printf("receive %d from %s\n" , inLen , inet_ntoa(addr.sin_addr));
    // printf("[UDP_Recv] ");
    // for(int i=0;i<inLen;i++)
    // {
    //     printf("%.2X ",inBuf[i]);
    // }
    // printf("\n");

    /*解析*/
    for(int i=0;i<inLen;i++)
    {
        Parse(inBuf[i]);
    }

}

void CPlanPathRecv::Parse(unsigned char inChar)
{
    if(bFrameStart == 0)
    {
        if(m_byteLast == 0x55 && inChar == 0xaa)
        {
            bFrameStart = 1;
            arParseBuf[0] = 0x55;
            arParseBuf[1] = 0xaa;
            nParseIndex = 2;
        }
        else
        {
            m_byteLast = inChar;
        }
    }
    else
    {
        //开始缓存
        arParseBuf[nParseIndex] = inChar;
        nParseIndex ++;

        if(nParseIndex >= nLenToParse)
        {
            ParseFrame((unsigned char*)arParseBuf,nLenToParse);
            bFrameStart = 0;
            nParseIndex = 0;
            m_byteLast = 0;
        }
    }
}

void CPlanPathRecv::ParseFrame(unsigned char* inBuf,int inLen)
{
    memcpy(&path_recv,inBuf,inLen);
    RecvNewPackage(&path_recv);
}


void CPlanPathRecv::RecvNewPackage(stPathMsg* inPath)
{
    ROS_WARN("[CPlanPathRecv] nPathNum = %d",inPath->len);
    if(inPath->id <= 0 || inPath->id > nRobotNum)
        return;
    int nPathNum = inPath->len;
    path_msg.poses.resize(nPathNum);
    for(int i=0;i<nPathNum;i++)
    {
        path_msg.poses[i].pose.position.x = inPath->path_x[i];
        path_msg.poses[i].pose.position.y = inPath->path_y[i];
        path_msg.poses[i].pose.position.z = 0;
    }
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    int nPubIndex = inPath->id -1;
    path_pub[nPubIndex].publish(path_msg);
}