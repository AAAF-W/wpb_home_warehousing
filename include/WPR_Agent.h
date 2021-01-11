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
#ifndef _WPR_AGENT_H
#define _WPR_AGENT_H

#include "ServerCmdRecv.h"
#include "StringHelper.h"
#include "RobotInfoSend.h"
#include "PathHolder.h"

#define ROBOT_FAKE_NUM 3
#define PATH_HOLDER_NUM 3

typedef struct stRobotFake
{
    int nDevType;
    int nState;
    int nLastRecvCmd;
    float map_x;
    float map_y;
    float map_yaw;
    float target_x;
    float target_y;
    float target_yaw;
    float vel_x;
    float vel_y;
    float vel_angular;
    float joint_pos[10];
    CPathHolder path_holder[PATH_HOLDER_NUM];
    int nPathIndex;
    int nLastPathIndex;
    int nPointIndex;
    int nCount;
}stRobotFake;

class CWPR_Agent : public CServerCmdRecv
{
public:
    CWPR_Agent();
    virtual ~CWPR_Agent();
    void SetID(int inID);
    void RecvNewPackage(stCommandMsg* inCmd);
    void UpdateAll();
    CStringHelper str_helper;
    stRobotFake robot_fake;
    CRobotInfoSend arInfoSend;
    float fTimeScale;
};
#endif
