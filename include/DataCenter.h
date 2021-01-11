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

#ifndef _DATA_CENTER_H
#define _DATA_CENTER_H

#include "ros/ros.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include "Struct.h"
#include "Common.h"
#include "RobotInfoRecv.h"
#include "MissionManager.h"
#include <wpr_warehousing_monitor/RobotData.h>

#define ROBOT_TF_NUM  3   //和ROBOT_NUM不同，有的机器人不需要发送tf

class CDataCenter : public CRobotInfoRecv
{
public:
    CDataCenter();
    virtual ~CDataCenter();
    void RecvNewPackage(stRobotInfoMsg* inInfo);
    void UpdateRobotTF();
    void UpdateEnvTF();
    void DrawRobotText(int inRobotID, std::string inBase);
    void PublishJointStates(const stRobotInfoMsg* inInfo);
    void PublishRobotIP();
    void CheckRecvPackage();
    CMissionManager* pMisMgr;
    std::string robot_base[ROBOT_TF_NUM];
    ros::Publisher pub_joints[ROBOT_TF_NUM];
    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher marker_pub;
    ros::Publisher robot_ip_pub;
    visualization_msgs::Marker text_marker;
};
#endif
