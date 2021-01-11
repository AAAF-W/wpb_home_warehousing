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

#ifndef _SPECIAL_TASK_H
#define _SPECIAL_TASK_H

#include "ros/ros.h"
#include "Struct.h"
#include "Common.h"
#include "MissionManager.h"
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32MultiArray.h>
#include <wpr_warehousing_monitor/RobotPose.h>
#include <wpr_warehousing_monitor/RobotData.h>
#include <waterplus_map_tools/GetChargerByName.h>

class CSpecialTask
{
public:
    CSpecialTask();
    virtual ~CSpecialTask();
    CMissionManager* pMisMgr;
    ros::Subscriber special_task_sub;
    ros::ServiceClient cliGetChName;
    ros::Publisher navi_pose_pub;
    ros::Publisher robot_cmd_pub;
    void SpecialTaskCallback(const wpr_warehousing_monitor::RobotData::ConstPtr& msg);
    void TaskGoCharge(int inRobotID, std::string inChargerName);
};
#endif //_SPECIAL_TASK_H
