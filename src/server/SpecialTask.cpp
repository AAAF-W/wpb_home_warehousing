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
#include "SpecialTask.h"

CSpecialTask::CSpecialTask()
{
    ros::NodeHandle n;
    special_task_sub  = n.subscribe<wpr_warehousing_monitor::RobotData>("/wpr_warehousing/special_task",10,&CSpecialTask::SpecialTaskCallback,this);
    cliGetChName = n.serviceClient<waterplus_map_tools::GetChargerByName>("/waterplus/get_charger_name");
    navi_pose_pub = n.advertise<wpr_warehousing_monitor::RobotPose>("/wpr_warehousing/ext_navi_pose",10);
    robot_cmd_pub = n.advertise<std_msgs::Int32MultiArray>("/wpr_warehousing/ext_cmd",10);
}

CSpecialTask::~CSpecialTask()
{
}

void CSpecialTask::SpecialTaskCallback(const wpr_warehousing_monitor::RobotData::ConstPtr& msg)
{
    int nRobotID = msg->robot_id;
    if( nRobotID <=0 || nRobotID > ROBOT_NUM)
    {
        ROS_ERROR("[CSpecialTask] RobotID = %d , No this one!",nRobotID);
        return;
    }
    ROS_INFO("[CSpecialTask]RobotID = %d , Cmd= %s,  Data= %s",nRobotID,msg->cmd.data.c_str(),msg->data.data.c_str());

    if(msg->cmd.data == "charging")
    {
        TaskGoCharge(nRobotID, msg->data.data);
    }
}

void CSpecialTask::TaskGoCharge(int inRobotID, std::string inChargerName)
{
    std::string strChargerName = inChargerName;
    waterplus_map_tools::GetChargerByName srvN;
    srvN.request.name = strChargerName;
    if (cliGetChName.call(srvN))
    {
        std::string name = srvN.response.name;
        float x = srvN.response.pose.position.x;
        float y = srvN.response.pose.position.y;
        tf::Quaternion q(srvN.response.pose.orientation.x,srvN.response.pose.orientation.y,srvN.response.pose.orientation.z,srvN.response.pose.orientation.w);
        float yaw = tf::getYaw(q);
        ROS_INFO("Get_charger_name: name = %s (%.2f,%.2f) %.2f", srvN.request.name.c_str(),x,y,yaw);
        // 计算其对面坐标
        float dist_dock = 1.0;
        float dx = dist_dock * cos(yaw);
        float dy  = dist_dock * sin(yaw);
        float ready_x = x +dx;
        float ready_y = y+dy;
        float ready_yaw = yaw + 3.14159;
        ROS_INFO("ready_pose (%.2f,%.2f) %.2f", ready_x,ready_y,ready_yaw);
        wpr_warehousing_monitor::RobotPose navi_msg;
        navi_msg.robot_id = inRobotID;
        navi_msg.x = ready_x;
        navi_msg.y = ready_y;
        navi_msg.yaw = ready_yaw;
        navi_pose_pub.publish(navi_msg);
        // 到达目标点后进入充电坞
        std_msgs::Int32MultiArray cmd_msg;
        cmd_msg.data.push_back(inRobotID);
        cmd_msg.data.push_back(CMD_DOCKING);
        robot_cmd_pub.publish(cmd_msg);
    }
    else
    {
        ROS_ERROR("No charger named ( %s )", srvN.request.name.c_str());
    }
}