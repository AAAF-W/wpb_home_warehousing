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
#include "WPBMission.h"

CWPBMission::CWPBMission()
{
    nLeader_ID = 1;

    nFollower_A_ID = 2;
    fFollower_A_Angle = 30;
    fFollower_A_Distance = 1.5;

    nFollower_B_ID = 3;
    fFollower_B_Angle = -30;
    fFollower_B_Distance = 1.5;
}

CWPBMission::~CWPBMission()
{
}

/**************************************************
 * 任务系统初始化
 * ************************************************/
void CWPBMission::Init()
{
    ros::NodeHandle n;
    follower_pose_pub = n.advertise<geometry_msgs::PoseArray>("/wpb_follower_pose", 10);
}
// 预置任务：编队行进
void CWPBMission::Preset_Leader_Followers()
{
    stMission new_mis;
    stTask nt;
    // [1] Leader不需要设置行为，自由活动

    // [2] 设置Followers
    new_mis.mission = "Followers";
    new_mis.state = M_ST_WAIT;
    new_mis.arTask.clear();
    memset(&nt,0,sizeof(nt));
    nt.command.id = nFollower_A_ID;
    nt.command.follow_angle = fFollower_A_Angle;
    nt.command.follow_dist = fFollower_A_Distance;
    nt.command.command = CMD_FOLLOWER;
    nt.state = T_ST_WAIT; 
    new_mis.arTask.push_back(nt);
    nt.command.id = nFollower_B_ID;
    nt.command.follow_angle = fFollower_B_Angle;
    nt.command.follow_dist = fFollower_B_Distance;
    nt.command.command = CMD_FOLLOWER;
    nt.state = T_ST_WAIT; 
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CWPBMission::Start()
{
    bPause = false;
}

/**************************************************
 * 根据领队的坐标计算跟随者的坐标
 * ************************************************/
void CWPBMission::UpdateFollowersPose()
{
    if(bPause == true)
        return;
    // 查看mission_list里是否有跟随任务
   list<stMission>::iterator it;
    for(it = mission_list.begin();it!=mission_list.end();it++)
    {
        int nFindIndex =(*it).mission.find("Followers");
        if( nFindIndex >= 0 )
        {
           // 先获取Leader的姿态
            float leader_x = robot_list[nLeader_ID-1].info.map_x;
            float leader_y = robot_list[nLeader_ID-1].info.map_y;
            float leader_yaw = robot_list[nLeader_ID-1].info.map_yaw;

            // 计算Follower_A的坐标
            (*it).arTask[0].command.id = nFollower_A_ID;
            CalFollowerPose(leader_x,leader_y,leader_yaw, &((*it).arTask[0].command));

            // 计算Follower_B的坐标
            (*it).arTask[1].command.id = nFollower_B_ID;
            CalFollowerPose(leader_x,leader_y,leader_yaw, &((*it).arTask[1].command));

            // 发布Follower的坐标值给界面显示
            geometry_msgs::PoseArray msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";

            geometry_msgs::Pose pose;
            pose.position.x = leader_x;
            pose.position.y = leader_y;
            msg.poses.push_back(pose); 
            pose.position.x = (*it).arTask[0].command.map_x;
            pose.position.y = (*it).arTask[0].command.map_y;
            msg.poses.push_back(pose); 
            pose.position.x = (*it).arTask[1].command.map_x;
            pose.position.y = (*it).arTask[1].command.map_y;
            msg.poses.push_back(pose);

            follower_pose_pub.publish(msg);
        }
    }
}

static double m_kAngle = M_PI / 180;
void CWPBMission::CalFollowerPose(float inLeaderX,float inLeaderY,float inLeaderYaw,stCommandMsg* outCmd)
{
    float follow_angle = outCmd->follow_angle * m_kAngle;
    float follow_dist = outCmd->follow_dist;

    outCmd->map_x = inLeaderX + (follow_dist * cos((-inLeaderYaw + M_PI + follow_angle)));
	outCmd->map_y = inLeaderY + (follow_dist * -1 *sin((-inLeaderYaw + M_PI + follow_angle)));
    outCmd->map_yaw = inLeaderYaw;
}
/**************************************************
 * 任务切换
 * ************************************************/
void CWPBMission::MissionComplete(stMission* inMission)
{
    ROS_WARN("[CWPBMission] Mission \"%s\" complete!! ",inMission->mission.c_str());
}

void CWPBMission::RobotFollowPathTask(int inRobot,int inPath)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot;
    oss << "path";
    oss << inPath;
    new_mis.mission = oss.str();
    nt.command.id = inRobot;
    nt.command.command = CMD_FOLLOW_PATH;
    nt.command.path_index = inPath;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CWPBMission::RobotGotoTask(int inRobot,string inWaypoint)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot;
    oss << "->";
    oss << inWaypoint;
    new_mis.mission = oss.str();
    GetTaskGoto(inWaypoint,&nt);
    nt.command.id = inRobot;
    nt.command.command = CMD_ROBOT_GOTO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CWPBMission::RobotsSyncGoto(int inRobot_1,string inWaypoint_1,int inRobot_2,string inWaypoint_2)
{
    stMission new_mis;
    stTask nt;
    ostringstream oss;
    oss << inRobot_1;
    oss << "_sync_";
    oss << inRobot_2;
    new_mis.mission = oss.str();
    GetTaskGoto(inWaypoint_1,&nt);
    nt.command.id = inRobot_1;
    nt.command.command = CMD_ROBOT_GOTO;
    nt.state = T_ST_WAIT;
    new_mis.arTask.clear();
    new_mis.arTask.push_back(nt);
    GetTaskGoto(inWaypoint_2,&nt);
    nt.command.id = inRobot_2;
    nt.command.command = CMD_ROBOT_GOTO;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CWPBMission::Signal_1()
{
    
}

void CWPBMission::Signal_2()
{
    
}

void CWPBMission::Signal_3()
{
   
}