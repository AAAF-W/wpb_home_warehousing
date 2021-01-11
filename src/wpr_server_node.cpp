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

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <wpr_warehousing_monitor/RobotPose.h>
#include <wpr_warehousing_monitor/RobotVelocity.h>
#include "WPBMission.h"
#include "DataCenter.h"
#include "SpecialTask.h"
#include "PlanPathRecv.h"

static std::string  robot_1_pose_name = "A";
static std::string  robot_2_pose_name = "B";
static std::string  robot_3_pose_name = "C";

static CWPBMission mission_manager;

void MissionCB(const std_msgs::String::ConstPtr &msg)
{
  int nFindIndex = 0;

  nFindIndex = msg->data.find("start");
  if( nFindIndex >= 0 )
  {
    if(mission_manager.mission_list.size() > 0)
    {
      printf("[提示] 抱歉，上一组任务尚未执行完毕，此条指令被忽略，请稍后再次发送！\n");
      return;
    }
    ROS_WARN("start mission!");
    mission_manager.Preset_Leader_Followers();
    mission_manager.Start();
  }

  nFindIndex = msg->data.find("stop");
  if( nFindIndex >= 0 )
  {
    ROS_WARN("stop mission!");
    mission_manager.bPause = true;
    mission_manager.mission_list.clear();
    mission_manager.ResetRobotsState();
  }

  nFindIndex = msg->data.find("pause");
  if( nFindIndex >= 0 )
  {
    ROS_WARN("pause mission!");
    mission_manager.bPause = true;
  }

  nFindIndex = msg->data.find("continue");
  if( nFindIndex >= 0 )
  {
    ROS_WARN("continue mission!");
    mission_manager.bPause = false;
  }

  nFindIndex = msg->data.find("pose_waypoint");
  if( nFindIndex >= 0 )
  {
    ROS_WARN("pose_waypoint!");
    mission_manager.SetRobotPoseToWaypoint(1, robot_1_pose_name);
    mission_manager.SetRobotPoseToWaypoint(2, robot_2_pose_name);
    mission_manager.SetRobotPoseToWaypoint(3, robot_3_pose_name);
  }

  nFindIndex = msg->data.find("next_mission");
  if( nFindIndex >= 0 )
  {
    ROS_WARN("next_mission!");
    mission_manager.SwitchToNextMission();
  }

  nFindIndex = msg->data.find("signal_1");
  if( nFindIndex >= 0 )
  {
    if(mission_manager.mission_list.size() > 0)
    {
      printf("[提示] 抱歉，上一组任务尚未执行完毕，此条指令被忽略，请稍后再次发送！\n");
      return;
    }
    ROS_WARN("signal_1!");
    mission_manager.Signal_1();
  }

  nFindIndex = msg->data.find("signal_2");
  if( nFindIndex >= 0 )
  {
    if(mission_manager.mission_list.size() > 0)
    {
      printf("[提示] 抱歉，上一组任务尚未执行完毕，此条指令被忽略，请稍后再次发送！\n");
      return;
    }
    ROS_WARN("signal_2");
    mission_manager.Signal_2();
  }

  nFindIndex = msg->data.find("signal_3");
  if( nFindIndex >= 0 )
  {
    if(mission_manager.mission_list.size() > 0)
    {
      printf("[提示] 抱歉，上一组任务尚未执行完毕，此条指令被忽略，请稍后再次发送！\n");
      return;
    }
    ROS_WARN("signal_3!");
    mission_manager.Signal_3();
  }
}

void CommandCB(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if(msg->data.size() < 2)
    return;
  int nRobotID = msg->data[0];
  int nCommand = msg->data[1];
  ROS_WARN("[CommandCB] RobotID= %d Command= %d",nRobotID,nCommand);
  mission_manager.ExtCommand(nRobotID,nCommand);
}

void RobotPoseCB(const wpr_warehousing_monitor::RobotPose::ConstPtr &msg)
{
  int nRobotID = msg->robot_id;
  float x = msg->x;
  float y = msg->y;
  float yaw = msg->yaw;
  ROS_WARN("[RobotPoseCB] RobotID= %d pose= (%.2f , %.2f)  %.2f",nRobotID,x,y,yaw);
  mission_manager.ForceRobotPose(nRobotID,x,y,yaw);
}

void RobotVelocityCB(const wpr_warehousing_monitor::RobotVelocity::ConstPtr &msg)
{
  int nRobotID = msg->robot_id;
  float vel_x = msg->velocity.linear.x;
  float vel_y = msg->velocity.linear.y;
  float vel_angular = msg->velocity.angular.z;
  // ROS_WARN("[RobotVelocityCB] RobotID= %d vel= (%.2f , %.2f)  %.2f",nRobotID,vel_x,vel_y,vel_angular);
  mission_manager.TeleopRobotVelocity(nRobotID,vel_x,vel_y,vel_angular);
}

void NaviNameCB(const std_msgs::MultiArrayDimension::ConstPtr &msg)
{
  int nRobotID = msg->size;
  string strWaypoint = msg->label;
  ROS_WARN("[NaviNameCB] RobotID= %d Waypoint= %s",nRobotID,strWaypoint.c_str());
  mission_manager.ExtGoto(nRobotID,strWaypoint);
}

void NaviPoseCB(const wpr_warehousing_monitor::RobotPose::ConstPtr &msg)
{
   int nRobotID = msg->robot_id;
  float x = msg->x;
  float y = msg->y;
  float yaw = msg->yaw;
  ROS_WARN("[NaviPoseCB] RobotID= %d Pose= (%.2f , %.2f)  yaw= %.2f",nRobotID,x,y,yaw);
  mission_manager.ExtNaviPose(nRobotID,x,y,yaw);
}

void RobotMoveCB(const wpr_warehousing_monitor::RobotPose::ConstPtr &msg)
{
   int nRobotID = msg->robot_id;
  float x = msg->x;
  float y = msg->y;
  float yaw = msg->yaw;
  ROS_WARN("[RobotMoveCB] RobotID= %d Pose= (%.2f , %.2f)  yaw= %.2f",nRobotID,x,y,yaw);
  mission_manager.ForceRobotMove(nRobotID,x,y,yaw);
}

void PathFollowCB(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if(msg->data.size() < 2)
    return;
  int nRobotID = msg->data[0];
  int nPathIndex = msg->data[1];
  ROS_WARN("[PathFollowCB] RobotID= %d nPathIndex= %d",nRobotID,nPathIndex);
  mission_manager.ExtPathFollow(nRobotID,nPathIndex);
}

int main(int argc, char **argv)
{ 
  ros::init(argc,argv, "wpr_server_node");
  ROS_INFO("[wpr_server_node] Start");

  ros::NodeHandle n;

  ros::Subscriber sub_mission = n.subscribe("/wpr_warehousing/mission", 10, MissionCB);
  ros::Subscriber sub_cmd = n.subscribe("/wpr_warehousing/ext_cmd", 10, CommandCB);
  ros::Subscriber sub_navi_name = n.subscribe("/wpr_warehousing/ext_navi_name", 10, NaviNameCB);
  ros::Subscriber sub_navi_pose = n.subscribe("/wpr_warehousing/ext_navi_pose", 10, NaviPoseCB);
  ros::Subscriber sub_robot_pose = n.subscribe("/wpr_warehousing/ext_robot_pose", 10,RobotPoseCB);
  ros::Subscriber sub_robot_move = n.subscribe("/wpr_warehousing/ext_robot_move", 10,RobotMoveCB);
  ros::Subscriber sub_robot_vel = n.subscribe("/wpr_warehousing/teleop_robot_velocity", 10,RobotVelocityCB);
  ros::Subscriber sub_path_follow = n.subscribe("/wpr_warehousing/ext_path_follow", 10, PathFollowCB);

  ros::Rate r(30);

  CSpecialTask special_task;
  special_task.pMisMgr = &mission_manager;
  mission_manager.pSpecialTask = &special_task;

  mission_manager.Initial();
  mission_manager.Init();
  //mission_manager.Start();
  CDataCenter data_center;
  data_center.pMisMgr = &mission_manager;
  data_center.InitUDPServer(21202);

  CPlanPathRecv path_recv;
  path_recv.Initial(3);
  path_recv.InitUDPServer(21203);

  int nChkLoop = 0;

  while(ros::ok())
  {
    mission_manager.UpdateAll();
    data_center.UpdateRobotTF();
    data_center.UpdateEnvTF();
    nChkLoop ++;
    if(nChkLoop > 30)
    {
      data_center.CheckRecvPackage();
      data_center.PublishRobotIP();
      nChkLoop = 0;
    }

    mission_manager.UpdateFollowersPose();

    r.sleep();
    ros::spinOnce();
  }
 
  return 0;
}