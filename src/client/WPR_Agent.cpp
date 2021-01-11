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
#include "WPR_Agent.h"

CWPR_Agent::CWPR_Agent()
{
    
    memset(&robot_fake,0,sizeof(stRobotFake));
    memset(&(arInfoSend.info_msg),0,sizeof(stRobotInfoMsg));
    robot_fake.nLastPathIndex = -1;
    //////////////
    robot_fake.nDevType = DEV_WPB_HOME;
    robot_fake.map_x = 1.0;
    robot_fake.map_y = -4.30;
    robot_fake.map_yaw = 4.712385;
    robot_fake.joint_pos[1] = 1.57f;
    robot_fake.joint_pos[2] = -0.7f;
    //////////////
    fTimeScale = 2;
}

CWPR_Agent::~CWPR_Agent()
{
}

void CWPR_Agent::SetID(int inID)
{
    arInfoSend.SetDevType(DEV_WPB_HOME);
    arInfoSend.SetID(inID);
    string ServerIP = "127.0.0.1";
    arInfoSend.InitUDPClient(ServerIP.c_str(),21202);
}

void CWPR_Agent::RecvNewPackage(stCommandMsg* inCmd)
{
    // ROS_WARN("CWPR_Agent ID = %d  CMD= %s",inCmd->id, str_helper.GetCommandStr(inCmd->command).c_str());
    if(inCmd->id < 1 || inCmd->id > ROBOT_FAKE_NUM)
    {
        ROS_WARN("CWPR_Agent::RecvNewPackage Invalid ID = %d",inCmd->id);
        return;
    }

    int nRobotID = inCmd->id;
    if(robot_fake.nLastRecvCmd == inCmd->command)
    {
        if(inCmd->command == CMD_ROBOT_GOTO && (robot_fake.target_x !=  inCmd->map_x || robot_fake.target_y !=  inCmd->map_y || robot_fake.target_yaw !=  inCmd->map_yaw))
        {}
        else if(inCmd->command == CMD_ROBOT_POSE && (robot_fake.target_x !=  inCmd->map_x || robot_fake.target_y !=  inCmd->map_y || robot_fake.target_yaw !=  inCmd->map_yaw))
        {}
        else if(inCmd->command == CMD_ROBOT_TELEOP)
        {}
        else if(inCmd->command == CMD_ROBOT_MOVE)
        {}
        else if(inCmd->command == CMD_FOLLOWER)
        {}
        else if(inCmd->command == CMD_FOLLOW_PATH && inCmd->path_index != robot_fake.nLastPathIndex)
        {
           robot_fake.nLastPathIndex = inCmd->path_index;
        }
        else
        {
            return;
        }
    }

    switch (inCmd->command)
    {
    case CMD_STOP:
        robot_fake.nState = RBT_ST_STOP;
        break;
    case CMD_ROBOT_GOTO:
        robot_fake.nState = RBT_ST_GOTO;
        robot_fake.target_x = inCmd->map_x;
        robot_fake.target_y = inCmd->map_y;
        robot_fake.target_yaw = inCmd->map_yaw;
        break;
    case CMD_GRAB_BOX:
        robot_fake.nState = RBT_ST_BOX_DETECT;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_GRAB_COLOR:
        robot_fake.nState = RBT_ST_BOX_GRAB;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_PLACE_COLOR:
        robot_fake.nState = RBT_ST_PLACE_COLOR;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_PLACE_MOBILE:
        robot_fake.nState = RBT_ST_MOBILE_DETECT;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_GRAB_MOBILE:
        robot_fake.nState = RBT_ST_GM_DETECT;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_PLACE_PALLET:
        robot_fake.nState = RBT_ST_PALLET_DETECT;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_DOCKING:
        robot_fake.nState = RBT_ST_DOCK_FACETO;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_CHARGING:
        robot_fake.nState = RBT_ST_DOCK_FACETO;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_LEAVE_DOCK:
        robot_fake.nState = RBT_ST_DOCK_LEAVE;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_CONVEY_BOX:
        robot_fake.nState = RBT_ST_CONVEY_BOX;
        robot_fake.nCount = 30*fTimeScale;
        break;
    case CMD_LT_POSE_INIT:
        robot_fake.nState = RBT_ST_LT_INIT;
        robot_fake.nCount = 30*fTimeScale;
    case CMD_LT_POSE_SET:
        robot_fake.nState = RBT_ST_STOP;
        break;
    case CMD_ROBOT_POSE:
        robot_fake.map_x =  inCmd->map_x;
        robot_fake.map_y =  inCmd->map_y;
        robot_fake.map_yaw =  inCmd->map_yaw;
        break;
    case CMD_ROBOT_TELEOP:
        robot_fake.nState= RBT_ST_TELEOP;
        robot_fake.vel_x =  inCmd->vel_x;
        robot_fake.vel_y =  inCmd->vel_y;
        robot_fake.vel_angular =  inCmd->vel_angular;
        break; 
    case CMD_FOLLOW_PATH:
        robot_fake.nState = RBT_ST_FOLLOW_PATH;
        robot_fake.nPathIndex = inCmd->path_index;
        robot_fake.nPointIndex = 0;
        break;
    case CMD_ROBOT_MOVE:
        robot_fake.nState = RBT_ST_MOVE;
        robot_fake.target_x = inCmd->map_x;
        robot_fake.target_y = inCmd->map_y;
        robot_fake.target_yaw = inCmd->map_yaw;
        break;
    case CMD_FOLLOWER:
        robot_fake.nState = RBT_ST_FOLLOWER;
        robot_fake.target_x = inCmd->map_x;
        robot_fake.target_y = inCmd->map_y;
        robot_fake.target_yaw = inCmd->map_yaw;
        break;
    default:
        ROS_WARN("CWPR_Agent::RecvNewPackage Unkown CMD = %d",inCmd->command);
        break;
    }
    robot_fake.nLastRecvCmd = inCmd->command;
}

void CWPR_Agent::UpdateAll()
{
    //for(int i=2;i<ROBOT_FAKE_NUM;i++)  //只模拟固定机械臂
    //for(int i=0;i<ROBOT_FAKE_NUM;i++)//模拟所有机器人
    {
        if(robot_fake.nState == RBT_ST_GOTO || robot_fake.nState == RBT_ST_MOVE || robot_fake.nState == RBT_ST_FOLLOWER)
        {
            float dx = robot_fake.target_x - robot_fake.map_x;
            float dy = robot_fake.target_y - robot_fake.map_y;
            float dyaw = robot_fake.target_yaw - robot_fake.map_yaw;
            if(fabs(dx) > 0.02 || fabs(dy) > 0.02 || fabs(dyaw) > 0.03)
            {
                if(dx > 0) dx=0.01; else dx = -0.01;
                robot_fake.map_x += dx;
                if(dy > 0) dy=0.01; else dy = -0.01;
                robot_fake.map_y += dy;
                if(dyaw > 0) dyaw=0.03; else dyaw = -0.03;
                robot_fake.map_yaw += dyaw;
                //printf("[Goto] id= %d ( %.2f , %.2f )  %.2f \n",i+1,robot_fake.map_x,robot_fake.map_y,robot_fake.map_yaw);
            }
            else
            {
                robot_fake.nState = RBT_ST_ARRIVED;
            }
            
        }
      

        if(robot_fake.nState == RBT_ST_TELEOP)
        {
                robot_fake.map_x += robot_fake.vel_x/30;
                robot_fake.map_y += robot_fake.vel_y/30;
                robot_fake.map_yaw += robot_fake.vel_angular/30;
        }

        if(robot_fake.nState == RBT_ST_FOLLOW_PATH)
        {
            int path_index = robot_fake.nPathIndex;
            if(path_index >= PATH_HOLDER_NUM)
            {
                robot_fake.nState = RBT_ST_FOLLOW_PATH_END;
            }
            else if(robot_fake.nPointIndex >= robot_fake.path_holder[path_index].arPoint.size())
            {
                robot_fake.nState = RBT_ST_FOLLOW_PATH_END;
            }
            else
            {
                int pnt_index =  robot_fake.nPointIndex;
                float dx = robot_fake.path_holder[path_index].arPoint[pnt_index].x - robot_fake.map_x;
                float dy = robot_fake.path_holder[path_index].arPoint[pnt_index].y - robot_fake.map_y;
                 if(fabs(dx) > 0.04 || fabs(dy) > 0.04)
                {
                    if(dx > 0) dx=0.01; else dx = -0.01;
                    robot_fake.map_x += dx;
                    if(dy > 0) dy=0.01; else dy = -0.01;
                    robot_fake.map_y += dy;
                }
                else
                {
                    pnt_index ++;
                    robot_fake.nPointIndex = pnt_index;
                 }
                 //ROS_WARN("CWPR_Agent pntIndex = %d  size=%d",robot_fake.nPointIndex ,(int)robot_fake.path_holder[path_index].arPoint.size());
            }
        }

        arInfoSend.info_msg.state = robot_fake.nState;
        arInfoSend.info_msg.map_x = robot_fake.map_x;
        arInfoSend.info_msg.map_y = robot_fake.map_y;
        arInfoSend.info_msg.map_yaw = robot_fake.map_yaw;
        arInfoSend.info_msg.cmd_recv = robot_fake.nLastRecvCmd;
        for(int j=0;j<10;j++)
        {
            arInfoSend.info_msg.joint_pos[j] = robot_fake.joint_pos[j];
        }
        arInfoSend.SendInfo();
    }
}