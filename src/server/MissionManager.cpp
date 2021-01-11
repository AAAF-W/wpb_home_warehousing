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
#include "MissionManager.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define UI_WIDTH 1900
#define UI_HEIGHT 1000

using namespace std;
using namespace cv;
cv::Mat ui_image( UI_HEIGHT, UI_WIDTH, CV_8UC3, Scalar(100,0,0) );

CMissionManager::CMissionManager()
{
    cv::namedWindow("Info",CV_WINDOW_NORMAL);
    cvResizeWindow("Info", UI_WIDTH,UI_HEIGHT+40); 

    bPause = false;
}

CMissionManager::~CMissionManager()
{
    delete []arCmdSend;
    delete []robot_list;
}

void CMissionManager::Initial()
{
    robot_list = new stRobotState[ROBOT_NUM];
    memset(robot_list,0,ROBOT_NUM*sizeof(stRobotState));
    /////////////////////////////////////////////////////////////////////
    robot_list[0].info.dev_type = DEV_WPB_HOME; //避免出现启明1模型残留在原点
    robot_list[0].info.id = 1;
    /////////////////////////////////////////////////////////////////////
    arCmdSend = new CServerCmdSend[ROBOT_NUM];
    for(int i=0;i<ROBOT_NUM;i++)
    {
        arCmdSend[i].SetID(i+1);
        //arCmdSend[i].InitUDPClient("127.0.0.1",20201);
    }

    ros::NodeHandle nh;
    cliGetWPName = nh.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    cliGetChName = nh.serviceClient<waterplus_map_tools::GetChargerByName>("/waterplus/get_charger_name");
}

void CMissionManager::UpdateAll()
{
    UpdateTaskState();
    if(bPause == false)
    {
        UpdateMissionState();
        AssignTasksToRobots();
    }
    else
    {
        // 暂停状态，全部机器人发送STOP指令
        for(int i=0;i<ROBOT_NUM;i++)
        {
            arCmdSend[i].cmd_msg.command = CMD_STOP;
            arCmdSend[i].SendCmd();
        }
    }
    
    DisplayMissions();
}

void CMissionManager::UpdateTaskState()
{
    // [一] 遍历任务 List ,更新以及完成的Task和机器人的状态
    list<stMission>::iterator it;
    for(it = mission_list.begin();it!=mission_list.end();it++)
    {
        // 未在执行状态的任务，直接跳过
        if((*it).state != M_ST_IN_PROGRESS)
            continue;
        
        int nTCount = (*it).arTask.size();
        for(int i=0;i<nTCount;i++)
        {
            // 不在线的机器人，直接让其任务完成
            int nRobotID = (*it).arTask[i].command.id;
            if(robot_list[nRobotID-1].state == T_ST_OFFLINE)
                ChangeRobotTaskState(nRobotID,T_ST_DONE);

            // 检查任务是否完成
            if((*it).arTask[i].command.command == CMD_STOP)
            {
                (*it).arTask[i].state = T_ST_DONE;
                ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                TaskComplete(&((*it).arTask[i]));
            }

            if((*it).arTask[i].command.command == CMD_ROBOT_GOTO)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_ARRIVED)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_GRAB_BOX)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_BOX_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_GRAB_COLOR)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_BOX_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_GRAB_MOBILE)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_GM_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_PLACE_MOBILE)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_MOBILE_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_GRAB_PALLET)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_GP_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_PLACE_PALLET)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_PALLET_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_LEAVE_DOCK)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_LEAVE_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_DOCKING)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_DOCK_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_CHARGING)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_CHARGE_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_GRAB_STORAGE)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_GS_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_PLACE_STORAGE)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_PS_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_CONVEY_BOX)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_EXIT_BOX)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_FOLLOW_PATH)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_FOLLOW_PATH_END)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_PLACE_COLOR)
            {
                int rbt_state = GetRobotInfoState((*it).arTask[i].command.id);
                if(rbt_state == RBT_ST_PLACE_COLOR_DONE)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                }
            }

            if((*it).arTask[i].command.command == CMD_ROBOT_MOVE)
            {
                float target_x = (*it).arTask[i].command.map_x;
                float target_y = (*it).arTask[i].command.map_y;
                float target_yaw = (*it).arTask[i].command.map_yaw;
                int robot_id = (*it).arTask[i].command.id;
                float robot_x = robot_list[robot_id - 1].info.map_x;
                float robot_y = robot_list[robot_id - 1].info.map_y;
                float robot_yaw = robot_list[robot_id - 1].info.map_yaw;
                if(sqrt((robot_x - target_x)*(robot_x - target_x) + (robot_y - target_y)*(robot_y - target_y)) < 0.1 && fabs(robot_yaw-target_yaw) < 0.08)
                {
                    (*it).arTask[i].state = T_ST_DONE;
                    ChangeRobotTaskState((*it).arTask[i].command.id,T_ST_DONE);
                    TaskComplete(&((*it).arTask[i]));
                    // 发个STOP，不然机器人还会继续晃
                    arCmdSend[robot_id-1].cmd_msg.command = CMD_STOP;
                    arCmdSend[robot_id-1].SendCmd();
                }
            }
        }
    }
}

void CMissionManager::UpdateMissionState()
{
    // [二] 遍历 任务List ,将Tasks已经完成的任务标注出来，完结它，以生成新的任务（生成新任务的代码放到子类实现）
    list<stMission>::iterator it;
    for(it = mission_list.begin();it!=mission_list.end();it++)
    {
        // 未在执行状态的任务，直接跳过
        if((*it).state != M_ST_IN_PROGRESS)
            continue;

        bool bMissionComplete = true;
        int nTCount = (*it).arTask.size();
        for(int i=0;i<nTCount;i++)
        {
            // 检查任务是否完成
            if((*it).arTask[i].state != T_ST_DONE)
            {
                bMissionComplete = false;
            }
        }
        if(bMissionComplete == true)
        {
            (*it).state = M_ST_COMPLETE;
        }
    }

    // 第二次遍历 任务List，将完结的任务删除，并激活回调函数
    for(it = mission_list.begin();it!=mission_list.end();)
    {
        if((*it).state == M_ST_COMPLETE)
        {
            MissionComplete(&(*it));
            // 删除任务
            it = mission_list.erase(it);
        }
        else
        {
            it++;
        }
    }
}

void CMissionManager::AddNewMission(stMission* inMission)
{
    inMission->state = M_ST_WAIT;
    int nTCount = inMission->arTask.size();
    for(int i=0;i<nTCount;i++)
    {
        inMission->arTask[i].state = T_ST_WAIT;
        // 检查分配任务的机器人是否处于待命状态
        int robot_id = inMission->arTask[i].command.id;
        if(RobotIDValid(robot_id) == true)
        {
            // 将机器人状态设置为待命
            robot_list[robot_id-1].state = T_ST_WAIT;
        }
    }

    mission_list.push_back(*inMission);
}

void CMissionManager::AssignTasksToRobots()
{
    for(int i=0;i<ROBOT_NUM;i++)
    {
        bRobotWaiting[i] = false;
    }
    // [三] 遍历任务 List ,将新生的任务激活，并将指令发送给机器人
    list<stMission>::iterator it;
    for(it = mission_list.begin();it!=mission_list.end();it++)
    {
        if((*it).state == M_ST_WAIT)
        {
            bool bMissionCanStart = true;
            // 查看其task列表所涉及的所有机器人都已经处于待命状态
            int nTCount = (*it).arTask.size();
            for(int i=0;i<nTCount;i++)
            {
                // 检查分配任务的机器人是否处于待命状态
                int robot_id = (*it).arTask[i].command.id;
                if(RobotIDValid(robot_id) == true)
                {
                    // (1)如果机器人正在执行前一个任务，该任务不能启动
                    if(robot_list[robot_id-1].state == T_ST_IN_PROGRESS)
                    {
                        bMissionCanStart = false;
                    }
                    // (2)如果某个任务里某机器人处于忙碌状态，其他机器人设为等待，避免后面任务给他分配单体任务
                    if(robot_list[robot_id-1].state == T_ST_IN_PROGRESS)
                    {
                        for(int j=0;j<nTCount;j++)
                        {
                            int t_robot_id = (*it).arTask[j].command.id;
                            bRobotWaiting[t_robot_id-1] = true;
                        }
                    }
                    // (3)如果机器人在另外一个已经启动但是未完成的任务里，也不能启动此任务
                    if(RobotInActivedMission(robot_id) == true || bRobotWaiting[robot_id-1] == true)
                    {
                        bMissionCanStart = false;
                    }
                }
                else
                {
                    // 机器人id不合法，该任务不能启动
                    bMissionCanStart = false;
                }
            }

            // 若任务可执行，分配给机器人
            if(bMissionCanStart == true)
            {
                (*it).state = M_ST_IN_PROGRESS;
                // 将机器人状态也设置成忙碌状态
                int nTCount = (*it).arTask.size();
                for(int i=0;i<nTCount;i++)
                {
                    // 检查分配任务的机器人ID是合法
                    int robot_id = (*it).arTask[i].command.id;
                    if(RobotIDValid(robot_id) == true)
                    {
                        robot_list[robot_id-1].state = T_ST_IN_PROGRESS;
                    }
                }
            }
        }

        // 将处于执行状态的任务分发给机器人
        if((*it).state == M_ST_IN_PROGRESS)
        {
                   // printf("-----------------------------\n");
            // 发送task给机器人
            int nTCount = (*it).arTask.size();
            for(int i=0;i<nTCount;i++)
            {
                // 检查分配任务的机器人ID是合法
                int robot_id = (*it).arTask[i].command.id;
                if(RobotIDValid(robot_id) == true)
                {
                    memcpy(&(arCmdSend[robot_id-1].cmd_msg),&((*it).arTask[i].command),sizeof(stCommandMsg));
                    UpdateTeamMatesPose(robot_id,&(arCmdSend[robot_id-1].cmd_msg));
                    arCmdSend[robot_id-1].SendCmd();
                    //printf("机器人%d 执行任务%d\n",robot_id,arCmdSend[robot_id-1].cmd_msg.command);
                }
            }
        }
    }
}

void CMissionManager::UpdateTeamMatesPose(int inID, stCommandMsg* inCmd)
{
    if(ROBOT_NUM < 3)
        return;
    // 1号机队友坐标
    if(inID == 1)
    {
        inCmd->team_mate_x[0] = robot_list[1].info.map_x;
        inCmd->team_mate_y[0] = robot_list[1].info.map_y;
        inCmd->team_mate_x[1] = robot_list[2].info.map_x;
        inCmd->team_mate_y[1] = robot_list[2].info.map_y;
    }

    // 2号机队友坐标
    if(inID == 2)
    {
        inCmd->team_mate_x[0] = robot_list[0].info.map_x;
        inCmd->team_mate_y[0] = robot_list[0].info.map_y;
        inCmd->team_mate_x[1] = robot_list[2].info.map_x;
        inCmd->team_mate_y[1] = robot_list[2].info.map_y;
    }

    // 3号机队友坐标
    if(inID == 3)
    {
        inCmd->team_mate_x[0] = robot_list[0].info.map_x;
        inCmd->team_mate_y[0] = robot_list[0].info.map_y;
        inCmd->team_mate_x[1] = robot_list[1].info.map_x;
        inCmd->team_mate_y[1] = robot_list[1].info.map_y;
    }
}

bool CMissionManager::RobotIDValid(int inID)
{
    if(inID > 0 && inID <= ROBOT_NUM)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool CMissionManager::RobotInActivedMission(int inID)
{
    // 遍历任务 List ,查看此机器人是否在一个已经激活但未完成的任务里
    list<stMission>::iterator it;
    for(it = mission_list.begin();it!=mission_list.end();it++)
    {
        if((*it).state == M_ST_IN_PROGRESS)
        {
            int nTCount = (*it).arTask.size();
            for(int i=0;i<nTCount;i++)
            {
                int robot_id = (*it).arTask[i].command.id;
                if(robot_id == inID)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

void CMissionManager::ChangeRobotTaskState(int inID,int inTaskState)
{
    if(RobotIDValid(inID) == true)
    {
        robot_list[inID-1].state = inTaskState;
    }
    else
    {
        printf("[WARN] MissionManager::ChangeRobotTaskState( %d , %d )\n",inID,inTaskState);
    }
    
}

int CMissionManager::GetRobotInfoState(int inID)
{
    int ret = 0;
    if(RobotIDValid(inID) == true)
    {
        ret = robot_list[inID-1].info.state;
    }
    else
    {
        printf("[WARN] MissionManager::GetRobotInfoState( %d )\n",inID);
    }
    return ret;
}

bool CMissionManager::GetTaskGoto(string inWaypoint, stTask* outTask)
{
    bool res = false;
    waterplus_map_tools::GetWaypointByName srvN;
    srvN.request.name = inWaypoint;
    if (cliGetWPName.call(srvN))
    {
        std::string name = srvN.response.name;
        float x = srvN.response.pose.position.x;
        float y = srvN.response.pose.position.y;
        ROS_INFO("[GetTaskGoto] Get_wp_name: name = %s (%.2f,%.2f)", inWaypoint.c_str(),x,y);
        outTask->command.command = CMD_ROBOT_GOTO;
        outTask->command.map_x = x;
        outTask->command.map_y = y;
        geometry_msgs::Quaternion orientation = srvN.response.pose.orientation;    
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));    
        double yaw, pitch, roll;    
        mat.getEulerYPR(yaw, pitch, roll);
        outTask->command.map_yaw = yaw;
        outTask->state = T_ST_WAIT;
        res =  true;
    }
    else
    {
        ROS_ERROR("[GetTaskGoto] Failed to call service get_waypoint_name");
        res =  false;
    }
    return res;
}

bool CMissionManager::GetTaskToCharger(string inChargerName, float inDist, stTask* outTask)
{
     std::string strChargerName = inChargerName;
    waterplus_map_tools::GetChargerByName srvN;
    srvN.request.name = strChargerName;
    if (cliGetChName.call(srvN))
    {
        float x = srvN.response.pose.position.x;
        float y = srvN.response.pose.position.y;
        tf::Quaternion q(srvN.response.pose.orientation.x,srvN.response.pose.orientation.y,srvN.response.pose.orientation.z,srvN.response.pose.orientation.w);
        float yaw = tf::getYaw(q);
        ROS_INFO("Get_charger_name: name = %s (%.2f,%.2f) %.2f", strChargerName.c_str(),x,y,yaw);
        // 计算其对面坐标
        float dx = inDist * cos(yaw);
        float dy  = inDist * sin(yaw);
        float ready_x = x +dx;
        float ready_y = y+dy;
        float ready_yaw = yaw + 3.14159;
        ROS_INFO("face_dock_pose (%.2f,%.2f) %.2f", ready_x,ready_y,ready_yaw);
        outTask->command.command = CMD_ROBOT_GOTO;
        outTask->command.map_x = ready_x;
        outTask->command.map_y = ready_y;
        outTask->command.map_yaw = ready_yaw;
        outTask->state = T_ST_WAIT;
        return true;
    }
    else
    {
        ROS_ERROR("No charger named ( %s )", srvN.request.name.c_str());
        return false;
    }
}

void CMissionManager::ExtCommand(int inRobotID, int inCommand)
{
    stMission new_mis;
    new_mis.mission = "ext_command";
    new_mis.state = M_ST_WAIT;
    stTask nt;
    memset(&nt,0,sizeof(nt));
    nt.command.id = inRobotID;
    nt.command.command = inCommand;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionManager::ExtGoto(int inRobotID, string inWaypoint)
{
    stMission new_mis;
    new_mis.mission = "ext_navi_name";
    stTask nt;
    memset(&nt,0,sizeof(nt));
    bool res = GetTaskGoto(inWaypoint,&nt);
    if(res == true)
    {
        nt.command.command = CMD_ROBOT_GOTO;
        nt.command.id = inRobotID;
        new_mis.arTask.push_back(nt);
        AddNewMission(&new_mis);
    }
}

void CMissionManager::ExtNaviPose(int inRobotID, float inX, float inY, float inYaw)
{
    stMission new_mis;
    new_mis.mission = "ext_navi_pose";
    stTask nt;
    memset(&nt,0,sizeof(nt));
    nt.command.id = inRobotID;
    nt.command.command = CMD_ROBOT_GOTO;
    nt.command.map_x = inX;
    nt.command.map_y = inY;
    nt.command.map_yaw = inYaw;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionManager::ForceRobotMove(int inRobotID, float inX, float inY, float inYaw)
{
     if(inRobotID < 1 || inRobotID > ROBOT_NUM)
        return;
    //[方法一] 直接发一条
    // int nRobotIndex = inRobotID - 1;
    // arCmdSend[nRobotIndex].cmd_msg.command = CMD_ROBOT_MOVE;
    // arCmdSend[nRobotIndex].cmd_msg.map_x = inX;
    // arCmdSend[nRobotIndex].cmd_msg.map_y = inY;
    // arCmdSend[nRobotIndex].cmd_msg.map_yaw = inYaw;
    // arCmdSend[nRobotIndex].SendCmd();

    //[方法二] 插入一条任务
    stMission new_mis;
    new_mis.mission = "robot_move";
    stTask nt;
    memset(&nt,0,sizeof(nt));
    nt.command.id = inRobotID;
    nt.command.command = CMD_ROBOT_MOVE;
    nt.command.map_x = inX;
    nt.command.map_y = inY;
    nt.command.map_yaw = inYaw;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

void CMissionManager::ForceRobotPose(int inRobotID, float inX, float inY, float inYaw)
{
    if(inRobotID < 1 || inRobotID > ROBOT_NUM)
        return;

    int nRobotIndex = inRobotID - 1;
    arCmdSend[nRobotIndex].cmd_msg.command = CMD_ROBOT_POSE;
    arCmdSend[nRobotIndex].cmd_msg.map_x = inX;
    arCmdSend[nRobotIndex].cmd_msg.map_y = inY;
    arCmdSend[nRobotIndex].cmd_msg.map_yaw = inYaw;
    arCmdSend[nRobotIndex].SendCmd();
}

bool CMissionManager::SetRobotPoseToWaypoint(int inRobotID, string inWaypoint)
{
    bool res = false;
    waterplus_map_tools::GetWaypointByName srvN;
    srvN.request.name = inWaypoint;
    if (cliGetWPName.call(srvN))
    {
        std::string name = srvN.response.name;
        float x = srvN.response.pose.position.x;
        float y = srvN.response.pose.position.y;
        ROS_INFO("[SetRobotPoseToWaypoint] Get_wp_name: name = %s (%.2f,%.2f)", inWaypoint.c_str(),x,y);
        geometry_msgs::Quaternion orientation = srvN.response.pose.orientation;    
        tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));    
        double yaw, pitch, roll;    
        mat.getEulerYPR(yaw, pitch, roll);
        ForceRobotPose(inRobotID, x, y, yaw);
        res =  true;
    }
    else
    {
        ROS_ERROR("[SetRobotPoseToWaypoint] Failed to call service get_waypoint_name");
        res =  false;
    }
    return res;
}

void CMissionManager::ForceCommandAll(int inCommand)
{
    mission_list.clear();
    bPause = false;
    for(int i=0;i<ROBOT_NUM;i++)
    {
        arCmdSend[i].cmd_msg.command = inCommand;
        arCmdSend[i].SendCmd();
    }
}

void CMissionManager::TeleopRobotVelocity(int inRobotID, float inVelX, float inVelY, float inVelAngular)
{
    if(inRobotID < 1 || inRobotID > ROBOT_NUM)
        return;

    int nRobotIndex = inRobotID - 1;
    arCmdSend[nRobotIndex].cmd_msg.command = CMD_ROBOT_TELEOP;
    arCmdSend[nRobotIndex].cmd_msg.vel_x = inVelX;
    arCmdSend[nRobotIndex].cmd_msg.vel_y = inVelY;
    arCmdSend[nRobotIndex].cmd_msg.vel_angular = inVelAngular;
    arCmdSend[nRobotIndex].SendCmd();
}

void CMissionManager::ExtPathFollow(int inRobotID, int inPathIndex)
{
     if(inRobotID < 1 || inRobotID > ROBOT_NUM)
        return;
    stMission new_mis;
    new_mis.mission = "ext_path_follow";
    new_mis.state = M_ST_WAIT;
    stTask nt;
    memset(&nt,0,sizeof(nt));
    nt.command.id = inRobotID;
    nt.command.command = CMD_FOLLOW_PATH;
    nt.command.path_index = inPathIndex;
    nt.state = T_ST_WAIT;
    new_mis.arTask.push_back(nt);
    AddNewMission(&new_mis);
}

float CMissionManager::GetDistRobotWaypoint(int inRobotID, string inWaypoint)
{
    if(inRobotID < 1 || inRobotID > ROBOT_NUM)
        return 0.0;
    int nRobotIndex = inRobotID - 1;
    float fRobotX = robot_list[nRobotIndex].info.map_x;
    float fRobotY = robot_list[nRobotIndex].info.map_y;
     waterplus_map_tools::GetWaypointByName srvN;
    srvN.request.name = inWaypoint;
    if (cliGetWPName.call(srvN))
    {
        std::string name = srvN.response.name;
        float x = srvN.response.pose.position.x;
        float y = srvN.response.pose.position.y;
        float dist = sqrt((fRobotX-x)*(fRobotX-x) + (fRobotY-y)*(fRobotY-y));
        return dist;
    }
    else
    {
        ROS_ERROR("[GetDistRobotWaypoint] Failed to call service get_waypoint_name");
        return 0.0;
    }
}

void CMissionManager::ResetRobotsState()
{
    for(int i=0;i<ROBOT_NUM;i++)
    {
        if(robot_list[i].state != T_ST_OFFLINE)
        {
            robot_list[i].state = T_ST_DONE;
        }
    }
}

void CMissionManager::SwitchToNextMission()
{
    list<stMission>::iterator it;
    for(it = mission_list.begin();it!=mission_list.end();it++)
    {
        // 未在执行状态的任务，直接跳过
        if((*it).state != M_ST_IN_PROGRESS)
            continue;

        // 找到第一个正在执行中的任务，将每个任务成员的认为变成Stop，他们会立刻完成任务
        int nTCount = (*it).arTask.size();
        for(int i=0;i<nTCount;i++)
        {
            // 检查分配任务的机器人ID是合法
            int robot_id = (*it).arTask[i].command.id;
            if(RobotIDValid(robot_id) == true)
            {
                (*it).arTask[i].command.command = CMD_STOP;
                printf("[SwitchToNextMission]机器人%d 任务变更为 CMD_STOP %d 立即完成!\n",robot_id,(*it).arTask[i].command.command);
            }
        }
    }
}

inline float twodec(float n) { return floor(n * 100 + 0.5) / 100; }

void CMissionManager::DisplayMissions()
{
    // 背景色
    ui_image.setTo(cv::Scalar(100, 0, 0));
    // 字体属性
	int font_face = cv::FONT_HERSHEY_COMPLEX_SMALL; 
	double font_scale = 1;
	int thickness = 1;
    ostringstream stringStream;
    cv::Scalar text_color;
    // [一] 任务列表
    // 标题
    cv::putText(ui_image, "MISSION & TASK", Point(10,30), cv::FONT_HERSHEY_TRIPLEX, font_scale, cv::Scalar(0, 255, 255), thickness,8, 0);
    // 遍历任务 List
    list<stMission>::iterator it;
    int nMCount = 0;
    int nBlockHeight = 160;
    for(it = mission_list.begin();it!=mission_list.end();it++)
    {
        int x_offset = 20 + nMCount*350;
        int y_offset = 70;
        // 画群体任务矩形背景
        int mission_state = (*it).state;
        int nTCount = (*it).arTask.size();
        if(mission_state == M_ST_IN_PROGRESS)
        {
            rectangle(ui_image, Point(x_offset-5,y_offset-30), Point(x_offset-2+340,y_offset+nBlockHeight*nTCount +50), cv::Scalar(0, 155, 0), CV_FILLED, 8, 0);
            text_color = cv::Scalar(255, 255, 255);
        }
        else
        {
            rectangle(ui_image, Point(x_offset-5,y_offset-30), Point(x_offset-2+340,y_offset+nBlockHeight*nTCount +50), cv::Scalar(100, 100, 100), CV_FILLED, 8, 0);
            text_color = cv::Scalar(150, 150, 150);
        }
        // [1] 群体任务名称
		string mission_name = (*it).mission;
        cv::putText(ui_image, mission_name, Point(x_offset,y_offset), font_face, font_scale*1.5, cv::Scalar(0, 255, 255), thickness,8, 0);
        stringStream.str("");
        stringStream << "st= " << str_helper.GetMissionStateStr(mission_state);
        string m_state_str = stringStream.str();
        cv::putText(ui_image, m_state_str, Point(x_offset,y_offset+35), font_face, font_scale, cv::Scalar(255, 255, 255), thickness,8, 0);
        // [2] 单体任务列表
        for(int i=0;i<nTCount;i++)
        {
            // 画单体任务矩形背景
            if(mission_state == M_ST_IN_PROGRESS)
                rectangle(ui_image, Point(x_offset,y_offset + nBlockHeight*i+50), Point(x_offset-5+340,y_offset + 80 + nBlockHeight*i + 30 * 4), cv::Scalar(0, 0, 100), CV_FILLED, 8, 0);
            else
                rectangle(ui_image, Point(x_offset,y_offset + nBlockHeight*i+50), Point(x_offset-5+340,y_offset + 80 + nBlockHeight*i + 30 * 4), cv::Scalar(70, 70, 70), CV_FILLED, 8, 0);
            
            // [2-1] 单体任务机器人ID
            stringStream.str("");
            int robot_id = (*it).arTask[i].command.id;
            stringStream << "robot_id= " << robot_id;
            string id_str = stringStream.str();
            cv::putText(ui_image, id_str, Point(x_offset,y_offset + 70 + nBlockHeight*i), font_face, font_scale, text_color, thickness,8, 0);
            //printf("robot_id= %d\n",(*it).arTask[i].command.id);
            // [2-2] 单体任务指令
            stringStream.str("");
            int cmd = (*it).arTask[i].command.command;
            stringStream << "cmd= " << str_helper.GetCommandStr(cmd);
            string cmd_str = stringStream.str();
            cv::putText(ui_image, cmd_str, Point(x_offset,y_offset + 70 + nBlockHeight*i + 30), font_face, font_scale, text_color, thickness,8, 0);
            // [2-3] 单体任务坐标
            stringStream.str("");
            float map_x = (*it).arTask[i].command.map_x;
            stringStream << "map_x= " << twodec(map_x);
            string map_x_str = stringStream.str();
            cv::putText(ui_image, map_x_str, Point(x_offset,y_offset + 70 + nBlockHeight*i + 30 * 2), font_face, font_scale, text_color, thickness,8, 0);
            stringStream.str("");
            float map_y = (*it).arTask[i].command.map_y;
            stringStream << "map_y= " << twodec(map_y);
            string map_y_str = stringStream.str();
            cv::putText(ui_image, map_y_str, Point(x_offset,y_offset + 70 + nBlockHeight*i + 30 * 3), font_face, font_scale, text_color, thickness,8, 0);stringStream.str("");
            float map_yaw = (*it).arTask[i].command.map_yaw;
            stringStream << "map_yaw= " << twodec(map_yaw);
            string map_yaw_str = stringStream.str();
            cv::putText(ui_image, map_yaw_str, Point(x_offset,y_offset + 70 + nBlockHeight*i + 30 * 4), font_face, font_scale, text_color, thickness,8, 0);
        }
        nMCount ++;
	}

    // [二] 机器人状态列表
    int state_display_y = 650;
    cv::line(ui_image,Point(10,state_display_y-30),Point(UI_WIDTH-10,state_display_y-30),cv::Scalar(255,255,255),1);
    // 标题
    cv::putText(ui_image, "ROBOT STATE", Point(10,state_display_y), cv::FONT_HERSHEY_TRIPLEX, font_scale, cv::Scalar(0, 255, 255), thickness,8, 0);
    int r_count = 0;
    int r_width = 350;
    int r_height = 330;
    // 遍历机器人状态数组
    for(int i=0;i<ROBOT_NUM;i++)
    {
        int x_offset = 20 + r_count * r_width;
        int y_offset = state_display_y + 10;
        // 画机器人状态矩形背景
        if(robot_list[i].state == T_ST_IN_PROGRESS)
        {
            rectangle(ui_image, Point(x_offset-5,y_offset), Point(x_offset+r_width-10,y_offset+r_height), cv::Scalar(0, 155, 0), CV_FILLED, 8, 0);
            rectangle(ui_image, Point(x_offset,y_offset+90), Point(x_offset+r_width-15,y_offset+r_height-5), cv::Scalar(50, 0, 50), CV_FILLED, 8, 0);
            text_color = cv::Scalar(255, 255, 255);
        } else if(robot_list[i].state == T_ST_OFFLINE)
        {
            rectangle(ui_image, Point(x_offset-5,y_offset), Point(x_offset+r_width-10,y_offset+r_height), cv::Scalar(100, 100, 100), CV_FILLED, 8, 0);
            rectangle(ui_image, Point(x_offset,y_offset+90), Point(x_offset+r_width-15,y_offset+r_height-5), cv::Scalar(70, 70, 70), CV_FILLED, 8, 0);
            text_color = cv::Scalar(150, 150, 150);
        }else
        {
            rectangle(ui_image, Point(x_offset-5,y_offset), Point(x_offset+r_width-10,y_offset+r_height), cv::Scalar(100, 100, 00), CV_FILLED, 8, 0);
            rectangle(ui_image, Point(x_offset,y_offset+90), Point(x_offset+r_width-15,y_offset+r_height-5), cv::Scalar(50, 0, 50), CV_FILLED, 8, 0);
            text_color = cv::Scalar(255, 255, 255);
        }
        
        // [3-1] id
        stringStream.str("");
        int robot_id = robot_list[i].info.id;
        stringStream << "ID= " << robot_id;
        string id_str = stringStream.str();
        cv::putText(ui_image, id_str, Point(x_offset,y_offset+20), font_face, font_scale, text_color, thickness,8, 0);
        // [3-2] dev 设备类型
        stringStream.str("");
        int dev_type = robot_list[i].info.dev_type;
        stringStream << "dev= " << str_helper.GetDevTypeStr(dev_type);
        string dev_str = stringStream.str();
        cv::putText(ui_image, dev_str, Point(x_offset,y_offset+20 + 30), font_face, font_scale, text_color, thickness,8, 0);
        // [3-3] 工作状态 
        stringStream.str("");
        int task_state = robot_list[i].state;
        stringStream << "task_st= " << str_helper.GetTaskStateStr(task_state);
        string task_state_str = stringStream.str();
        cv::putText(ui_image, task_state_str, Point(x_offset,y_offset+20 + 30 *2), font_face, font_scale, text_color, thickness,8, 0);
        // [3-4] 行为状态
        stringStream.str("");
        int info_state = robot_list[i].info.state;
        stringStream << "info_st= " << str_helper.GetRobotStateStr(info_state);
        string info_str = stringStream.str();
        cv::putText(ui_image, info_str, Point(x_offset,y_offset+20 + 30 *3), font_face, font_scale, text_color, thickness,8, 0);
        // [3-5] map_x
        stringStream.str("");
        float map_x = robot_list[i].info.map_x;
        stringStream << "map_x= " << twodec(map_x);
        string map_x_str = stringStream.str();
        cv::putText(ui_image, map_x_str, Point(x_offset,y_offset+20 + 30 *4), font_face, font_scale, text_color, thickness,8, 0);
        // [3-6] map_y
        stringStream.str("");
        float map_y = robot_list[i].info.map_y;
        stringStream << "map_y= " << twodec(map_y);
        string map_y_str = stringStream.str();
        cv::putText(ui_image, map_y_str, Point(x_offset,y_offset+20 + 30 *5), font_face, font_scale, text_color, thickness,8, 0);
        // [3-7] map_yaw
        stringStream.str("");
        float map_yaw = robot_list[i].info.map_yaw;
        stringStream << "map_yaw= " << twodec(map_yaw);
        string map_yaw_str = stringStream.str();
        cv::putText(ui_image, map_yaw_str, Point(x_offset,y_offset+20 + 30 *6), font_face, font_scale, text_color, thickness,8, 0);
         // [3-9] 分配的任务指令 
        stringStream.str("");
        int cmd_recv = robot_list[i].info.cmd_recv;
        stringStream << "cmd= " << str_helper.GetCommandStr(cmd_recv);
        string cmd_str = stringStream.str();
        cv::putText(ui_image, cmd_str, Point(x_offset,y_offset+20 + 30 *8), font_face, font_scale, text_color, thickness,8, 0);
        // [3-10] 接收到的数据包 
        stringStream.str("");
        int package_recv = robot_list[i].package_recv;
        int package_recv_last = robot_list[i].package_recv_last;
        stringStream << "pack= "<<package_recv_last << " / " << package_recv;
        string pack_recv_str = stringStream.str();
        cv::putText(ui_image, pack_recv_str, Point(x_offset,y_offset+20 + 30 *10), font_face, font_scale, text_color, thickness,8, 0);
    
        r_count ++;
    }

	cv::imshow("Info", ui_image);
    cv::waitKey(1);
}
