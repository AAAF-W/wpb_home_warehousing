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
#ifndef _MISSION_MANAGER_H
#define _MISSION_MANAGER_H
#include "Struct.h"
#include "Common.h"
#include "StringHelper.h"
#include "ServerCmdSend.h"
#include <list>
#include <math.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <waterplus_map_tools/GetWaypointByName.h>
#include <waterplus_map_tools/GetChargerByName.h>

class CMissionManager
{
public:
    CMissionManager();
    virtual ~CMissionManager();
    void Initial();
    void UpdateAll();
    void DisplayMissions();
    void UpdateTaskState();
    virtual void TaskComplete(stTask* inTask){};
    void UpdateMissionState();
    virtual void MissionComplete(stMission* inMission){};
    void AddNewMission(stMission* inMission);
    void UpdateTeamMatesPose(int inID, stCommandMsg* inCmd);
    void AssignTasksToRobots();
    bool RobotIDValid(int inID);
    bool RobotInActivedMission(int inID);
    void ChangeRobotTaskState(int inID,int inTaskState);
    int GetRobotInfoState(int inID);
    bool GetTaskGoto(string inWaypoint, stTask* outTask);
    bool GetTaskToCharger(string inChargerName, float inDist, stTask* outTask);
    void ExtCommand(int inRobotID, int inCommand);
    void ExtGoto(int inRobotID, string inWaypoint);
    void ExtNaviPose(int inRobotID, float inX, float inY, float inYaw);
    void ForceRobotMove(int inRobotID, float inX, float inY, float inYaw);
    void ForceRobotPose(int inRobotID, float inX, float inY, float inYaw);
    bool SetRobotPoseToWaypoint(int inRobotID, string inWaypoint);
    void ForceCommandAll(int inCommand);
    void TeleopRobotVelocity(int inRobotID, float inVelX, float inVelY, float inVelAngular);
    void ExtPathFollow(int inRobotID, int inPathIndex);
    float GetDistRobotWaypoint(int inRobotID, string inWaypoint);
    void ResetRobotsState();
    void SwitchToNextMission();
    list<stMission> mission_list;
    stRobotState* robot_list;
    bool bRobotWaiting[ROBOT_NUM];
    CServerCmdSend* arCmdSend;
    CStringHelper str_helper;
    ros::ServiceClient cliGetWPName;
    ros::ServiceClient cliGetChName;
    bool bPause;
};
#endif
