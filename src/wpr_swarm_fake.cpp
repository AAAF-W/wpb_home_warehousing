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
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "WPR_Agent.h"

int main(int argc, char **argv)
{
  ros::init(argc,argv, "wpr_swarm_fake"); 
  ROS_WARN("[wpr_swarm_fake] Start");

  ros::NodeHandle n;

  tf::TransformBroadcaster odom_broadcaster;
  float robot_x,robot_y,robot_yaw;
  robot_x = robot_y = robot_yaw = 0;
  tf::Transform robot_tf;

  CWPR_Agent wpr_agent[ROBOT_FAKE_NUM];

  for(int i=0;i<ROBOT_FAKE_NUM;i++)
  {
    int nRobotID = i+1;
    wpr_agent[i].SetID(nRobotID);
    int nPort = 20200+nRobotID;
    wpr_agent[i].InitUDPServer(nPort);
    ROS_WARN("wpr_agent[%d].InitUDPServer(%d)",i,nPort);
  }

  ros::Rate r(30);
  while(ros::ok())
  {
     for(int i=0;i<ROBOT_FAKE_NUM;i++)
    {
      wpr_agent[i].UpdateAll();
    }

    r.sleep();
    ros::spinOnce();
  }
 
  return 0;
}