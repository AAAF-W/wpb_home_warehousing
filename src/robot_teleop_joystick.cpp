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

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Joy.h>
#include <wpr_warehousing_monitor/RobotVelocity.h>

using namespace std;

class RobotTeleopJoy
{
public:
  RobotTeleopJoy();
  int teleop_robot_id;
  float lx;
  float ly;
  float ry;
  ros::NodeHandle n;
  ros::Subscriber joy_sub;
  ros::Subscriber robot_id_sub;

  ros::Time current_time;
  ros::Time last_time;
  ros::Publisher robot_vel_pub;
private:
  void JoyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void RobotIdCallback(const std_msgs::Int64::ConstPtr& msg);
};

RobotTeleopJoy::RobotTeleopJoy()
{
  teleop_robot_id = 0;
  lx = 0;
  ly = 0;
  ry = 0;
  robot_vel_pub = n.advertise<wpr_warehousing_monitor::RobotVelocity>("/wpr_warehousing/teleop_robot_velocity",10);
  joy_sub = n.subscribe<sensor_msgs::Joy>("/joy",10,&RobotTeleopJoy::JoyCallback,this);
  robot_id_sub = n.subscribe<std_msgs::Int64>("/wpr_warehousing/teleop_robot_id",10,&RobotTeleopJoy::RobotIdCallback,this);
  current_time = ros::Time::now();
  last_time = ros::Time::now();
 
}

void RobotTeleopJoy::RobotIdCallback(const std_msgs::Int64::ConstPtr& msg)
{
  teleop_robot_id = msg->data;
  ROS_INFO("[RobotTeleopJoy] teleop_robot_id = %d", teleop_robot_id);
}

static float kl = 0.2;
static float kt = 0.5;
void RobotTeleopJoy::JoyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(teleop_robot_id <= 0)
    return;

  ROS_INFO("[RobotTeleopJoy] lx=%.2f ,ly=%d, ry=%.2f",lx,ly,ry);
  lx = joy->axes[1];  //forward & backward
  ly = joy->axes[0];  //shift
  ry = joy->axes[3];

  wpr_warehousing_monitor::RobotVelocity vel_cmd;
  vel_cmd.robot_id = teleop_robot_id;
  vel_cmd.velocity.linear.x = (float)lx*kl;
  vel_cmd.velocity.linear.y = (float)ly*kl;
  vel_cmd.velocity.linear.z = 0;
  vel_cmd.velocity.angular.x = 0;
  vel_cmd.velocity.angular.y = 0;
  vel_cmd.velocity.angular.z = (float)ry*kt;
  robot_vel_pub.publish(vel_cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_teleop_joystick");

  RobotTeleopJoy cRobotTeleopJoy;

  ros::spin();

  return 0;
}