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
#include "DataCenter.h"

static float arm_4_x = 2.00;
static float arm_4_y = -1.3;
static float arm_4_z = 0.6;
static float arm_4_yaw = -1.57;

static float arm_5_x = 4.5;
static float arm_5_y = -4.3;
static float arm_5_z = 0.6;
static float arm_5_yaw = 1.57;

CDataCenter::CDataCenter()
{
    robot_base[0] = "robot_1/base_footprint";
    robot_base[1] = "robot_2/base_footprint";
    robot_base[2] = "robot_3/base_footprint";
    ros::NodeHandle n;
    pub_joints[0] = n.advertise<sensor_msgs::JointState>("/robot_1/joint_states",100);
    pub_joints[1] = n.advertise<sensor_msgs::JointState>("/robot_2/joint_states",100);
    pub_joints[2] = n.advertise<sensor_msgs::JointState>("/robot_3/joint_states",100);
    marker_pub = n.advertise<visualization_msgs::Marker>("/wpr_warehousing/robot_txt", 100);
    robot_ip_pub = n.advertise<wpr_warehousing_monitor::RobotData>("/wpr_warehousing/robot_ip", 100);
}

CDataCenter::~CDataCenter()
{
}

void CDataCenter::RecvNewPackage(stRobotInfoMsg* inInfo)
{
    int nRobotID = inInfo->id;
    //ROS_INFO("CDataCenter::RecvNewPackage ID = %d",nRobotID);
    if( pMisMgr->RobotIDValid(nRobotID) == true )
    {
        // 复制数据
        memcpy( &(pMisMgr->robot_list[nRobotID-1].info) , inInfo , sizeof(stRobotInfoMsg) );
        pMisMgr->robot_list[nRobotID-1].package_recv ++;
        /*获取client端的IP地址*/
        //printf("receive package from %s\n" , inet_ntoa(addr.sin_addr));
        static char tmpIP[16];
        memcpy(tmpIP,inet_ntoa(addr.sin_addr),16);
        
        const char* curIP = pMisMgr->arCmdSend[nRobotID-1].arRemoteIP;
        int res = strncmp(tmpIP,curIP,16);
        if(res != 0)
        {
            int nPort = 20200+nRobotID;
            pMisMgr->arCmdSend[nRobotID-1].InitUDPClient(tmpIP,nPort);
            ROS_WARN("[New Robot] ID= %d   IP= %s   Port= %d",nRobotID,tmpIP,nPort);
        }
    }
    
}

void CDataCenter::PublishRobotIP()
{
    wpr_warehousing_monitor::RobotData msg;
    for(int i=0 ; i < ROBOT_NUM; i++)
    {
        if(pMisMgr->arCmdSend[i].bInited == true)
        {
            msg.robot_id = i+1;
            msg.data.data = pMisMgr->arCmdSend[i].arRemoteIP;
            robot_ip_pub.publish(msg);
        }
    }
}

void CDataCenter::UpdateRobotTF()
{
    for(int i=0;i<ROBOT_TF_NUM;i++)
    {
        stRobotInfoMsg* pInfo = &(pMisMgr->robot_list[i].info);
        tf::Transform robot_tf;
        if(pMisMgr->robot_list[i].state != T_ST_OFFLINE)
        {
            robot_tf.setOrigin( tf::Vector3(pInfo->map_x, pInfo->map_y, 0) );
        }
        else
        {
            robot_tf.setOrigin( tf::Vector3(-100, -100, 100) );     //如果机器人不在线，将其显示在界面外
        }
        tf::Quaternion q;
        q.setEuler(0,0,pInfo->map_yaw);
        robot_tf.setRotation(q);
        odom_broadcaster.sendTransform(tf::StampedTransform(robot_tf, ros::Time::now(), "map", robot_base[i]));

        PublishJointStates(&(pMisMgr->robot_list[i].info));

        DrawRobotText(i+1,robot_base[i]);
    }
}

void CDataCenter::UpdateEnvTF()
{
    // 实验室房间模型
    // tf::Transform tf_env;
    // tf_env.setOrigin( tf::Vector3(0, 0, 0) );
    // tf::Quaternion q;
    // q.setEuler(0,0,0);
    // tf_env.setRotation(q);
    // odom_broadcaster.sendTransform(tf::StampedTransform(tf_env, ros::Time::now(), "map", "environment/base_link"));
}

void CDataCenter::PublishJointStates(const stRobotInfoMsg* inInfo)
{
    int nRobotID = inInfo->id;
    if( nRobotID < 0 || nRobotID > ROBOT_TF_NUM)
        return;

    if(inInfo->dev_type == DEV_WPB_HOME)
    {
        static sensor_msgs::JointState wpb_joints_msg;
        static std::vector<std::string> wpb_joint_name(2);
        static std::vector<double> wpb_joint_pos(2);
        wpb_joint_name[0] = "kinect_height";
        wpb_joint_name[1] = "kinect_pitch";
        for(int i=0;i<2;i++)
        {
            wpb_joint_pos[i] = inInfo->joint_pos[i];
        }
        wpb_joints_msg.header.stamp = ros::Time::now();
        wpb_joints_msg.header.seq ++;
        wpb_joints_msg.name = wpb_joint_name;
        wpb_joints_msg.position = wpb_joint_pos;
        pub_joints[nRobotID-1].publish(wpb_joints_msg);
    }
}

void CDataCenter::DrawRobotText(int inRobotID, std::string inBase)
{
    text_marker.header.frame_id = inBase;
    text_marker.ns = "text";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inRobotID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.3;
    text_marker.color.r = 1.0;
    text_marker.color.g = 0;
    text_marker.color.b = 0;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = 0;
    text_marker.pose.position.y = 0;
    text_marker.pose.position.z = 1.7;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    std::ostringstream stringStream;
    stringStream << inRobotID;
    std::string retStr = stringStream.str();
    text_marker.text = retStr;

    marker_pub.publish(text_marker);
}

void CDataCenter::CheckRecvPackage()
{
    for(int i=0;i<ROBOT_NUM;i++)
    if(pMisMgr->robot_list[i].package_recv > 0)
    {
        if(pMisMgr->robot_list[i].state == T_ST_OFFLINE)
        {
            pMisMgr->robot_list[i].state = T_ST_WAIT;
            // 新连接上来的机器人先发STOP
            pMisMgr->arCmdSend[i].cmd_msg.command = CMD_STOP;
            pMisMgr->arCmdSend[i].SendCmd();
        }
        pMisMgr->robot_list[i].package_recv_last = pMisMgr->robot_list[i].package_recv;
        pMisMgr->robot_list[i].package_recv = 0;

    }
    else
    {
        pMisMgr->robot_list[i].state = T_ST_OFFLINE;
    }
    
}