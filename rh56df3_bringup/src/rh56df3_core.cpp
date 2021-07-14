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
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>
#include "driver/RH56DF3_Driver.h"
#include <math.h>
#include <vector>

using namespace std;

typedef struct
{
    float position[7];
    int velocity[7];
}st_wpm_pose;


static CRH56DF3_Driver hand_driver;
static double fDegToAng = 3.1415926/180;
static double fAngToDeg = 180/3.1415926;
static int pos_send[6];

//控制关节角度
void SetAnglesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    int nNumJoint = msg->position.size();
    if(nNumJoint > 6)
    {
        nNumJoint = 6;
    }
    ROS_INFO("[SetAnglesCallback] ---------------");
    for(int i=0;i<nNumJoint;i++)
    {
        pos_send[i] = msg->position[i];
        ROS_INFO("[SetAnglesCallback] angle_%d  = %.2f", i ,msg->position[i]);
    }
    hand_driver.SetAngles(pos_send);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"rh56df3_core");
    ros::NodeHandle n;

    ros::Subscriber joint_ctrl_degree_sub = n.subscribe("/rh56df3/set_angles",30,&SetAnglesCallback);
    
    ros::NodeHandle n_param("~");
    hand_driver.nID = 1;
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/rh56df3");
    hand_driver.Open(strSerialPort.c_str(),115200);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    sensor_msgs::JointState msg;
    std::vector<std::string> joint_name(6);
    std::vector<double> joint_pos(6);
 
    joint_name[0] = "joint_0";
    joint_name[1] = "joint_1";
    joint_name[2] = "joint_2";
    joint_name[3] = "joint_3";
    joint_name[4] = "joint_4";
    joint_name[5] = "joint_5";
    joint_pos[0] = 0.0f;
    joint_pos[1] = 0.0f;
    joint_pos[2] = 0.0f;
    joint_pos[3] = 0.0f;
    joint_pos[4] = 0.0f;
    pos_send[5] = 0.0f;
    
    ros::Rate r(30);

    r.sleep();
    int nCount = 0;

    // for(int i=0;i<4;i++)
    //     pos_send[i] = 100;
    // pos_send[4] = 2000;
    // pos_send[5] = 0;
    // hand_driver.SetAngles(pos_send);

    while(n.ok())
    {
        //hand_driver.ReadNewData();
        //ROS_INFO("[hand_driver.nParseCount]= %d",hand_driver.nParseCount);
        
        // msg.header.stamp = ros::Time::now();
        // msg.header.seq ++;

        // double fTmp = 0;
        // for(int i=0;i<6;i++)
        // {
        //     fTmp = hand_driver.nRecvJointPos[i];
        //     joint_pos[i] = (fTmp-90)*fDegToAng;
        // }
        // joint_pos[2] -= 20*fDegToAng;
        // joint_pos[4] += 15*fDegToAng;
        // msg.name = joint_name;
        // msg.position = joint_pos;
        // joint_state_pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }
}