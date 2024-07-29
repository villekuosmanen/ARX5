#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include "utility.h"
#include "Hardware/can.h"
#include "Hardware/motor.h"
#include "Hardware/teleop.h"
#include "App/arm_control.h"
#include "App/arm_control.cpp"
#include "App/keyboard.h"
#include "App/play.h"
#include "App/solve.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <signal.h>
#include "App/arx_s.h"
#include "Ecat/ecat_base.hpp"
#include "Ecat/ecat_typedef.hpp"

#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"

char phy[] = "enx00e04c3611ee";
int CONTROL_MODE=0;
command cmd;

bool arx_flag2 = false;
void sigint_handler(int sig);
void safe_stop();

ecat::EcatBase Ethercat(1);
can CAN_Handlej;

float calc_cur[7]={};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm4"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);

    arx_arm ARX_ARM((int) CONTROL_MODE);

    // The following are edited based on Cobot Magic code
    ros::Subscriber sub_joint = node.subscribe<sensor_msgs::JointState>("/master/joint_left", 10, 
                                [&ARX_ARM](const sensor_msgs::JointState::ConstPtr& msg) {
                                    ARX_ARM.ros_control_pos_t[0] = msg->position[0];
                                    ARX_ARM.ros_control_pos_t[1] = msg->position[1];
                                    ARX_ARM.ros_control_pos_t[2] = msg->position[2];
                                    ARX_ARM.ros_control_pos_t[3] = msg->position[3];
                                    ARX_ARM.ros_control_pos_t[4] = msg->position[4];
                                    ARX_ARM.ros_control_pos_t[5] = msg->position[5];
                                    ARX_ARM.ros_control_pos_t[6] = msg->position[6];
                                });
    ros::Subscriber sub_pos = node.subscribe<geometry_msgs::PoseStamped>("/master/end_left", 10, 
                                [&ARX_ARM](const geometry_msgs::PoseStamped::ConstPtr& msg) {
                                        ARX_ARM.arx5_cmd.x            = msg->pose.position.x;
                                        ARX_ARM.arx5_cmd.y            = msg->pose.position.y;
                                        ARX_ARM.arx5_cmd.z            = msg->pose.position.z;
                                        ARX_ARM.arx5_cmd.waist_roll   = msg->pose.orientation.x;
                                        ARX_ARM.arx5_cmd.waist_pitch  = msg->pose.orientation.y;
                                        ARX_ARM.arx5_cmd.waist_yaw    = msg->pose.orientation.z;
                                        ARX_ARM.arx5_cmd.gripper      = msg->pose.orientation.w;
                                });
    ros::Publisher pub_current = node.advertise<sensor_msgs::JointState>("/puppet/joint_left", 10);
    ros::Publisher pub_pos = node.advertise<geometry_msgs::PoseStamped>("/puppet/end_left", 10);



    arx5_keyboard ARX_KEYBOARD;

    ros::Rate loop_rate(200);

    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);
    sleep(1);
    Ethercat.EcatStart(phy);
    CAN_Handlej.Send_moto_Cmd1(Ethercat, 1, 0, 0, 0, 0, 0);
    CAN_Handlej.Send_moto_Cmd1(Ethercat, 2, 0, 0, 0, 0, 0);
    CAN_Handlej.Send_moto_Cmd1(Ethercat, 4, 0, 0, 0, 0, 0);
    CAN_Handlej.Enable_Moto(Ethercat,5);
    CAN_Handlej.Enable_Moto(Ethercat,6);
    CAN_Handlej.Enable_Moto(Ethercat,7);
    CAN_Handlej.Enable_Moto(Ethercat,8);
    Ethercat.EcatSyncMsg(); CAN_Handlej.can0_ReceiveFrame(Ethercat); 

    while(ros::ok())
    { 

        Ethercat.packet_tx[0].LED = 1;
        Ethercat.packet_tx[0].LED = 1;
        Ethercat.EcatSyncMsg();

//程序主逻辑

        char key = ARX_KEYBOARD.keyPress.load();
        ARX_ARM.getKey(key);

        ARX_ARM.get_curr_pos();
        if(!ARX_ARM.is_starting){
             cmd = ARX_ARM.get_cmd();
        }
        ARX_ARM.update_real(Ethercat,cmd);

//发送关节数据

        sensor_msgs::JointState msg_joint;
        msg_joint.header.stamp = ros::Time::now();
        size_t num_joint = 7;
        msg_joint.name.resize(num_joint);
        msg_joint.velocity.resize(num_joint);
        msg_joint.position.resize(num_joint);
        msg_joint.effort.resize(num_joint);
        
        for (size_t i=0; i < 7; ++i)
        {   
            msg_joint.name[i] = "joint" + std::to_string(i);
            msg_joint.position[i] = ARX_ARM.current_pos[i];
            msg_joint.velocity[i] = ARX_ARM.current_vel[i];
            msg_joint.effort[i] = ARX_ARM.current_torque[i];
        }
        pub_current.publish(msg_joint);

        geometry_msgs::PoseStamped msg_pos_back;
        msg_pos_back.header.stamp = msg_joint.header.stamp;
        msg_pos_back.pose.position.x      =ARX_ARM.fk_end_pos[0];
        msg_pos_back.pose.position.y      =ARX_ARM.fk_end_pos[1];
        msg_pos_back.pose.position.z      =ARX_ARM.fk_end_pos[2];
        msg_pos_back.pose.orientation.x   =ARX_ARM.fk_end_pos[3];
        msg_pos_back.pose.orientation.y   =ARX_ARM.fk_end_pos[4];
        msg_pos_back.pose.orientation.z   =ARX_ARM.fk_end_pos[5];
        msg_pos_back.pose.orientation.w   =ARX_ARM.current_pos[6];   // TODO: does it need to times 12 or 5?
        pub_pos.publish(msg_pos_back);


        ros::spinOnce();
        loop_rate.sleep();
        arx_v();

        if (arx_flag2)
        {
            break;
        }

        
    }

    arx_2(CAN_Handlej,Ethercat);
    Ethercat.EcatStop();
    

    return 0;
}