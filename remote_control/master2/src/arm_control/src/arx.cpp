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
// #include "arm_control/JointInformation.h"

#include <sensor_msgs/JointState.h>
#include "geometry_msgs/PoseStamped.h"
 
char phy[] = "enx5c5310ecc0ec"; 

int CONTROL_MODE=0;

bool arx_flag2 = false;
void sigint_handler(int sig);
void safe_stop();

ecat::EcatBase Ethercat(1);
can CAN_Handlej;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm3"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);

    arx_arm ARX_ARM((int) CONTROL_MODE);

    // this was changed to match the Cobot Magic publishers
    ros::Publisher pub_current = node.advertise<sensor_msgs::JointState>("/master/joint_left", 10);
    // ros::Publisher pub_pos = node.advertise<geometry_msgs::PoseStamped>("/master/end_left", 10);

    // TODO: unclear what this subscriber does and what it is needed for
    // ros::Subscriber sub_information = node.subscribe<arm_control::JointInformation>("/joint_information2", 10, 
    //                               [&ARX_ARM](const arm_control::JointInformation::ConstPtr& msg) {
    //                                   ARX_ARM.ros_control_cur_t[0] = msg->joint_cur[0];
    //                                   ARX_ARM.ros_control_cur_t[1] = msg->joint_cur[1];
    //                                   ARX_ARM.ros_control_cur_t[2] = msg->joint_cur[2];
    //                                   ARX_ARM.ros_control_cur_t[3] = msg->joint_cur[3];
    //                                   ARX_ARM.ros_control_cur_t[4] = msg->joint_cur[4];
    //                                   ARX_ARM.ros_control_cur_t[5] = msg->joint_cur[5];
    //                                   ARX_ARM.ros_control_cur_t[6] = msg->joint_cur[6];
    //                                   ARX_ARM.ros_control_pos_t[6] = msg->joint_pos[6];
    //                               });

    ros::Rate loop_rate(200);
    sleep(1);
    Ethercat.EcatStart(phy);
    // CAN_Handlej.Send_moto_Cmd1(Ethercat, 1, 0, 0, 0, 0, 0);
    // CAN_Handlej.Send_moto_Cmd1(Ethercat, 2, 0, 0, 0, 0, 0);
    // CAN_Handlej.Send_moto_Cmd1(Ethercat, 4, 0, 0, 0, 0, 0);
    CAN_Handlej.Enable_Moto(Ethercat,1);
    CAN_Handlej.Enable_Moto(Ethercat,2);
    // CAN_Handlej.Enable_Moto(Ethercat,3);
    CAN_Handlej.Enable_Moto(Ethercat,4);
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

        ARX_ARM.get_curr_pos();
        ARX_ARM.motor_control(Ethercat);
        // if(!ARX_ARM.is_starting){
        //      cmd = ARX_ARM.get_cmd();
        // }
        // ARX_ARM.update_real(Ethercat,cmd);
        
        // This was changed to match the Cobot Magic JointState publish code
        sensor_msgs::JointState msg_joint;
        msg_joint.header.stamp = ros::Time::now();
        // msg_joint.header.frame_id = "map";
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
            if (i == 6) msg_joint.position[i] *=5;    // 映射放大 // No idea why this is needed (differs from L5) but we'll keep it
            std::cout << "Joint " << i << ", Position = " << msg_joint.position[i]
                        << ", Velocity = " << msg_joint.velocity[i]
                        << ", Effort = " << msg_joint.effort[i] << std::endl;
        }
        pub_current.publish(msg_joint);
        


        // This was edited to match the Cobot Magic version
        // geometry_msgs::PoseStamped msg_pos_back;
        // msg_pos_back.header.stamp = msg_joint.header.stamp
        // msg_pos_back.pose.position.x      =ARX_ARM.fk_end_pos[0];
        // msg_pos_back.pose.position.y      =ARX_ARM.fk_end_pos[1];
        // msg_pos_back.pose.position.z      =ARX_ARM.fk_end_pos[2];
        // msg_pos_back.pose.orientation.x   =ARX_ARM.fk_end_pos[3];
        // msg_pos_back.pose.orientation.y   =ARX_ARM.fk_end_pos[4];
        // msg_pos_back.pose.orientation.z   =ARX_ARM.fk_end_pos[5];
        // msg_pos_back.pose.orientation.w   =ARX_ARM.current_pos[6];  // TODO: does it need to times 12 or 5?
        // pub_pos.publish(msg_pos_back);

        ros::spinOnce();
        loop_rate.sleep();

        arx_v();    // TODO: is this needed?

        if (arx_flag2)
        {
            break;
        }

        
    }

    arx_2(CAN_Handlej,Ethercat);
    Ethercat.EcatStop();
    

    return 0;
}