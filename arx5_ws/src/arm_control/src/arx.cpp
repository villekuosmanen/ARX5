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
#include "arm_control/PosCmd.h"
#include "arm_control/JointControl.h"
#include "arm_control/JointInformation.h"
#include "arm_control/ChassisCtrl.h"
#include "arm_control/MagicCmd.h"

char phy[] = "enx207bd2d3159b"; 

int CONTROL_MODE=0; 
// 0 arx5 rc ，
// 1 5a rc ，
// 2 arx5 joint_control ，
// 3 5a joint_control   
// 4 arx5 pos_control 
// 5 5a pos_control
command cmd;

bool arx_flag2 = false;
// bool app_stopped = false;
void sigint_handler(int sig);
void safe_stop();

ecat::EcatBase Ethercat(1);
can CAN_Handlej;

int main(int argc, char **argv)
{

    ros::init(argc, argv, "arm_node"); 
    ros::NodeHandle node;
    Teleop_Use()->teleop_init(node);

    arx_arm ARX_ARM(Ethercat,(int) CONTROL_MODE);

    ros::Subscriber sub_joint = node.subscribe<arm_control::JointControl>("/joint_control", 10, 
                                  [&ARX_ARM](const arm_control::JointControl::ConstPtr& msg) {
                                      ARX_ARM.ros_control_pos_t[0] = msg->joint_pos[0];
                                      ARX_ARM.ros_control_pos_t[1] = msg->joint_pos[1];
                                      ARX_ARM.ros_control_pos_t[2] = msg->joint_pos[2];
                                      ARX_ARM.ros_control_pos_t[3] = msg->joint_pos[3];
                                      ARX_ARM.ros_control_pos_t[4] = msg->joint_pos[4];
                                      ARX_ARM.ros_control_pos_t[5] = msg->joint_pos[5];
                                  });

    ros::Subscriber sub_cmd = node.subscribe<arm_control::PosCmd>("/arx5_pos_cmd", 10, 
                                [&ARX_ARM](const arm_control::PosCmd::ConstPtr& msg) {
                                    ARX_ARM.arx5_cmd.x          = msg->x;
                                    ARX_ARM.arx5_cmd.y          = msg->y;
                                    ARX_ARM.arx5_cmd.z          = msg->z;
                                    ARX_ARM.arx5_cmd.waist_roll = msg->roll;
                                    ARX_ARM.arx5_cmd.waist_pitch  = msg->pitch;
                                    ARX_ARM.arx5_cmd.waist_yaw    = msg->yaw;
                                });

    arx5_keyboard ARX_KEYBOARD;

    ros::Rate loop_rate(200);
    std::thread keyThread(&arx5_keyboard::detectKeyPress, &ARX_KEYBOARD);
    sleep(1);

    Ethercat.EcatStart(phy);

    CAN_Handlej.Send_moto_Cmd1(Ethercat, 1, 0, 0, 0, 0, 0);CAN_Handlej.Send_moto_Cmd1(Ethercat, 1, 0, 0, 0, 0, 0);
    CAN_Handlej.Send_moto_Cmd1(Ethercat, 2, 0, 0, 0, 0, 0);CAN_Handlej.Send_moto_Cmd1(Ethercat, 2, 0, 0, 0, 0, 0);
    CAN_Handlej.Send_moto_Cmd1(Ethercat, 4, 0, 0, 0, 0, 0);CAN_Handlej.Send_moto_Cmd1(Ethercat, 4, 0, 0, 0, 0, 0);
    CAN_Handlej.Enable_Moto(Ethercat,5);CAN_Handlej.Enable_Moto(Ethercat,5);
    CAN_Handlej.Enable_Moto(Ethercat,6);CAN_Handlej.Enable_Moto(Ethercat,6);
    CAN_Handlej.Enable_Moto(Ethercat,7);CAN_Handlej.Enable_Moto(Ethercat,7);
    CAN_Handlej.Enable_Moto(Ethercat,8);CAN_Handlej.Enable_Moto(Ethercat,8);

    while(ros::ok())
    { 

        Ethercat.packet_tx[0].LED = 1;
        Ethercat.packet_tx[0].LED = 1;
        Ethercat.EcatSyncMsg();

        char key = ARX_KEYBOARD.keyPress.load();
        ARX_ARM.getKey(key);

        ARX_ARM.get_joint();
        if(!ARX_ARM.is_starting){
             cmd = ARX_ARM.get_cmd();
        }
        ARX_ARM.update_real(Ethercat,cmd);
    
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