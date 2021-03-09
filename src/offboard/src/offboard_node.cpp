#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <opencv2/opencv.hpp>

mavros_msgs::State current_state;//飞控当前状态
geometry_msgs::PoseStamped uavCurrentLocalPose;//飞机当前位姿
mavros_msgs::PositionTarget uav_vel_tar, uav_poshold_tar;//local_raw控制消息,机体和全局
double uavRollENU, uavPitchENU, uavYawENU;//飞控ENU姿态

enum UAV_STATE {
    UAV_READY,//非offboard模式
    UAV_TAKEOFF_left,//起飞升高状态-左
    UAV_TAKEOFF_right,//起飞升高状态-右
    UAV_POS_HOLD,//定点状态
    UAV_FLY_LEFT,//左飞（即Y轴正方向）
    UAV_FLY_RIGHT,//右飞（即Y轴负方向）
    UAV_LANDING //降落状态
}uav_state;//切换控制状态

double height_step = 0.4;//飞行步进高度
double takeoff_height = 0.5 * height_step;//命令下发的飞行高度，初始高度等于步进高度的一半
double pos_hold_height = takeoff_height;//定高高度
double length = 4.5;//货架长度（即左右飞行距离）
ros::Time takeoff_start_time;//开始起飞时间
ros::Time last_cmd_time;//上一次接收指令的时间
bool sim_mode = true;//仿真控制标志位
bool cmd_lost_flag = true;//连续控制指令中断标志

//以下是可选的控制标志,注意各标志位的优先关系,位置>速度>加速度
/*mavros_msgs::PositionTarget::IGNORE_PX|mavros_msgs::PositionTarget::IGNORE_PY|mavros_msgs::PositionTarget::IGNORE_PZ|
mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|mavros_msgs::PositionTarget::IGNORE_VZ|
mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|
mavros_msgs::PositionTarget::IGNORE_YAW|mavros_msgs::PositionTarget::IGNORE_YAW_RATE;*/
//定高模式(注意VZ不能忽略!!!)
#define VEL_2D_MODE  mavros_msgs::PositionTarget::IGNORE_PX|mavros_msgs::PositionTarget::IGNORE_PY|\
                     mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|\
                     mavros_msgs::PositionTarget::IGNORE_YAW

//定点模式
#define POS_HOLD_MODE mavros_msgs::PositionTarget::IGNORE_VX|mavros_msgs::PositionTarget::IGNORE_VY|mavros_msgs::PositionTarget::IGNORE_VZ|\
                      mavros_msgs::PositionTarget::IGNORE_AFX|mavros_msgs::PositionTarget::IGNORE_AFY|mavros_msgs::PositionTarget::IGNORE_AFZ|\
                      mavros_msgs::PositionTarget::IGNORE_YAW_RATE

void state_cb(const mavros_msgs::State::ConstPtr& msg)//飞控状态消息回调
{
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)//当前位姿消息回调
{
    uavCurrentLocalPose.pose = msg->pose;//更新当前位姿
    //Using ROS tf to get RPY angle from Quaternion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(uavCurrentLocalPose.pose.orientation, quat);//tf转换得到姿态
    tf::Matrix3x3(quat).getRPY(uavRollENU, uavPitchENU, uavYawENU);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;
    //飞机状态订阅
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    //local pose订阅
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 10, local_pos_cb);
    //飞机全局坐标位置发布，目前统一由setpoint_raw来替代完成
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    //飞机全局坐标速度发布，目前统一由setpoint_raw来替代完成
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_velocity/cmd_vel", 10);
    //setpoint_raw话题,根据消息设置不同,可提供全局坐标系下的控速控点,及机体坐标系下的控速
    ros::Publisher local_raw_tar_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("/mavros/setpoint_raw/local", 10);
    //px4解锁上锁服务,慎用,解锁上锁的权限最好交给遥控器
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/mavros/cmd/arming");
    //px4模式切换服务
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    uav_vel_tar.header.stamp = ros::Time::now();
    uav_vel_tar.header.frame_id = "uav";
    uav_vel_tar.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;//设置为机体坐标系
    uav_vel_tar.type_mask = VEL_2D_MODE;
    uav_vel_tar.position.x = 0;
    uav_vel_tar.position.y = 0;
    uav_vel_tar.position.z = takeoff_height;

    uav_vel_tar.velocity.x = 0;
    uav_vel_tar.velocity.y = 0;
    uav_vel_tar.velocity.z = 0;

    uav_vel_tar.acceleration_or_force.x = 0;
    uav_vel_tar.acceleration_or_force.y = 0;
    uav_vel_tar.acceleration_or_force.z = 0;

    uav_vel_tar.yaw = 0;
    uav_vel_tar.yaw_rate = 0;

    uav_poshold_tar.header.stamp = ros::Time::now();
    uav_poshold_tar.header.frame_id = "uav";
    uav_poshold_tar.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;//设置为全局坐标系
    uav_poshold_tar.type_mask = POS_HOLD_MODE;
    uav_poshold_tar.position.x = 0;
    uav_poshold_tar.position.y = 0;
    uav_poshold_tar.position.z = takeoff_height;
    uav_poshold_tar.yaw = 0;
    uav_poshold_tar.yaw_rate = 0;
    std::cout << "初始设置完毕" << std::endl;

    uav_state = UAV_READY;//初始化飞控状态
    mavros_msgs::SetMode offb_set_mode;//设置要切换的模式,此处用于自动降落
    //offb_set_mode.request.custom_mode = "AUTO.LAND";
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;//电机解锁命令
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ROS_INFO("px4_control_node is up!");
    if(sim_mode)
    {
        cv::namedWindow("Key");
        ROS_INFO("node run in sim mode!");
    }

    while(ros::ok())
    {
        if(current_state.mode != "OFFBOARD" || !current_state.armed)//不在offboard模式或未解锁，则切换到准备状态
        {
            uav_state = UAV_READY;
        }
        std::cout << "last_cmd_time=" << last_cmd_time << std::endl;
        if((uav_state == UAV_FLY_LEFT || uav_state == UAV_FLY_RIGHT)
            && (ros::Time::now() - last_cmd_time > ros::Duration(0.5)))//飞行状态时若0.5s内未接收到指令，则原地定点
        {
            cmd_lost_flag = false;
            uav_state = UAV_POS_HOLD;
            pos_hold_height = takeoff_height;
            std::cout << "Enter in UAV_POS_HOLD mode! pos_hold_height = " << pos_hold_height << std::endl;
            uav_poshold_tar.header.stamp = ros::Time::now();
            uav_poshold_tar.position.x = uavCurrentLocalPose.pose.position.x;
            uav_poshold_tar.position.y = uavCurrentLocalPose.pose.position.y;
            uav_poshold_tar.position.z = pos_hold_height;
            uav_poshold_tar.yaw = uavYawENU;
        }
        switch (uav_state)
        {
            case UAV_READY://准备状态
            {
                std::cout << "Enter in UAV_READY" << std::endl;
                takeoff_height = 0.5 * height_step;
                uav_poshold_tar.header.stamp = ros::Time::now();
                uav_poshold_tar.position.x = 0;
                uav_poshold_tar.position.y = 0;
                uav_poshold_tar.position.z = takeoff_height;
                uav_poshold_tar.yaw = 1.57;
                local_raw_tar_pub.publish(uav_poshold_tar);
                if(current_state.mode == "OFFBOARD" && current_state.armed)//切offboard转入起飞状态
                {
                    std::cout << "********************************************************8" << std::endl;
                    //takeoff_start_time = ros::Time::now();
                    uav_poshold_tar.header.stamp = ros::Time::now();
                    std::cout << "reday-z=" << uavCurrentLocalPose.pose.position.z << std::endl;
                    std::cout << "********************************************************8" << std::endl;
                    if((uavCurrentLocalPose.pose.position.z > takeoff_height-0.05)&&//到达起飞高度且保证至少5s的起飞时间
                        (uavCurrentLocalPose.pose.position.z < height_step)&&
                        (uavCurrentLocalPose.pose.position.x > 0.0-0.05 ))
                    {
                        uav_state = UAV_POS_HOLD;//转入定点模式
                        std::cout << "Enter in UAV_POS_HOLD mode!!!" << std::endl;
                    }
                }
                break;
            }

            case UAV_POS_HOLD://定点模式
            {
                std::cout << "Enter in UAV_POS_HOLD*************************" << std::endl;
                uav_poshold_tar.header.stamp = ros::Time::now();
                uav_poshold_tar.position.x = uavCurrentLocalPose.pose.position.x;
                uav_poshold_tar.position.y = uavCurrentLocalPose.pose.position.y;
                uav_poshold_tar.position.z = takeoff_height;
                uav_poshold_tar.yaw = 1.57;
                local_raw_tar_pub.publish(uav_poshold_tar);

                std::cout << "********************************************************8" << std::endl;
                std::cout << "poshold-z =" << uavCurrentLocalPose.pose.position.z << std::endl;
                std::cout << "poshold-x =" << uavCurrentLocalPose.pose.position.x << std::endl;
                std::cout << "********************************************************8" << std::endl;
                if((uavCurrentLocalPose.pose.position.z > 0.5*height_step-0.05)&& //takeoff_height=0.25
                    (uavCurrentLocalPose.pose.position.z < height_step)&&
                    (uavCurrentLocalPose.pose.position.x > 0.0-0.05 ))
                {
                    uav_state = UAV_FLY_LEFT;
                    last_cmd_time = ros::Time::now();
                    std::cout << "Enter in UAV_FLY_LEFT mode!" << std::endl;
                }
                else if((uavCurrentLocalPose.pose.position.z > 0.5*height_step-0.05)&& //takeoff_height=0.25
                        (uavCurrentLocalPose.pose.position.z < height_step)&&
                        (uavCurrentLocalPose.pose.position.x < -length+0.05))
                {
                    uav_state = UAV_TAKEOFF_left;
                    last_cmd_time = ros::Time::now();
                    std::cout << "Enter in UAV_TAKEOFF_left mode!" << std::endl;
                }
                else if((uavCurrentLocalPose.pose.position.z > 1.5*height_step-0.05)&&//takeoff_height=0.75
                        (uavCurrentLocalPose.pose.position.z < 2*height_step)&&
                        (uavCurrentLocalPose.pose.position.x < -length+0.05))
                {
                    uav_state = UAV_FLY_RIGHT;
                    last_cmd_time = ros::Time::now();
                    std::cout << "Enter in UAV_FLY_RIGHT mode!" << std::endl;
                }
                else if((uavCurrentLocalPose.pose.position.z > 1.5*height_step-0.05)&&//takeoff_height=0.75
                        (uavCurrentLocalPose.pose.position.z < 2*height_step)&&
                        (uavCurrentLocalPose.pose.position.x > 0.0-0.05))
                {
                    uav_state = UAV_TAKEOFF_right;
                    last_cmd_time = ros::Time::now();
                    std::cout << "Enter in UAV_TAKEOFF mode!" << std::endl;
                }
                else if((uavCurrentLocalPose.pose.position.z > 2.5*height_step-0.05)&&//takeoff_height=1.25
                        (uavCurrentLocalPose.pose.position.x > 0.0-0.05))
                {
                    uav_state = UAV_FLY_LEFT;
                    last_cmd_time = ros::Time::now();
                    std::cout << "Enter in UAV_FLY_LEFT mode!" << std::endl;
                }
                else if((uavCurrentLocalPose.pose.position.z > 2.5*height_step-0.05)&&//takeoff_height=1.25
                        (uavCurrentLocalPose.pose.position.x < -length+0.05))
                {
                    uav_state = UAV_LANDING;
                    last_cmd_time = ros::Time::now();
                    std::cout << "Enter in UAV_LANDING mode!" << std::endl;
                }
                else
                {
                    uav_state = UAV_LANDING;
                    std::cout << "Enter in UAV_LANDING mode!" << std::endl;
                }
                break;
            }

            case UAV_FLY_LEFT://左飞模式
            {
                std::cout << "enter in left********************************" << std::endl;
                std::cout << "left-takeoff_height = " << takeoff_height << std::endl;
                uav_vel_tar.header.stamp = ros::Time::now();
                pos_hold_height = takeoff_height;
                uav_vel_tar.position.z = pos_hold_height;
                uav_vel_tar.velocity.x = 0;
                uav_vel_tar.velocity.y = 0.5;
                uav_vel_tar.velocity.z = 0;
                uav_vel_tar.yaw = 1.57;
                uav_vel_tar.yaw_rate = 0;
                local_raw_tar_pub.publish(uav_vel_tar);
                last_cmd_time = ros::Time::now();
                std::cout << "left-x = " << uavCurrentLocalPose.pose.position.x << std::endl;
                if(uavCurrentLocalPose.pose.position.x < -length+0.05)
                {
                    uav_state = UAV_POS_HOLD;
                    std::cout << "Enter in UAV_POS_HOLD mode!!!" << std::endl;
                }
                break;
            }

            case UAV_TAKEOFF_left://起飞升高状态-左
            {
                std::cout << "enter in takeoff-left*************************" <<std::endl;
                uav_poshold_tar.header.stamp = ros::Time::now();
                takeoff_height = 1.5*height_step;
                std::cout << "left-takeoff_height = " << takeoff_height << std::endl;
                uav_poshold_tar.position.x = -length;
                uav_poshold_tar.position.y = 0;
                uav_poshold_tar.position.z = takeoff_height;
                uav_poshold_tar.yaw = 1.57;
                local_raw_tar_pub.publish(uav_poshold_tar);
                last_cmd_time = ros::Time::now();
                std::cout << "takeoff-left-z = " << uavCurrentLocalPose.pose.position.z << std::endl;
                std::cout << "******************************************" << std::endl;
                if(uavCurrentLocalPose.pose.position.z > takeoff_height-0.05)
                {
                    uav_state = UAV_POS_HOLD;
                    std::cout << "Enter in UAV_POS_HOLD mode!!!" << std::endl;
                }
                break;
            }

            case UAV_FLY_RIGHT://右飞模式
            {
                std::cout << "enter in right********************************" << std::endl;
                std::cout << "right-takeoff_height = " << takeoff_height << std::endl;
                uav_vel_tar.header.stamp = ros::Time::now();
                pos_hold_height = takeoff_height;
                uav_vel_tar.position.z = pos_hold_height;
                uav_vel_tar.velocity.x = 0;
                uav_vel_tar.velocity.y = -0.5;
                uav_vel_tar.velocity.z = 0;
                uav_vel_tar.yaw = 1.57;
                uav_vel_tar.yaw_rate = 0;
                local_raw_tar_pub.publish(uav_vel_tar);
                last_cmd_time = ros::Time::now();
                if(uavCurrentLocalPose.pose.position.x > 0.0-0.05)
                {
                    uav_state = UAV_POS_HOLD;
                    std::cout << "Enter in UAV_POS_HOLD mode!!!" << std::endl;
                }
                break;
            }

            case UAV_TAKEOFF_right://起飞升高状态-右
            {
                std::cout << "enter in takeoff-right*************************" <<std::endl;
                uav_poshold_tar.header.stamp = ros::Time::now();
                takeoff_height = 2.5*height_step;
                std::cout << "right-takeoff_height = " << takeoff_height << std::endl;
                uav_poshold_tar.position.x = 0;
                uav_poshold_tar.position.y = 0;
                uav_poshold_tar.position.z = takeoff_height;
                uav_poshold_tar.yaw = 1.57;
                local_raw_tar_pub.publish(uav_poshold_tar);
                last_cmd_time = ros::Time::now();
                std::cout << "takeoff-right-z = " << uavCurrentLocalPose.pose.position.z << std::endl;
                std::cout << "******************************************" << std::endl;
                if(uavCurrentLocalPose.pose.position.z > takeoff_height-0.05)
                {
                    uav_state = UAV_POS_HOLD;
                    std::cout << "Enter in UAV_POS_HOLD mode!!!" << std::endl;
                }
                break;
            }

            case UAV_LANDING:
            {
                if(current_state.mode != "AUTO.LAND")
                {
                    offb_set_mode.request.custom_mode = "AUTO.LAND";
                    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    {
                        ROS_INFO("AUTO.LAND Enabled");
                    }
                }
                break;
            }

            default:break;
        }

        if(sim_mode)
        {
            char key = cv::waitKey(5);
            switch (key)
            {
                case 'q':
                {
                    if( current_state.mode != "OFFBOARD" )
                    {
                        std::cout<<"not in offboard!!!please press again to confirm takeoff"<<std::endl;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                            ROS_INFO("OFFBOARD enabled");
                        }
                        break;
                    }
                    if(current_state.armed == false)
                    {
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                        {
                            ROS_INFO("Vehicle armed");
                        }
                        break;
                    }
                    takeoff_height = 0.5 * height_step;
                    uav_poshold_tar.header.stamp = ros::Time::now();
                    uav_poshold_tar.position.x = uavCurrentLocalPose.pose.position.x;
                    uav_poshold_tar.position.y = uavCurrentLocalPose.pose.position.y;
                    uav_poshold_tar.position.z = takeoff_height;
                    uav_poshold_tar.yaw = 1.57;
                    local_raw_tar_pub.publish(uav_poshold_tar);
                    takeoff_start_time = ros::Time::now();
                    uav_state = UAV_READY;
                    std::cout << "auto takeoff start" << std::endl;
                    break;
                }
                case 's':
                {
                    if(current_state.armed == false)
                    {
                        std::cout << "not armed!!!" << std::endl;
                        break;
                    }
                    uav_state = UAV_LANDING;
                    std::cout << "auto landing start" << std::endl;
                    break;
                }
                default:break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

