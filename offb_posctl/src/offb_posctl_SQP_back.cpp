#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <chrono>
#include <iomanip>
#include <thread>
#include <unistd.h>
#include <mutex>
#include <Eigen/Eigen>
#include <Eigen/Geometry> 
#include <Eigen/Core>

#include <ros/ros.h>
#include "Parameter.h"
#include "PID.h"

//topic
#include "offb_posctl/controlstate.h"  // 自定义消息
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

using namespace Eigen;
using namespace std;

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>全 局 变 量<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

mavros_msgs::State current_state;           //无人机当前状态
nav_msgs::Odometry pose_drone_odom;       //读入的无人机drone当前位置，x，y，z+姿态
sensor_msgs::Imu pose_drone_Imu;       //读入的无人机drone当前位置，x，y，z+姿态
nav_msgs::Odometry pose_car_odom;       //读入car当前位置
geometry_msgs::TwistStamped vel_drone;      //读入的无人机当前速度 线速度+角速度
geometry_msgs::Quaternion orientation_target;   //发给无人机的姿态指令  四元数
geometry_msgs::Vector3 angle_target;   //欧拉角
geometry_msgs::Vector3 vel_target;   //期望速度
geometry_msgs::Point plane_expected_position; //车的零点和飞机的零点差3m，根据车的当前位置计算飞机位置
geometry_msgs::PoseStamped target_attitude;  //1
mavros_msgs::Thrust target_thrust_msg; //1循环
std_msgs::Float64 plane_real_alt; //control前
mavros_msgs::AttitudeTarget target_atti_thrust_msg; //最终发布的消息 油门+角度
mavros_msgs::AttitudeTarget base_atti_thrust_msg; //最终发布的消息 油门+角度
nav_msgs::Odometry  planned_postwist_msg;
nav_msgs::Odometry planned_u_msg;
nav_msgs::Odometry current_relativepostwist_msg;
geometry_msgs::Vector3 targeterror_msg;
geometry_msgs::Vector3 temp_angle;
geometry_msgs::Vector3 rpy;
offb_posctl::controlstate controlstatearray;
offb_posctl::controlstate temp_controlstatearray;
geometry_msgs::Vector3 begin_flag_temp;
geometry_msgs::Twist cmd_vel;
geometry_msgs::TwistStamped plane_cmd_twist;
std_msgs::Float32 time_cost; // the calculation time of GPM or BVP


bool begin_flag_bool = false;
float thrust_target;        //期望推力
float Yaw_Init;
float Yaw_Locked = 0;           //锁定的偏航角(一般锁定为0)
bool got_initial_point = false;
PID PIDVX, PIDVY, PIDVZ;    //声明PID类
Parameter param;
std::ofstream logfile;

///for bvp
float px_ini = -3;
float pz_ini = 0;
float py_ini = 0;
float vx_ini = -0.1;
float vz_ini = 0.0;
float vy_ini = 0;
float z0 = 0.1;
float t_end = 2;
double thrustforceacc = 0.0;
int pointnumber = 150;// the number is almost always 20. It less, the accuracy won't be enough, if more, the time consumpiton will be too large.
int controlfreq = 50;//50;
int discretizedpointpersecond = (int)pointnumber/t_end;
int controlcounter = 0;

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//欧拉角转四元数
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);
//geometry_msgs的Quaternion类型的函数
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);

float get_ros_time(ros::Time time_begin);
float get_time();
int pix_controller(float cur_time);
void vector3dLimit(Vector3d &v, double limit) ; ///limit should be positive
Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2);
void tractor_controller(float time);
//int pix_controller(int cur_time);
void data_log(std::ofstream &logfile, float cur_time, float ros_time);

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}//当有消息到达topic时会自动调用一次
bool planeupdateflag= false;
void plane_pos_cb(const nav_msgs::Odometry::ConstPtr &msg){
    pose_drone_odom = *msg;//pose_drone_odom是nav_msgs::Odometry类型
    planeupdateflag= true;
}
void plane_imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    pose_drone_Imu = *msg;//pose_drone_odom是nav_msgs::Odometry类型
}

void plane_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}
void car_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) { // 根据车的位置计算飞机位置
    pose_car_odom = *msg;
    plane_expected_position.x = pose_car_odom.pose.pose.position.x; //-1 means axis difference
    plane_expected_position.y = pose_car_odom.pose.pose.position.y; //-1 means axis difference
    plane_expected_position.z = pose_car_odom.pose.pose.position.z + z0;
}
void plane_alt_cb(const std_msgs::Float64::ConstPtr &msg){
    plane_real_alt = *msg;
}
bool contstaterecieveflag= false;
void controlstate_cb(const offb_posctl::controlstate::ConstPtr &msg)
{
    controlstatearray = *msg;
    temp_controlstatearray = controlstatearray;
    controlcounter = controlstatearray.inicounter; // controlstatearray消息中的inicounter
    // 设置discretizedpointpersecond
    if(contstaterecieveflag == false)//第一次回调时初始化，之后这个flag一直是true
    {
        contstaterecieveflag= true;
        discretizedpointpersecond=controlstatearray.discrepointpersecond; //=(int)pointnumber/t_end
        cout<<"contstaterecieveflag"<<contstaterecieveflag<<endl;// 输出该定值以标记
    }
}

void begin_run_cb(const geometry_msgs::Vector3::ConstPtr &msg){
    begin_flag_temp = *msg;
    // x方向大于3时标记begin_flag_bool为true
    if (begin_flag_temp.x > 3.0)
    {
        begin_flag_bool = true;
    }
}

void time_cost_cb(const std_msgs::Float32::ConstPtr &msg){
    time_cost = *msg;
}

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)//argc  argument count 传参个数，argument value
{
    ros::init(argc, argv, "position_control");//初始化节点名称
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming"); //使能解锁飞机  创建client对象并向arming发出请求，服务类型为CommandBool
    ros::ServiceClient setmode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode"); //设置为自动控制模式 服务类型为SetMode
    
    // 【订阅】无人机当前状态/位置/速度信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);//订阅器，订阅接收mavros/state话题的mavros_msgs::State类型的消息，有消息到达这个话题时会自动调用state_cb函数
    ros::Subscriber plane_position_pose_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, plane_pos_cb);//pos+twist
    ros::Subscriber plane_poseimu_sub = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, plane_imu_cb);//pos+twist
    ros::Subscriber plane_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, plane_vel_cb); //twist
    ros::Subscriber car_position_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, car_pos_cb); //车的pos+twist
    ros::Subscriber plane_alt_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1, plane_alt_cb);
    ros::Subscriber controlstate_sub = nh.subscribe<offb_posctl::controlstate>("SQP_controlstate", 1, controlstate_cb);
    ros::Subscriber begin_run_sub = nh.subscribe<geometry_msgs::Vector3>("begin_run", 1, begin_run_cb);
    ros::Subscriber time_cost = nh.subscribe<std_msgs::Float32>("time_cost", 1, time_cost_cb);

    // 【发布】飞机姿态/拉力信息 坐标系:NED系
    ros::Publisher ocplan_postwist_pub = nh.advertise<nav_msgs::Odometry>("ocplan_positiontwist", 1);//bvp计算的期望位置
    ros::Publisher ocplan_u_pub = nh.advertise<nav_msgs::Odometry>("ocplan_u", 1);//bvp计算的期望控制量(,没有转化的)
    ros::Publisher target_atti_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);//发布给mavros的控制量(经过换算)
    ros::Publisher target_vel_cmd_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel", 1);//发布给mavros的控制量(经过换算)
    ros::Publisher plane_rpy_pub = nh.advertise<geometry_msgs::Vector3>("drone/current_rpy", 1);//飞机当前的rpy角
    ros::Publisher current_relativepostwist_pub = nh.advertise<nav_msgs::Odometry>("current_relative_postwist", 1);//当前状态方程中的状态量,即相对量
    ros::Publisher targeterror_pub = nh.advertise<geometry_msgs::Vector3>("targeterror", 1);//目标误差
    ros::Rate rate(controlfreq);   //50hz的频率发送/接收topic  ros与pixhawk之间,50Hz control frequency

    // log输出文件初始化
    logfile.open("/home/chenwh/MATLAB-Drive/CIUS/exp/case.csv", std::ios::out);
    if (!logfile.is_open()) {
        ROS_ERROR("log to file error!");
        return 0;
    }

    // 读取PID参数
    std::string paraadr("/home/chenwh/exp_ws/src/offb_posctl/src/param");
    if (param.readParam(paraadr.c_str()) == 0) {
        std::cout << "read config file error!" << std::endl;
        return 0;
    }

    /// 设置速度环PID参数 比例参数 积分参数 微分参数
    PIDVX.setPID(param.vx_p, param.vx_i, param.vx_d);
    PIDVY.setPID(param.vy_p, param.vy_i, param.vy_d);
    PIDVZ.setPID(param.vz_p, param.vz_i, param.vz_d);
    // 设置速度环积分上限 控制量最大值 误差死区
    PIDVX.set_sat(6, 5, 0);
    PIDVY.set_sat(2, 3, 0);
    PIDVZ.set_sat(2, 5, 0);

    /// 等待和飞控的连接
    while (ros::ok() && current_state.connected == 0) {

        ros::spinOnce(); //调用回调函数
        ros::Duration(1).sleep();
        ROS_INFO("Not Connected");
    }
    ROS_INFO("Connected!!");

    target_atti_thrust_msg.orientation.x = 0;
    target_atti_thrust_msg.orientation.y = 0;
    target_atti_thrust_msg.orientation.z = 0;
    target_atti_thrust_msg.orientation.w = -1;
    target_atti_thrust_msg.thrust = 0.65; //65%的油门 50%与重力平衡即悬停

    /// get car current pose to set plane pose
    float x = pose_car_odom.pose.pose.orientation.x;
    float y = pose_car_odom.pose.pose.orientation.y;
    float z = pose_car_odom.pose.pose.orientation.z;
    float w = pose_car_odom.pose.pose.orientation.w;
    Yaw_Init = quaternion2euler(x, y, z, w).z;

    ///set initial hover position and pose
    target_atti_thrust_msg.orientation.x = x;
    target_atti_thrust_msg.orientation.y = y;
    target_atti_thrust_msg.orientation.z = z;
    target_atti_thrust_msg.orientation.w = w;
    ROS_INFO("got initial point ");

    for (int i = 10; ros::ok() && i > 0; --i)// let drone take off slightly at begining, but this step seems useless because the drono has not been armed
    {
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        ros::spinOnce();//让回调函数有机会被执行
        rate.sleep();
    }
    ROS_INFO("OUT OF LOOP WAIT");

    /// change mode to arm ,then offboard 发出请求
    mavros_msgs::CommandBool arm_cmd; //解锁
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    ///解锁飞机
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (setmode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        if (plane_real_alt.data > 0.3) {
            ROS_INFO("plane takeoff !");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }


    /// reach initial hover position and pose by position control
    float error_position_sum = 0;
    float error_pose_sum = 5;
    float yaw_current;
    ros::Time begin_time_01 = ros::Time::now();
    float thrust_target_sum = 0;
    vector<float> orientation_x;
    vector<float> orientation_y;
    vector<float> orientation_z;
    vector<float> orientation_w;
    got_initial_point = true;
    base_atti_thrust_msg.thrust = 0.57;
    int echo_count = 0;

    while (ros::ok() && (!begin_flag_bool || pose_car_odom.twist.twist.linear.x > 0.2 )) {
		cout << "------------Adjusting Loop----------------" << endl;
        ros::spinOnce();
        cout << "The waffle pi velocity is:" << pose_car_odom.twist.twist.linear.x << endl;
        cout << "The waffle pi pos is:" << pose_car_odom.pose.pose.position.x << endl;
        cout << "plane_vel.x:" << vel_drone.twist.linear.x << endl;
        cout << "plane_pos.x:" << pose_drone_odom.pose.pose.position.x << endl;
        if (planeupdateflag)   //订阅到飞机位置则flag为true，发布完相对位置的消息后flag置false
        {
            px_ini = pose_drone_odom.pose.pose.position.x - pose_car_odom.pose.pose.position.x - 3;
            pz_ini = pose_drone_odom.pose.pose.position.z - (pose_car_odom.pose.pose.position.z + z0);
            py_ini = pose_drone_odom.pose.pose.position.y - pose_car_odom.pose.pose.position.y;
            vx_ini = vel_drone.twist.linear.x - pose_car_odom.twist.twist.linear.x;
            vz_ini = vel_drone.twist.linear.z - pose_car_odom.twist.twist.linear.z;
            vy_ini = vel_drone.twist.linear.y - pose_car_odom.twist.twist.linear.y;

            std::cout << "px_ini:  " << px_ini << "pz_ini:  " << pz_ini << "vx_ini:  " << vx_ini << "vz_ini:  " << vz_ini << "va_ini:" << vel_drone.twist.linear.x << std::endl;
            current_relativepostwist_msg.pose.pose.position.x = px_ini;
            current_relativepostwist_msg.pose.pose.position.z = pz_ini;
            current_relativepostwist_msg.pose.pose.position.y = py_ini;
            current_relativepostwist_msg.twist.twist.linear.x = vx_ini;
            current_relativepostwist_msg.twist.twist.linear.y = vy_ini;
            current_relativepostwist_msg.twist.twist.linear.z = vz_ini;
            current_relativepostwist_msg.header.stamp = pose_drone_odom.header.stamp;
            current_relativepostwist_pub.publish(current_relativepostwist_msg);
        }

        float cur_time_01 = get_ros_time(begin_time_01);  // 相对时间

        plane_expected_position.x = 0;
        plane_expected_position.y = 0;
        plane_expected_position.z = pose_car_odom.pose.pose.position.z + z0;

        pix_controller(cur_time_01);
        target_atti_thrust_msg.header.stamp.sec = pose_car_odom.header.stamp.sec;
        target_atti_thrust_msg.header.stamp.nsec = pose_car_odom.header.stamp.nsec;
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;


        temp_angle = quaternion2euler(pose_drone_odom.pose.pose.orientation.x, pose_drone_odom.pose.pose.orientation.y, pose_drone_odom.pose.pose.orientation.z, pose_drone_odom.pose.pose.orientation.w);//欧拉角
        rpy.y = temp_angle.y;
        rpy.x = temp_angle.x;
        rpy.z = temp_angle.z;
        plane_rpy_pub.publish(rpy);

        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        rate.sleep();//休息

    }

    ROS_INFO("reached initial point and pose ");


    ros::Time begin_time_02 = ros::Time::now();

    int lefnodeindex = 0;
    Yaw_Init = -0.05;
///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    while (ros::ok() && begin_flag_bool) {

        //cout << "----------------Numerical control Loop-------------------" << endl;
        //cout << "The waffle pi velocity is:" << pose_car_odom.twist.twist.linear.x << endl;
        ros::spinOnce();//刷新callback的消息

        /// publish current_relativepostwist_msg
        if (planeupdateflag)   //订阅到飞机位置则flag为true，发布完相对位置的消息后flag置false
        {
            px_ini = pose_drone_odom.pose.pose.position.x - pose_car_odom.pose.pose.position.x - 3;
            pz_ini = pose_drone_odom.pose.pose.position.z - (pose_car_odom.pose.pose.position.z + z0);
            py_ini = pose_drone_odom.pose.pose.position.y - pose_car_odom.pose.pose.position.y;
            vx_ini = vel_drone.twist.linear.x - pose_car_odom.twist.twist.linear.x;
            vz_ini = vel_drone.twist.linear.z - pose_car_odom.twist.twist.linear.z;
            vy_ini = vel_drone.twist.linear.y - pose_car_odom.twist.twist.linear.y;
            
            ++echo_count;
            if (echo_count > 20){
                //std::cout << "px_ini:  " << px_ini << "pz_ini:  " << pz_ini << "vx_ini:  " << vx_ini << "vz_ini:  " << vz_ini << "va_ini:" << vel_drone.twist.linear.x << std::endl;
                cout << "----------------Numerical control Loop-------------------" << endl;
                cout << "The waffle pi velocity is:" << pose_car_odom.twist.twist.linear.x << endl;
                std::cout << "px_ini:  " << px_ini << "pz_ini:  " << pz_ini << "vx_ini:  " << vx_ini << std::endl;
                std::cout << "error_x:  " << px_ini + 3 << "error_z:  " << pz_ini + px_ini * rpy.y << "thrust_target: " << thrust_target << std::endl;
                echo_count -= 20;
            }
            current_relativepostwist_msg.pose.pose.position.x = px_ini;
            current_relativepostwist_msg.pose.pose.position.z = pz_ini;
            current_relativepostwist_msg.pose.pose.position.y = py_ini;
            current_relativepostwist_msg.twist.twist.linear.x = vx_ini;
            current_relativepostwist_msg.twist.twist.linear.y = vy_ini;
            current_relativepostwist_msg.twist.twist.linear.z = vz_ini;
            current_relativepostwist_msg.header.stamp = pose_drone_odom.header.stamp;
            current_relativepostwist_pub.publish(current_relativepostwist_msg);
//            planeupdateflag = false;
        }

        /// publish current_relativepostwist_msg


        float cur_time_02 = get_ros_time(begin_time_02);  // 当前时间 → delta(e)
        float rostime = get_time();
        pix_controller(cur_time_02);                   //备用控制程序

        /// calculate orientation_target & thrust_target & planned_postwist_msg in ocp or tractor
        if (contstaterecieveflag)  //订阅到bvp计算的控制量则flag为true,用于起始时刻,还没算出bvp时
        {
            lefnodeindex = controlcounter;
            if (lefnodeindex + 1 < temp_controlstatearray.arraylength) {
                ///compensation
                angle_target.y = temp_controlstatearray.thetaarray[lefnodeindex];
                thrustforceacc = temp_controlstatearray.thrustarray[lefnodeindex];

                orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
                thrust_target = (float) (base_atti_thrust_msg.thrust) * thrustforceacc / 9.8;
                planned_u_msg.pose.pose.position.x = thrustforceacc; //after restrict & compensate
                planned_u_msg.pose.pose.position.y = angle_target.y;
                planned_u_msg.twist.twist.linear.x = thrustforceacc;
                planned_u_msg.twist.twist.linear.z = angle_target.y;
                ocplan_u_pub.publish(planned_u_msg);


                planned_postwist_msg.pose.pose.position.x = temp_controlstatearray.stateXarray[lefnodeindex];
                planned_postwist_msg.pose.pose.position.z = temp_controlstatearray.stateZarray[lefnodeindex];
                planned_postwist_msg.twist.twist.linear.x = temp_controlstatearray.stateVXarray[lefnodeindex];
                planned_postwist_msg.twist.twist.linear.z = temp_controlstatearray.stateVZarray[lefnodeindex];

            }
        }
        /// ---------------------------------------------------------------------------------------------------///


        ///publish plane current rpy
        temp_angle = quaternion2euler(pose_drone_odom.pose.pose.orientation.x,
                                      pose_drone_odom.pose.pose.orientation.y,
                                      pose_drone_odom.pose.pose.orientation.z,
                                      pose_drone_odom.pose.pose.orientation.w);//欧拉角
        rpy.y = temp_angle.y;
        rpy.x = temp_angle.x;
        rpy.z = temp_angle.z;
        plane_rpy_pub.publish(rpy);

        ///publish thrust & orientation
        //std::cout << "thrust_target: " << thrust_target << std::endl;
        //std::cout << "ocp_pitch: " << angle_target.y << std::endl;
        target_atti_thrust_msg.header.stamp = ros::Time::now();
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;
        target_atti_thrust_pub.publish(target_atti_thrust_msg);

        ///publish planned pos&twist in x&z
        planned_postwist_msg.header.stamp = ros::Time::now();
        ocplan_postwist_pub.publish(planned_postwist_msg);

        ///publish planned thrust & pitch
        ocplan_u_pub.publish(planned_u_msg);

        ///publish targeterror_msg
        targeterror_msg.x = px_ini + 3;
        targeterror_msg.z = pz_ini + px_ini * rpy.y;
        targeterror_msg.y = py_ini;
        targeterror_pub.publish(targeterror_msg);
//            if (contstaterecieveflag)  //订阅到bvp计算的控制量则flag为true,用于起始时刻,还没算出bvp时
//            {
        data_log(logfile, cur_time_02, rostime); //保存数据
//            }

        rate.sleep();


        if (lefnodeindex + 1 < temp_controlstatearray.arraylength) //用一次ocp的控制量，counter+1，直到重新订阅到ocp
        {
            controlcounter = (controlcounter + 1);
        }
        //cout << "controlcounter: " << controlcounter << endl;
    }
        logfile.close();
        return 0;

}

/**
 * 获取当前时间 单位：秒
 */
float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec-time_begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - time_begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

float get_time()
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec;
    float currTimenSec = time_now.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void vector3dLimit(Vector3d &v, double limit)  ///limit should be positive
{
    if(limit > 0){
        for(int i=0; i<3; i++){
            v(i) = fabs(v(i)) > limit ? (v(i) > 0 ? limit : -limit) : v(i);
        }
    }
}

Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2)
{
    Vector3d result;
    result << v1(0)*v2(0), v1(1)*v2(1), v1(2)*v2(2);
    return result;
}


int pix_controller(float cur_time)
{
//位 置 环
    //计算误差
    float error_x = plane_expected_position.x - pose_drone_odom.pose.pose.position.x;
    float error_y = plane_expected_position.y - pose_drone_odom.pose.pose.position.y;
    float error_z = plane_expected_position.z - plane_real_alt.data;
//    std::cout << "error: x：" << error_x << "\ty：" << error_y << "\tz：" << error_z << std::endl;
    //计算指定速度误差
    float vel_xd = param.x_p * error_x;
    float vel_yd = param.y_p * error_y;
    float vel_zd = param.z_p * error_z;
    vel_target.x = vel_xd;
    vel_target.y = vel_yd;
    vel_target.z = vel_zd;

//速 度 环
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    PIDVX.start_intergrate_flag = true;
    PIDVY.start_intergrate_flag = true;
    PIDVZ.start_intergrate_flag = true;
    if(got_initial_point == false){
        PIDVX.start_intergrate_flag = false;
        PIDVY.start_intergrate_flag = false;
        PIDVZ.start_intergrate_flag = false;
    }
    //计算误差
    float error_vx = vel_xd - vel_drone.twist.linear.x+pose_car_odom.twist.twist.linear.x;
    float error_vy = vel_yd - vel_drone.twist.linear.y;
    float error_vz = vel_zd - vel_drone.twist.linear.z;
    //传递误差
    PIDVX.add_error(error_vx, cur_time); //把error放到list中
    PIDVY.add_error(error_vy, cur_time);
    PIDVZ.add_error(error_vz, cur_time);
    //计算输出
    PIDVX.pid_output();
    PIDVY.pid_output();
    PIDVZ.pid_output();

    angle_target.x = asin(-PIDVY.Output/sqrt(pow(PIDVX.Output,2)+pow(PIDVY.Output,2)+pow(PIDVZ.Output+9.8,2)));
    angle_target.y = atan(PIDVX.Output/(PIDVZ.Output+9.8));
    angle_target.z = Yaw_Init;

    orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
//    thrust_target = (float)(0.05 * (9.8 + PIDVZ.Output));   //目标推力值
    thrust_target  = (float)sqrt(pow(PIDVX.Output,2)+pow(PIDVY.Output,2)+pow(PIDVZ.Output+9.8,2))/9.8*(0.56);   //目标推力值

    return 0;
}

/**
 * 将欧拉角转化为四元数
 * @param roll
 * @param pitch
 * @param yaw
 * @return 返回四元数
 */
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}

/**
 * 将四元数转化为欧拉角形式
 * @param x
 * @param y
 * @param z
 * @param w
 * @return 返回Vector3的欧拉角
 */
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    temp.y = asin(2.0 * (w * y - z * x));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}

/**
 * 将进入offboard后的位置&速度&姿态信息记录进文件
 * @param cur_time
 */

void data_log(std::ofstream &logfile, float cur_time, float ros_time)
{
    /*
    logfile << cur_time << "," << pose_car_odom.twist.twist.linear.x<<","<<time_cost.data<<","<<
           (pose_drone_odom.header.stamp.sec+pose_drone_odom.header.stamp.nsec/1e9)<<","<<pose_drone_odom.pose.pose.position.x-3<<","<<
           (pose_car_odom.header.stamp.sec+pose_car_odom.header.stamp.nsec/1e9)<<","<<pose_car_odom.pose.pose.position.x<<","<<
           thrustforceacc<<","<<angle_target.y<<","<< targeterror_msg.z<<","<<ros_time<<","<<std::endl;
           */
    logfile << cur_time << "," << targeterror_msg.x << "," << targeterror_msg.z << "," << time_cost.data << std::endl;
}
