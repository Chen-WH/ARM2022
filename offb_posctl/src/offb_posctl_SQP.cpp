#include <fstream>
#include <math.h>
#include <string>
#include <time.h>
#include <queue>
#include <vector>
#include <cstdlib>
#include <stdlib.h>
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

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Topic <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
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

mavros_msgs::State current_state;                    // 无人机连接状态
nav_msgs::Odometry pose_drone_odom;                  // 读入无人机当前状态，x，y，z+姿态
nav_msgs::Odometry pose_car_odom;                    // 读入小车当前状态，x，y，z+姿态
geometry_msgs::TwistStamped vel_drone;               // 读入的无人机当前速度 线速度+角速度
geometry_msgs::Quaternion orientation_target;        // 发给无人机的姿态指令  四元数
geometry_msgs::Vector3 angle_target;                 // 欧拉角
geometry_msgs::Vector3 vel_target;                   // 期望速度
geometry_msgs::Point plane_expected_position;        // 根据车的当前位置计算期望飞机位置
std_msgs::Float64 plane_real_alt;                    // 无人机高度
mavros_msgs::AttitudeTarget target_atti_thrust_msg;  // 最终发布的消息 油门+角度
mavros_msgs::AttitudeTarget base_atti_thrust_msg;    // 标准油门+角度
nav_msgs::Odometry current_relativepostwist_msg;     // 无人机与小车状态差
geometry_msgs::Vector3 targeterror_msg;              // 追踪误差
geometry_msgs::Vector3 temp_angle;                   // 欧拉角临时量
geometry_msgs::Vector3 rpy;                          // PID环节发布的欧拉角
offb_posctl::controlstate controlstatearray;         // 收到的控制指令
geometry_msgs::Vector3 begin_flag_temp;              // 小车启动

bool begin_flag_bool = false;      // 小车是否开始运动
bool planeupdateflag = false;      // 是否订阅到飞机位置
bool carupdateflag = false;        // 是否订阅到小车位置
float thrust_target;               // 期望推力
float Yaw_Locked = 0;              // 锁定的偏航角(一般锁定为0)
bool got_initial_point = false;    // 是否完成offboard
bool contstaterecieveflag = false; // 控制指令是否更新
PID PIDVX, PIDVY, PIDVZ;           // 声明PID类
Parameter param;
std::ofstream logfile;

float px_ini = 0;                // x方向位置差
float py_ini = 0;                // y方向位置差
float pz_ini = 0;                // z方向位置差
float vx_ini = 0;                // x方向速度差
float vy_ini = 0;                // y方向速度差
float vz_ini = 0;                // z方向速度差
float z0 = 0.1;                  // 起火点高度
float t_end = 2;                 // 时间窗口
double thrustforceacc = 0.0;     // z方向推力
int pointnumber = 100;           // 离散点个数
int controlfreq = 100;            // 控制频率
int discretizedpointpersecond = pointnumber / int(t_end);
int controlcounter = 0;
float time_cost = 0;
int echo_count = 0;              // 命令行输出计数

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>声 明 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw);  // 欧拉角转四元数
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w);     // 四元数转欧拉角
float get_ros_time(ros::Time time_begin);
int pix_controller(float cur_time);                                              // PID控制
void vector3dLimit(Vector3d &v, double limit) ;                                  // 向量元素不得大于limit
Vector3d vectorElementMultiply(Vector3d v1, Vector3d v2);                        // 数组乘法
void data_log(std::ofstream &logfile, float cur_time);                           // 记录误差

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}
void plane_pos_cb(const nav_msgs::Odometry::ConstPtr &msg){
    pose_drone_odom = *msg;
    planeupdateflag= true;
}
void plane_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    vel_drone = *msg;
}
void car_pos_cb(const nav_msgs::Odometry::ConstPtr &msg) {
    pose_car_odom = *msg;
    carupdateflag = true;
}
void plane_alt_cb(const std_msgs::Float64::ConstPtr &msg){
    plane_real_alt = *msg;
}
void controlstate_cb(const offb_posctl::controlstate::ConstPtr &msg)
{
    controlstatearray = *msg;
    controlcounter = controlstatearray.inicounter;
    contstaterecieveflag= true;
    discretizedpointpersecond=controlstatearray.discrepointpersecond;
}
void begin_run_cb(const geometry_msgs::Vector3::ConstPtr &msg){
    begin_flag_temp = *msg;
    
    if (begin_flag_temp.x > 3.0){
        begin_flag_bool = true;
    }
    
}

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control");
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming"); //使能解锁飞机  创建client对象并向arming发出请求，服务类型为CommandBool
    ros::ServiceClient setmode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");      //设置为自动控制模式 服务类型为SetMode
    
    // 订阅无人机当前状态/位置/速度信息
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
    ros::Subscriber plane_position_pose_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 1, plane_pos_cb);               //pos+twist，飞机坐标系
    ros::Subscriber plane_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, plane_vel_cb); //twist
    ros::Subscriber car_position_sub = nh.subscribe<nav_msgs::Odometry>("odom", 1, car_pos_cb);                                              //车的pos+twist，小车坐标系
    ros::Subscriber plane_alt_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/rel_alt", 1, plane_alt_cb);
    ros::Subscriber controlstate_sub = nh.subscribe<offb_posctl::controlstate>("SQP_controlstate", 1, controlstate_cb);
    ros::Subscriber begin_run_sub = nh.subscribe<geometry_msgs::Vector3>("begin_run", 1, begin_run_cb);

    // 发布飞机姿态/拉力信息 坐标系:NED系
    ros::Publisher target_atti_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1); //发布给mavros的控制量(经过换算)
    ros::Publisher plane_rpy_pub = nh.advertise<geometry_msgs::Vector3>("drone/current_rpy", 1);                          //飞机当前的rpy角
    ros::Publisher current_relativepostwist_pub = nh.advertise<nav_msgs::Odometry>("current_relative_postwist", 1);       //当前状态方程中的状态量,即相对量
    ros::Publisher targeterror_pub = nh.advertise<geometry_msgs::Vector3>("targeterror", 1);                              //目标误差

    ros::Rate rate(controlfreq);   //50hz的频率发送/接收topic

    // log输出文件初始化
    int exp_label;
    cout << "请输入实验编号: ";
    cin >> exp_label;
    logfile.open("/home/chenwh/exp_ws/src/offb_posctl/data/case" + to_string(exp_label) + ".csv", std::ios::out);
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
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        ros::Duration(1).sleep();
    }
    ROS_INFO("Connected!!");

    ///set initial hover position and pose
    base_atti_thrust_msg.thrust = 0.56;
    target_atti_thrust_msg.thrust = 0.65;       //65%的油门 50%与重力平衡即悬停
    target_atti_thrust_msg.orientation.x = pose_car_odom.pose.pose.orientation.x;
    target_atti_thrust_msg.orientation.y = pose_car_odom.pose.pose.orientation.y;
    target_atti_thrust_msg.orientation.z = pose_car_odom.pose.pose.orientation.z;
    target_atti_thrust_msg.orientation.w = pose_car_odom.pose.pose.orientation.w;
    ROS_INFO("got initial point");

    //send a few setpoints before starting
    for (int i = 10; ros::ok() && i > 0; --i){
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        ros::spinOnce();               //让回调函数有机会被执行
        rate.sleep();
    }

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>解 锁 飞 机<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))) {
            if (setmode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 
        else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(2.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        if (plane_real_alt.data > 0.15) {
            ROS_INFO("plane takeoff !");
            break;
        }
        ros::spinOnce();
        rate.sleep();
    }

    got_initial_point = true;

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PID 环 节<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    ros::Time begin_time_01 = ros::Time::now();     // 标记当前时间作为ROS时间零点
    while (ros::ok() && !begin_flag_bool) {
        ros::spinOnce();
        if (planeupdateflag && carupdateflag)   //订阅到飞机位置则flag为true，发布完相对位置的消息后flag置false
        {
            px_ini = (pose_drone_odom.pose.pose.position.x - 3) - pose_car_odom.pose.pose.position.x;
            pz_ini = pose_drone_odom.pose.pose.position.z - pose_car_odom.pose.pose.position.z - z0;
            py_ini = pose_drone_odom.pose.pose.position.y - pose_car_odom.pose.pose.position.y;
            vx_ini = vel_drone.twist.linear.x - pose_car_odom.twist.twist.linear.x;
            vz_ini = vel_drone.twist.linear.z - pose_car_odom.twist.twist.linear.z;
            vy_ini = vel_drone.twist.linear.y - pose_car_odom.twist.twist.linear.y;
            ++echo_count;
            if (echo_count > 20){
                cout << "------------Adjusting Loop----------------" << endl;
                std::cout << "px_ini:  " << px_ini << "pz_ini:  " << pz_ini << "vx_ini:  " << vx_ini << "thrust_target: " << thrust_target << std::endl;
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
            planeupdateflag = false;
            carupdateflag = false;
        }

         //远程追踪
         /*
        if (pose_drone_odom.pose.pose.position.x - 3 - pose_car_odom.pose.pose.position.x < -8.0){
            begin_flag_bool = true;
        }*/
        

        float cur_time_01 = get_ros_time(begin_time_01);  // 相对时间
        // 期望飞机位置
        plane_expected_position.x = 0;
        plane_expected_position.y = 0;
        plane_expected_position.z = pose_car_odom.pose.pose.position.z + z0;

        pix_controller(cur_time_01);
        target_atti_thrust_msg.header.stamp = pose_car_odom.header.stamp;
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;

        temp_angle = quaternion2euler(pose_drone_odom.pose.pose.orientation.x, pose_drone_odom.pose.pose.orientation.y, pose_drone_odom.pose.pose.orientation.z, pose_drone_odom.pose.pose.orientation.w);//欧拉角
        rpy.y = temp_angle.y;
        rpy.x = temp_angle.x;
        rpy.z = temp_angle.z;
        plane_rpy_pub.publish(rpy);

        target_atti_thrust_pub.publish(target_atti_thrust_msg);
        rate.sleep();
    }

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主  循  环<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    ros::Time begin_time_02 = ros::Time::now();
    echo_count = 0;
    clock_t start, end;
    while (ros::ok() && begin_flag_bool) {
        start = clock();
        ros::spinOnce();//刷新callback的消息
        if (planeupdateflag && carupdateflag)
        {
            px_ini = (pose_drone_odom.pose.pose.position.x - 3) - pose_car_odom.pose.pose.position.x;
            pz_ini = pose_drone_odom.pose.pose.position.z - pose_car_odom.pose.pose.position.z - z0;
            py_ini = pose_drone_odom.pose.pose.position.y - pose_car_odom.pose.pose.position.y;
            vx_ini = vel_drone.twist.linear.x - pose_car_odom.twist.twist.linear.x;
            vz_ini = vel_drone.twist.linear.z - pose_car_odom.twist.twist.linear.z;
            vy_ini = vel_drone.twist.linear.y - pose_car_odom.twist.twist.linear.y;
            ++echo_count;
            if (echo_count > 20){
                cout << "----------------Numerical control Loop-------------------" << endl;
                std::cout << "px_ini:  " << px_ini << "pz_ini:  " << pz_ini << "vx_ini:  " << vx_ini << std::endl;
                std::cout << "error_x:  " << px_ini + 3 << "error_z:  " << pz_ini + px_ini * rpy.y << "theta： " << angle_target.y << "thrust_target: " << thrust_target << std::endl;
            }
            current_relativepostwist_msg.pose.pose.position.x = px_ini;
            current_relativepostwist_msg.pose.pose.position.z = pz_ini;
            current_relativepostwist_msg.pose.pose.position.y = py_ini;
            current_relativepostwist_msg.twist.twist.linear.x = vx_ini;
            current_relativepostwist_msg.twist.twist.linear.y = vy_ini;
            current_relativepostwist_msg.twist.twist.linear.z = vz_ini;
            current_relativepostwist_msg.header.stamp = pose_drone_odom.header.stamp;
            current_relativepostwist_pub.publish(current_relativepostwist_msg);
            planeupdateflag = false;
        }

        float cur_time_02 = get_ros_time(begin_time_02);
        pix_controller(cur_time_02);                   //备用控制程序

        if (!contstaterecieveflag)  //订阅到控制量更新则flag为true,用于起始时刻,还没算出bvp时
        {
            ++controlcounter;
        }
        contstaterecieveflag = false;

        if (controlcounter + 1 < controlstatearray.arraylength) {
            angle_target.y = controlstatearray.thetaarray[controlcounter];
            thrustforceacc = controlstatearray.thrustarray[controlcounter] - 0.2;
            orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
            thrust_target = (float) (base_atti_thrust_msg.thrust) * thrustforceacc * sqrt(1 + angle_target.y*angle_target.y)/ 9.8;
        }
        ///publish plane current rpy
        temp_angle = quaternion2euler(pose_drone_odom.pose.pose.orientation.x,
                                      pose_drone_odom.pose.pose.orientation.y,
                                      pose_drone_odom.pose.pose.orientation.z,
                                      pose_drone_odom.pose.pose.orientation.w);//欧拉角
        rpy.y = temp_angle.y;
        rpy.x = temp_angle.x;
        rpy.z = temp_angle.z;
        plane_rpy_pub.publish(rpy);

        target_atti_thrust_msg.header.stamp = ros::Time::now();
        target_atti_thrust_msg.orientation = orientation_target;
        target_atti_thrust_msg.thrust = thrust_target;
        target_atti_thrust_pub.publish(target_atti_thrust_msg);

        ///publish targeterror_msg
        targeterror_msg.x = px_ini + 3;
        targeterror_msg.z = pz_ini + px_ini * rpy.y;
        targeterror_msg.y = py_ini;
        targeterror_pub.publish(targeterror_msg);
        end = clock();
        time_cost = (float)(end - start) / CLOCKS_PER_SEC * 1000;
        data_log(logfile, cur_time_02); //保存数据
        if (echo_count > 20){
            printf("u1 %f   u2 %f\n", controlstatearray.thrustarray[controlcounter], controlstatearray.thetaarray[controlcounter]);
            //printf("程序运行时间 %f ms\n", (float)(end - start) / CLOCKS_PER_SEC * 1000);
            echo_count -= 20;
        }
        rate.sleep();
    }
    logfile.close();
    return 0;
}

///>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>函 数 定 义<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// 获取 ROS 相对时间 单位：秒
float get_ros_time(ros::Time time_begin)
{
    ros::Time time_now = ros::Time::now();
    return ((time_now.sec - time_begin.sec) + (time_now.nsec - time_begin.nsec) / 1e9);
}
void vector3dLimit(Vector3d &v, double limit)  ///limit should be positive
{
    if(limit > 0){
        for(int i = 0; i < 3; ++i){
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

    //计算指定速度误差
    float vel_xd = param.x_p * error_x;
    float vel_yd = param.y_p * error_y;
    float vel_zd = param.z_p * error_z;
    vel_target.x = vel_xd;
    vel_target.y = vel_yd;
    vel_target.z = vel_zd;

//速 度 环
    //积分标志位.未进入OFFBOARD时,不累积积分项;进入OFFBOARD时,开始积分.
    if(got_initial_point){
        PIDVX.start_intergrate_flag = true;
        PIDVY.start_intergrate_flag = true;
        PIDVZ.start_intergrate_flag = true;
    }
    else {
        PIDVX.start_intergrate_flag = false;
        PIDVY.start_intergrate_flag = false;
        PIDVZ.start_intergrate_flag = false;
    }
    //计算误差
    float error_vx = vel_xd - vel_drone.twist.linear.x + pose_car_odom.twist.twist.linear.x;
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
    angle_target.y = atan(PIDVX.Output/(PIDVZ.Output + 9.8));
    angle_target.z = Yaw_Locked;

    orientation_target = euler2quaternion(angle_target.x, angle_target.y, angle_target.z);
    thrust_target  = (float)sqrt(pow(PIDVX.Output,2)+pow(PIDVY.Output,2)+pow(PIDVZ.Output + 9.8,2))/9.8*(float) (base_atti_thrust_msg.thrust);   //目标推力值

    return 0;
}
geometry_msgs::Quaternion euler2quaternion(float roll, float pitch, float yaw)
{
    geometry_msgs::Quaternion temp;
    temp.w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) + sin(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) - cos(roll/2)*sin(pitch/2)*sin(yaw/2);
    temp.y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) + sin(roll/2)*cos(pitch/2)*sin(yaw/2);
    temp.z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) - sin(roll/2)*sin(pitch/2)*cos(yaw/2);
    return temp;
}
geometry_msgs::Vector3 quaternion2euler(float x, float y, float z, float w)
{
    geometry_msgs::Vector3 temp;
    temp.x = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    temp.y = asin(2.0 * (w * y - z * x));
    temp.z = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return temp;
}
void data_log(std::ofstream &logfile, float cur_time)
{
    logfile << px_ini + 3 << "," << pz_ini + px_ini * rpy.y << "," << angle_target.y << "," << thrust_target << "," << time_cost << std::endl;
}