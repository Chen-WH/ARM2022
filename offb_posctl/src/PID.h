//
// Created by zm on 18-12-1.
//

#include <cstdlib>   //函数库
#include <utility>   //用pair、make_pair时要用
#include <stdlib.h>  //定义了五种类型、一些宏和通用工具函数
#include <iostream>  //标准输入输出流
#include <stdio.h>  //standard input & output 标准输入输出，要用到标准输入输出函数时必须包含
#include <fstream> //file stream
#include <cmath>   //数学函数
#include "string"  //字符串处理
#include <time.h>  //时间
#include <queue>   //队列
#include <vector>  //动态数组
#include <algorithm> //常用于计算、数据处理和自动推理

using namespace std;//释放标准命名空间std中的东西，调用cout函数时可以不用写成std::cout<<...<<endl；  因为在程序中要使用变量a和b，必须加上命名空间名和作用域分辨符“::”，如nsl::a

class PID {

public:
    //构造函数
    PID();
    float Kp;                          //参数P
    float Ki;                          //参数I
    float Kd;                          //参数D

    float error;                       //误差量 = 实际值 - 期望值
    float delta_time;                  //时间间隔dt

    std::vector <std::pair<float, float> > error_list; //误差表,用作计算微分项 平滑窗口 [2nd data, 1st time]

    float P_Out;                       //P环节输出 Kp*error
    float I_Out;                       //I环节输出
    float D_Out;                       //D环节输出
    float Output;                      //输出

    bool start_intergrate_flag;        //是否积分标志[进入offboard(启控)后,才开始积分] 1 or 0
    float Imax;                        //积分上限
    float Output_max;                  //输出最大值
    float errThres;                    //误差死区(if error<errThres, error<0)

    //设置PID参数函数[Kp Ki Kd]
    void setPID(float p_value, float i_value, float d_value);
    //设置积分上限 控制量最大值 误差死区
    void set_sat(float i_max, float con_max, float thres);
    //输入 误差 和 当前时间
    bool add_error(float input_error, float curtime);
    void pid_output(void);

    //饱和函数
    float satfunc(float data, float Max, float Thres);
};

//PID()函数，用于初始化
PID::PID() {
    error_list.push_back(make_pair(0.0f, 0.0f)); //error_list(0)=(0,0)
    error = 0;
    P_Out = 0;
    I_Out = 0;
    D_Out = 0;
    Output = 0;
    start_intergrate_flag = false;
}

//饱和函数 I环节限幅Thres<= I_out<=Max
float PID::satfunc(float data, float Max, float Thres)
{
    if (fabs(data)<Thres)
        return 0;
    else if(fabs(data)>Max){
        return (data>0)?Max:-Max;//if语句的简化 data>0则返回Max 若<0则返回-Max
    }
    else{
        return data;
    }
}

//设置PID参数函数[Kp Ki Kd]
void PID::setPID(float p_value, float i_value, float d_value)
{
    Kp = p_value;
    Ki = i_value;
    Kd = d_value;
}
//设置积分上限 控制量最大值 误差死区
void PID::set_sat(float i_max, float con_max, float thres)
{
    Output_max = con_max;
    Imax = i_max;
    errThres = thres;
}

//输入误差 和 当前时间
bool PID::add_error(float input_error, float curtime)
{
    error = input_error;
    if(error_list.size() == 1)
    {
        delta_time = curtime;
    }
    else{
        delta_time = curtime - error_list.rbegin()->first; //error_list 逆向队列的第一个元素
    }

    if(error_list.size()<10){
        error_list.push_back(make_pair(curtime, error)); //errorlist只存放10个pair
    }
    else{
        vector<pair<float, float> >::iterator k_beg = error_list.begin();//定义一个可以迭代pair类型的迭代器k_beg，指向error_list的首位
        error_list.erase(k_beg);//删除第k_beg个元素
        std::pair<float,float > p1(curtime, error); //定义pair类型的p1，并用(curtime, error)初始化
        error_list.push_back(p1);//新增p1
    }

    return true;
}

void PID::pid_output(void)
{
    P_Out = Kp * error;                          //P环节输出值
    I_Out = I_Out + Ki *error*delta_time;        //I环节输出值
    I_Out = satfunc(I_Out, Imax, 0);             //I环节限幅[I_Out<=Imax]
    if(start_intergrate_flag == 0)
    {
        I_Out = 0;
    }

    D_Out = 0;

    if (error_list.size() < 3 || Kd == 0)
    {
        D_Out = 0; //initiral process
    }
    else
    {
        vector<pair<float, float> >::reverse_iterator error_k = error_list.rbegin(); //传回一个逆向队列的第一个数据。
        vector<pair<float, float> >::reverse_iterator error_k_1 = error_k + 1;
        D_Out = (error_k->second - error_k_1->second)/delta_time * Kd;
    }

    Output = P_Out + I_Out + D_Out;
    Output = satfunc(Output, Output_max, errThres);
}
