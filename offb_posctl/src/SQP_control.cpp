#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/Float32.h"
#include "offb_posctl/controlstate.h"
#include "Matrix.h"
#include "offline_compute.h"
#include <time.h>
#include <cmath>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>参 数 设 置<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

constexpr int n = 7;                  // 点的个数
constexpr int x_num = 5*n;            // 状态变量个数
constexpr int u_num = 2*n;            // 控制变量个数
constexpr int ec_num = 5*n;           // 等号约束个数
float t0 = 0;
float tf = 2;
float g = 9.8;
float k[2] = {50, 50};
float c[2] = {1.5, 0.38};
int pointnumber = 100;      // 离散点数
int iteration = 200;        // 迭代次数上限
float epsilon1 = 1e-7;      // 线搜索精度
float epsilon2 = 1e-5;      // 迭代梯度残差
float epsilon3 = 1e-5;      // 迭代停止flag
float epsilon4 = 1e-3;      // 等号约束接受范围
float x0[5] = { -3, 0, 0, 0, 0 };     // 初始状态
float L1_tmp[8][8] = {-2.6812000e+01 ,  7.2910000e-15 ,  4.3312000e+01,  0.0 , -1.9687000e+01  , 9.1137000e-16  , 2.1875000e+00 , 0.0,
   		4.2221000e+01 ,  2.1487000e+00 , -7.0242000e+01 , -1.5354000e+00 ,  3.2458000e+01,   1.9461000e-01 , -3.6293000e+00 , 0.0,
  		-2.6028000e+01 , -6.7273000e+00  , 4.7033000e+01 ,  7.1681000e+00 , -2.4427000e+01 , -9.9815000e-01  , 2.8636000e+00 , 0.0,
   		1.8021000e+01 ,  1.0707000e+01,  -3.3456000e+01 , -1.5533000e+01 ,  1.9536000e+01 ,  5.3035000e+00,  -3.6227000e+00 , 0.0,
  		-1.2257000e+01  ,-1.2257000e+01 ,  1.9800000e+01 ,  1.9800000e+01 , -9.0000000e+00 , -9.0000000e+00 ,  1.0000000e+00,   1.0,
   		7.6162000e+00 ,  1.0707000e+01 , -7.9576000e+00 , -1.5533000e+01 , -7.1154000e-01 ,  5.3035000e+00  , 1.5310000e+00 , 0.0,
  		-3.8629000e+00 , -6.7273000e+00 ,  1.2515000e+00  , 7.1681000e+00 ,  2.4790000e+00 , -9.9815000e-01 , -4.2500000e-01 , 0.0,
   		1.1024000e+00 ,  2.1487000e+00 ,  2.5854000e-01 , -1.5354000e+00 , -6.4782000e-01  , 1.9461000e-01 ,  9.4762000e-02 ,  0.0};
float L2_tmp[7][7] = {2.1487000e+00 , -2.0393000e+00 , -1.5354000e+00  , 1.4573000e+00,   1.9461000e-01,  -1.8470000e-01 , 0.0,
  		-6.7273000e+00  , 4.9885000e+00,   7.1681000e+00 , -5.3154000e+00 , -9.9815000e-01 ,  7.4016000e-01 , 0.0,
   		1.0707000e+01 , -4.3455000e+00,  -1.5533000e+01 ,  6.3039000e+00 ,  5.3035000e+00  ,-2.1524000e+00 , 0.0,
  		-1.2257000e+01 , -5.6148000e-15 ,  1.9800000e+01 , -3.7432000e-15 , -9.0000000e+00 ,  2.3395000e-16,   1.0,
   		1.0707000e+01 ,  4.3455000e+00,  -1.5533000e+01,  -6.3039000e+00  , 5.3035000e+00 ,  2.1524000e+00 , 0.0,
  		-6.7273000e+00 , -4.9885000e+00 ,  7.1681000e+00 ,  5.3154000e+00 , -9.9815000e-01,  -7.4016000e-01 , 0.0,
   		2.1487000e+00 ,  2.0393000e+00,  -1.5354000e+00 , -1.4573000e+00 ,  1.9461000e-01 ,  1.8470000e-01  , 0.0};

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>变 量 设 置<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

constexpr int var_num = u_num + 5;       // 规划变量个数
constexpr int dim = u_num;               // 规划变量个数
offb_posctl::controlstate controlstate_msg;
float c0 = (tf - t0)/2;
float* tau = new float[pointnumber];
float gap = 2.0/(pointnumber - 1);
Matrix L1(8, 8);
Matrix L2(7, 7);
Matrix poly_x(5, 8);
Matrix poly_u(2, 7);
Matrix var(1, var_num);
Matrix record(1, pointnumber);
Matrix res(6, pointnumber);

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>离 线 计 算<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// 目标函数 F
float getF(float w[]) {
	float ans = F(w);
	return ans;
}
// 目标函数梯度 dF
Matrix getdF(float w[]){
	Matrix tmp(1, u_num);
	float dF[u_num] = dF(w);
	memcpy(tmp.Index(), dF, u_num*sizeof( float ));
	return tmp;
}
// 目标函数海森阵 HF
Matrix getHF(float w[]){
	Matrix tmp(u_num, u_num);
	float HF[u_num*u_num] = HF(w);
	memcpy(tmp.Index(), HF, u_num*u_num*sizeof( float ));
	return tmp;
}
// 等号约束 H
Matrix getH(float w[]){
	Matrix tmp(1, ec_num);
	float H[ec_num] = H(w);
	memcpy(tmp.Index(), H, ec_num*sizeof( float ));
	return tmp;
}
// 状态量 X
Matrix getX(float w[]){
	Matrix tmp(5, n);
	float X[x_num] = X(w);
	memcpy(tmp.Index(), X, x_num*sizeof( float ));
	return tmp;
}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>逐 步 二 次 规 划<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// 初始化函数
void init(){
	for (int i = 0; i < pointnumber; ++i){
		tau[i] = -1 + gap*i;
	}
	srand((unsigned)time(NULL));
	for (int i = 0; i < u_num; ++i) {
		var(0, i) = rand() / float(RAND_MAX);
	}
	for (int i = 0; i < 8; ++i){
		for (int j = 0; j < 8; ++j){
			L1(i, j) = L1_tmp[i][j];
		}
	}
	for (int i = 0; i < 7; ++i){
		for (int j = 0; j < 7; ++j){
			L2(i, j) = L2_tmp[i][j];
		}
	}
}
// dL = [∇F]
Matrix getB(Matrix guess) {
	Matrix tmp(1, u_num);
	tmp.cpy(getdF(guess.Index()), 0, 0);
	return tmp;
}
// HL = [∇^2F]
Matrix getA(Matrix guess) {
	Matrix tmp(u_num, u_num);
	tmp.cpy(getHF(guess.Index()), 0, 0);
	return tmp;
}
// 惩罚函数
float penalty(Matrix guess) {
	float ans = getF(guess.Index());
	return ans;
}
// 一维线搜索函数
float linesearch(Matrix direction, Matrix guess) {
	float alpha1 = 0;
	float alpha2 = 0.382;
	float alpha3 = 0.618;
	float alpha4 = 1;
	float tp1, tp2;
	Matrix tmp;
	while (alpha4 - alpha1 > epsilon1) {
		tmp = guess - direction*alpha2;
		tp1 = penalty(tmp);
		tmp = guess - direction*alpha3;
		tp2 = penalty(tmp);
		if (tp1 < tp2) {
			alpha4 = alpha3;
			alpha3 = alpha2;
			alpha2 = alpha1 + 0.382 * (alpha4 - alpha1);
		}
		else {
			alpha1 = alpha2;
			alpha2 = alpha3;
			alpha3 = alpha1 + 0.618 * (alpha4 - alpha1);
		}
	}
	return alpha2;
}
// QP子问题
Matrix subQP(Matrix guess) {
	float alpha1, alpha2, point1, point2;
	Matrix tmp, d1(1, var_num), d2(1, var_num);;
	Matrix A = getA(guess);
	Matrix B = getB(guess);
	Matrix tp = A.solve(B.trans()).trans();
	d1.cpy(tp, 0, 0);
	memset(d1.Index() + u_num, 0, 5*sizeof( float ));
	tp = getdF(guess.Index());
	d2.cpy(tp, 0, 0);
	memset(d2.Index() + u_num, 0, 5*sizeof( float ));
	//沿SQP方向搜索
	alpha1 = linesearch(d1, guess);
	tmp = guess - d1*alpha1;
	point1 = getF(tmp.Index());
	//沿梯度方向搜索
	alpha2 = linesearch(d2, guess);
	tmp = guess - d2*alpha2;
	point2 = getF(tmp.Index());
	if (point1 < point2) {
		return d1*alpha1;
	}
	else {
		return d2*alpha2;
	}
}
// 校验函数
bool check(Matrix guess, float step) {
	if (step > epsilon3){
		return false;
	}
	if (getdF(var.Index()).norm1() > epsilon2){
		return false;
	}
	Matrix check_H = getH(guess.Index());
	if (fabs(check_H.norm1()) > epsilon4){
		return false;
	}
	return true;
}
// 后处理函数
void post(){
	for (int i = 0; i < 5; ++i){
		poly_x(i, 0) = x0[i];
	}
	poly_x.cpy(getX(var.Index()), 0, 1);
	memcpy(poly_u.Index(), var.Index(), u_num*sizeof( float ));
	poly_x = poly_x*L1;
	poly_u = poly_u*L2;
	for (int i = 0; i < 4; ++i){
		for (int j = 0; j < pointnumber ;++j){
			res(i, j) = 0;
			for (int k = 0; k < 8; ++k){
				res(i, j) = res(i, j)*tau[j] + poly_x(i, k);
			}
		}
	}
	for (int i = 4; i < 6; ++i){
		for (int j = 0; j < pointnumber ;++j){
			res(i, j) = 0;
			for (int k = 0; k < 7; ++k){
				res(i, j) = res(i, j)*tau[j] + poly_u(i - 4, k);
			}
		}
	}
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>回 调 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void plane_vel_cb(const geometry_msgs::TwistStamped::ConstPtr &msg){
    x0[4] = msg->twist.linear.x;
}
void pos_twist_cb(const nav_msgs::Odometry::ConstPtr &msg){
    x0[0] = msg->pose.pose.position.x;
	x0[1] = msg->pose.pose.position.z;
	x0[2] = msg->twist.twist.linear.x;
	x0[3] = msg->twist.twist.linear.z;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SQP_control");
	ros::NodeHandle nh;
	ros::Subscriber current_relativepostwist_sub = nh.subscribe<nav_msgs::Odometry>("current_relative_postwist", 1, pos_twist_cb);//当前状态方程中的状态量,即相对量
	ros::Subscriber plane_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 1, plane_vel_cb); //twist
	ros::Publisher controlstate_pub = nh.advertise<offb_posctl::controlstate>("SQP_controlstate", 1);
	ros::Rate loop_rate(100);
	int count;  //迭代次数
	clock_t start, end;
	bool loop_flag;
	init();
  	while (ros::ok()){
		count = 0;
		start = clock();
		loop_flag = true;
		// 获取x0
		for (int i = u_num; i < var_num; ++i){
			var(0, i) = x0[i - u_num];
		}
		float compare = 1e+9;
		for (int repeat = 0; repeat < 5; ++repeat){
			srand((unsigned)time(NULL));
			for (int i = 0; i < u_num; ++i) {
				var(0, i) = rand() / float(RAND_MAX);
			}
			while (loop_flag && count < iteration) {
				loop_flag = true;
				Matrix d = subQP(var);
				var = var - d;
				if (check(var, d.norm1())) {
					loop_flag = false;
				}
				++count;
			}
			if (penalty(var) < compare){
				record = var;
				compare = penalty(var);
			}
		}
		var = record;
		post();
		end = clock();
		//printf("运行时间%lfms\n", (double)(end - start) / CLOCKS_PER_SEC * 1000);
		controlstate_msg.inicounter = 0;
		controlstate_msg.discrepointpersecond = 50;
		controlstate_msg.error = getdF(var.Index()).norm1();
		controlstate_msg.arraylength = 100;
		vector<float> x1(res.Index() + 0*pointnumber, res.Index() + 1*pointnumber);
		vector<float> x2(res.Index() + 1*pointnumber, res.Index() + 2*pointnumber);
		vector<float> x3(res.Index() + 2*pointnumber, res.Index() + 3*pointnumber);
		vector<float> x4(res.Index() + 3*pointnumber, res.Index() + 4*pointnumber);
		vector<float> u1(res.Index() + 4*pointnumber, res.Index() + 5*pointnumber);
		vector<float> u2(res.Index() + 5*pointnumber, res.Index() + 6*pointnumber);
		controlstate_msg.thrustarray = u1;
		controlstate_msg.thetaarray = u2;
		controlstate_msg.stateXarray = x1;
		controlstate_msg.stateZarray = x2;
		controlstate_msg.stateVXarray = x3;
		controlstate_msg.stateVZarray = x4;
		controlstate_pub.publish(controlstate_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}