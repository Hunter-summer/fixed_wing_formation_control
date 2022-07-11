/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: guanbin
 * @Email: 158242357@qq.com
 * @Description:  
 * 本程序的作用是将飞控的模式切换为需要的模式 
 */


#ifndef _TASK_MAIN_HPP_
#define _TASK_MAIN_HPP_

#include <ros/ros.h>
#include <iostream>
#include<vector>
#include "../fixed_wing_lib/syslib.hpp"
#include "fixed_wing_formation_control/FWstates.h"
#include "fixed_wing_formation_control/FWcmd.h"
#include "fixed_wing_formation_control/Leaderstates.h"
#include "fixed_wing_formation_control/Formation_control_states.h"
#include "fixed_wing_formation_control/Fwmonitor.h"
#include "fixed_wing_formation_control/Fw_cmd_mode.h"
#include "fixed_wing_formation_control/Fw_current_mode.h"
#include "fixed_wing_formation_control/my_ctrl.h"
#include "../formation_controller/formation_controller.hpp"
#include "../formation_controller/abs_formation_controller.hpp"
#include "../fixed_wing_lib/mathlib.hpp"
#include <mavros_msgs/WaypointList.h>
#include "../fixed_wing_lib/vector.hpp"
#include "../fixed_wing_lib/pid_controller.hpp"
#include "../fixed_wing_lib/filter.hpp"
#include "../fixed_wing_lib/pid_controller.hpp"
#include "../fixed_wing_lib/lateral_controller/L1_vel_controller.hpp"
#include "./Nav_solution.hpp"
using namespace std;

#define TASK_MAIN_INFO(a) cout << "[TASK_MAIN_INFO]:" << a << endl
#define MAX_POINT_NUM  100

#define Rad2Deg 57.2957805
#define Deg2Rad 0.01745329
#define PI 3.1415926535

class TASK_MAIN: public FORMATION_CONTROLLER
{
private:
    int planeID{1};                                                   /*飞机编号*/
    string uavID{ "uav1/"}; /* 本机编号，用于命名空间 */

    string leaderID{ "uav0/"};/* 领机编号，用于命名空间 */

    void print_data(const struct ABS_FORMATION_CONTROLLER ::_s_fw_states *p); /*打印数据*/

    ros::NodeHandle nh; /*ros句柄  nh句柄*/
    ros::Time begin_time;
    float leaderstates_update{0.0};
    float current_time;
    double ground_vel;
    float get_ros_time(ros::Time begin); /*获取当前时间*/

    ros::Subscriber fwmonitor_sub;               /*【订阅】来自fw_monitor任务状态的flags*/
    ros::Subscriber fw_states_sub;               /*【订阅】来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber leader_states_sub;           /*【订阅】来自通讯节点，或者视觉节点领机全部状态量*/
    ros::Subscriber fw_cmd_mode_sub;             /*【订阅】来自commander状态机的控制指令*/
    ros::Subscriber way_points_sub;

    ros::Publisher formation_control_states_pub; /*【发布】编队控制状态量*/
    ros::Publisher fw_cmd_pub;                                      /*【发布】飞机四通道控制量*/
    ros::Publisher fw_current_mode_pub;                /*【发布】飞机当前处于的任务状态*/
    //ros::Publisher iris_local_postion_sp_pub;
    ros::Publisher mc_local_att_sp_pub;

    fixed_wing_formation_control::FWstates fwstates;                                 /*自定义--飞机打包的全部状态*/
    fixed_wing_formation_control::FWcmd fw_4cmd;                                     /*自定义--飞机四通道控制量*/
    fixed_wing_formation_control::Leaderstates leaderstates;                         /*自定义--领机状态*/
    fixed_wing_formation_control::Formation_control_states formation_control_states; /*自定义--编队控制状态量*/
    fixed_wing_formation_control::Fwmonitor fwmonitor_flag;                          /*自定义--任务状态的flags*/
    fixed_wing_formation_control::Fw_cmd_mode fw_cmd_mode;                           /*自定义--来自commander的命令飞机模式*/
    fixed_wing_formation_control::Fw_current_mode fw_current_mode;                   /*自定义--要发布的飞机当前任务模式*/
    fixed_wing_formation_control::my_ctrl mc_ctrl;
    
    //fixed_wing_formation_control::my_xmf_state xmf;

    void Navigation_solution();
    void Navigation(double lat_m, double long_m, double lat_s, double long_s, double lat_e, double long_e);
    void ros_sub_pub();                                                                     /*ros消息订阅与发布*/
    void fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg);     /*任务状态flagscallback*/
    void fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg);          /*本机状态callback*/
    void leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg); /*领机状态callback*/
    void fw_cmd_mode_cb(const fixed_wing_formation_control::Fw_cmd_mode::ConstPtr &msg);    /*commander指令callback*/
    mavros_msgs::WaypointList current_waypoints;
    void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg);

    void update_fwsates();                    /*更新飞机状态*/
    void control_protected();
    void control_formation();               /*编队控制主函数*/

    void fw_state_pub();


    /* 滤波后的信息 */
    _s_fw_states fw_states_f;

    /* 是否使用滤波器对原始数据滤波 */
    bool use_the_filter{true};
    FILTER fw_filter_gol_vx;
    FILTER fw_filter_gol_vy;

    //  MY_CONTROLLER my_ctl;
    // MY_CONTROLLER my_roll_ctrl;

    /* 本机地速向量 */
    Vec fw_gspeed_2d;

    /* 本机空速向量 */
    Vec fw_arispd;

    /* 本机dir_cos，这其中的dir这个角度，可能是yaw，也可能是速度偏角 */
    double fw_cos_dir{0.0};

    /* 本机dir_sin，这其中的dir这个角度，可能是yaw，也可能是速度偏角*/
    double fw_sin_dir{0.0};

    /* 检验计算本机的空速（状态）以及实际读取的空速的合法性 */
    bool fw_airspd_states_valid{true};

    int cnt=0;
    void filter_states();
    float start_time;
    float control_time;

    int record=1;
    float roll_0,yaw_0,pitch_0;
    PID_CONTROLLER speed_ctl;
    PID_CONTROLLER takeoff_speed_ctl;
    PID_CONTROLLER height_ctl;
    bool rest_pid_speed{false};
    bool rest_pid_height{false};
    bool rest_pid_takeoff_speed{false};

    float last_height{0};
    float vh{0};
    float _dt=0.02;
     float Hcx=0,Vycx=0,Vy0;
    

    void control_takeoff();                     /*起飞控制主函数*/
    void control_takeoff1();                     /*起飞控制主函数*/
    void control_takeoff22();                     /*起飞控制主函数*/
    void control_takeoff_mc();
    //旋翼起飞参数
    



    void control_land();                     /*降落控制主函数*/
    void control_land_iris();
    float  time_test{0.0};
    void control_position(Point waypoint_sp); /*位置控制主函数*/
    void control_formation2();
    Point fw_NE_Distance;
    /* 横侧向控制器 */
    L1_CONTROLLER l1_controller;
    float roll_cmd{0.0};
    float throttle_cmd{0.0};
    float yaw_cmd{0.0};
    float h_err_i{0};
    /* 重置横侧向控制器 */
    bool reset_lateral_controller{false};
    void control_mission();                    /*任务控制主函数*/
    void control_takeoff2();
    void control_zcontrol1();
    void control_takeoff_iris();
  
    void printwaypoint(const mavros_msgs::WaypointList points);
    Point get_plane_to_sp_vector(Point origin, Point target);
    float t,t0,t1,t2,t3,t4,t5,t6,t7;
    ABS_FORMATION_CONTROLLER formation_controller; /*编队控制器*/
    string fw_col_mode_current{"MANUAL"};   /*当前模式*/
    string fw_col_mode_last{"MANUAL"};      /*上一时刻模式*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_model_params fw_params;                   /*飞机模型参数*/
    struct ABS_FORMATION_CONTROLLER::_s_tecs_params tecs_params;                     /*编队控制器内部TECS控制器参数*/
    struct ABS_FORMATION_CONTROLLER::_s_lateral_controller_params later_ctrl_params; /*编队控制器内部横侧向控制器参数*/
    void input_params();                                                      /*将外部的文件之中的参数加载到相应的函数当中去*/
    float pitch_cmd{0.0};
    bool hv_contol_takeoff{false};
    //编队参数结构体
    struct ABS_FORMATION_CONTROLLER::_s_leader_states leader_states;       /*领机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_states thisfw_states;           /*本机信息*/
    struct ABS_FORMATION_CONTROLLER::_s_4cmd formation_cmd;                /*四通道控制量*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_error formation_error;          /*编队误差以及偏差*/
    struct ABS_FORMATION_CONTROLLER::_s_mix_Xerr_params mix_Xerr_params; /*编队控制器混合误差产生参数,编队控制器参数*/
    struct ABS_FORMATION_CONTROLLER::_s_mix_Yerr_params mix_Yerr_params; /*编队控制器混合误差产生参数,编队控制器参数*/
    struct ABS_FORMATION_CONTROLLER::_s_fw_sp formation_sp;                /*编队控制运动学期望值*/
    void formation_states_pub();                                    /*发布编队控制器控制状态*/
    Point waypoint_sp;
    int  POINT_NUM=10;
    float Distance{999};
    int waypoint_next=0;
    double waypoint_cin[30][2]={
        0,0,//1
        47.3977356,8.5518536,//2
        47.3977356,8.5565386,//3
        47.3977356,8.5628042,//4
        47.3943576,8.5628042,//5
        47.3909796,8.5628042,//6
        47.3909796,8.5531749,//7
        47.3909796,8.5435457,//8
        47.3943576,8.5435457,//9
        47.3977356,8.5435457//10
    };  //航路点
    Point a_pos,b_pos;
    Vec dir;
    double Rcmd{0},RVcmd{0};
    double heading_cur{0};
    double I_p[3],I_t[3],I_s[3],I_c[3],I_0[3];
    double cur_lat_lon[2];
    double sp_lat_lon[2];
    double pre_lat_lon[2];
    double sp_c_lat_lon[2];
    double sp_o_lat_lon[2];
    double next_sp_lat_lon[2];
    double Dis{0},Dis_y{0},DYr{0},Dis_y_integ{0},gD{0};
    double heading_sp,heading_sp_next;
    double Turn_advance_distance{0};
    double Turn_angle{0};
    bool Turn_flag{false},Straight_flag{true};
    double constrain(int val,int min ,int max);
    double sign(double a);

    //积分分离相关
    double ki_a,HH;
    double Dis_y_i;//分离界限
    //抗饱和以及积分分离积分环节
   // double pid_integral(a_err,a,a_max,a_min);
    Nav_solution NAV_solution; 
       /*     waypoint_cin[15][2]={
      (47.3977509000,8.5510530000),//1
      (47.3977394000,8.5550530000),//2
      (47.3977356000,8.5575371000),//3
      (47.3977280000,8.5602236000),//4
      (47.3977394000,8.5628042000),//5
      (47.3964310000,8.5628395000),//6
      (47.3953934000,8.5627728000),//7
      (47.3941917000,8.5627718000),//8
      (47.3941612000,8.5593901000),//9
      (47.3941422000,8.5567904000),//10
      (47.3941422000,8.5567904000),//11
      (47.3941269000,8.5526352000),//12
      (47.3941345000,8.5504847000),//13
      (47.3941269000,8.5480556000),//14
      (47.3941460000,8.5435457000)//15
    } ;*/
    /*  way_point_sp[0].x=47.3977509000;
        way_point_sp[0].y=8.5510530000;

        way_point_sp[1].x=47.3977394000;
        way_point_sp[1].y=8.5550530000;

        way_point_sp[2].x=47.3977356000;
        way_point_sp[2].y=8.5575371000;

        way_point_sp[3].x=47.3977280000;
        way_point_sp[3].y=8.5602236000;

        way_point_sp[4].x=47.3977394000;
        way_point_sp[4].y=8.5628042000;

        way_point_sp[5].x=47.3964310000;
        way_point_sp[5].y=8.5628395000;

        way_point_sp[6].x=47.3953934000;
        way_point_sp[6].y=8.5627728000;

        way_point_sp[7].x=47.3941917000;
        way_point_sp[7].y=8.5627718000;

        way_point_sp[8].x=47.3941612000;
        way_point_sp[8].y=8.5593901000;

        way_point_sp[9].x=47.3941422000;
        way_point_sp[9].y=8.5567904000;

        way_point_sp[10].x=47.3941422000;
        way_point_sp[10].y=8.5567904000;

         way_point_sp[11].x=47.3941269000;
        way_point_sp[11].y=8.5526352000;

        way_point_sp[12].x=47.3941345000;
        way_point_sp[12].y=8.5504847000;

        way_point_sp[13].x=47.3941269000;
        way_point_sp[13].y=8.5480556000;

        way_point_sp[14].x=47.3941460000;
        way_point_sp[14].y=8.5435457000;
        */

double Transition_function(double t_begin,double t_current,double t_end);
int POINT_NUM_QGC{0};
double waypoint_qgc[MAX_POINT_NUM][2];
void get_waypoint_from_qgc_to_rk3399(const mavros_msgs::WaypointList points);
public:
  void run();
  void set_planeID(int id);
  void  test1();
  void  test2();


  /**********************聂工*********************************/
  int run_flag=0;
  int start_control=0;
  int first_control=1;
  double ALPHA_start={0.0};
      //俯仰参数
    double t_Flight{0.0};
    double g_Height{0.0},ALFA_cmd{0.0};
    int flag_xh=0,flag_sd=0;
    double VYold0{0.0},time_old{0.0},wz_cmd=0;
    double H_cx{0.0},Hold{0.0},ALFA_cmd_old0{0.0},ALFA_cmd_old{0.0},Vy_CX{0.0},VY{0.0};
    double VYold{0.0}, H_cx1{0.0}, time_old1{0.0};
    double JF_NY{0.0},u_Ymis_n{0.0},u_VY_n{0.0}; 
    double k11{0.0},u_GAMMA{0.0},THETA_v{0.0};
    double u_THETA{0.0};
    double u_VZ {0.0};
    double u_VZ_zw {0.0};
    double  z_control[5],V_MIS{0.0},Fai_real{0.0},JF_ma{0.0}; 
    double  m_control[5];
    //滚转参数
    double kpgd{0.0},kigd{0.0},kdgd{0.0};
    double VN_x{0.0},VE_z{0.0};
    double GAMMAcx{0.0},JF_NZ{0.0};
    int flag_zw=0;
    double STEP_20ms=0.02;
    double JF_Zmis_n{0.0},t_turn{0.0};
    double aturn{0.0},disturn{0.0};
    double R_Target{0.0},PSCI_t{0.0};
    //毕业数据存储
    double date_pitch{0.0},date_pitchcmd{0.0};
    double date_roll{0.0},date_rollcmd{0.0};
    double date_v{0.0},date_vcmd{0.0};
    double date_h{0.0},date_hcmd{0.0};
    double date_vy{0.0},date_vycmd{0.0};
    double date_delta_perr{0.0},date_delta_verr{0.0};
    double date_delta_gd{0.0};
    double date_yaw{0.0},date_yawcmd{0.0};

  double start_height;
    //导航点 
    double Long_Target0{123.374722*Deg2Rad},Lat_Target0{42.392222*Deg2Rad},angle_Target0{0.0};
    double Long_Target1{123.368644*Deg2Rad},Lat_Target1{42.384428*Deg2Rad},angle_Target1{90};
    double Long_Target2{123.364229*Deg2Rad},Lat_Target3{42.386320*Deg2Rad},angle_Target3{90};
    double Long_Target3{123.370308 *Deg2Rad},Lat_Target2{42.394114*Deg2Rad},angle_Target2{90};
    double Long_Target4{123.374724*Deg2Rad},Lat_Target4{42.392221* Deg2Rad},angle_Target4{90};
    double Long_Target5{123.374722 *Deg2Rad},Lat_Target5{42.392222* Deg2Rad},angle_Target5{0};
    int flag_point=1;
    double angle_Target=0;

    double Long_Target_arr[5]= {Long_Target1,Long_Target2,Long_Target3,Long_Target4,Long_Target1  };
	  double Lat_Target_arr [5] = {Lat_Target1,Lat_Target2,Lat_Target3,Lat_Target4,Lat_Target1};
    double angle_Target_arr[5] = {angle_Target1,angle_Target2,angle_Target3,angle_Target4,angle_Target5};
    double Turn_long{0.0},Turn_lat{0.0};
    
    double Lat{0.0},Long{0.0}, Lat_ini{0.0}, Long_ini{0.0},Lat_Target{0.0},Long_Target{0.0};
  //状态更新
   void  status_update();
  //通用函数
    void RelativeLocation(double lat_m, double long_m, double lat_s, double long_s, double lat_e, double long_e);
    void Func_Dictate();
    double limit1(double x,double t);
    double limit2(double x,double min,double max);

    double Transition(double t_begin,double t_end,double t_current);
    void Vincent_Z(double S_MT,double Alf_MT,double g_Longitude,double g_Latitude,double *g_Longitude_T,double *g_Latitude_T);
    void Vincenty_F(double Long_T,double Lat_T,double g_Longitude,double g_Latitude,double *g_R_MT,double *g_Alpha12_MT);

   //控制函数 
    void Func_ADRC_FY();
    void Func_ADRC_Gd();


   
};

double TASK_MAIN::limit1(double x,double t)
{
      if(x<-t) return -t;
      else if(x>=t) return t;
      else return x;
}

double TASK_MAIN::limit2(double x,double b,double a)
{
      if(x<a) return a;
      else if(x>b) return b;
      else return x;
}

//平滑处理函数
double TASK_MAIN::Transition(double t_begin,double t_end,double t_current)
{
    if(t_current<t_begin)
    {
        return 0;
    }
   else if(t_begin<=t_current&&t_current<=t_end)
   {
       return 0.5*(sin(((t_current-t_begin)/(t_end-t_begin)-0.5)*PI)+1);
   }
   else 
    return 1;
}


double TASK_MAIN::sign(double a)
{
      float result;
      if (a < 0)
          result = -a;

      else
          result = a;

      return result;
}


void TASK_MAIN::status_update()
{
  /***************记录初始状态*************/
    if(first_control==1)
    {
        start_time=current_time; //记录当前时间记作开始时间
        first_control=0;                      //只进入一次
        Lat_ini=thisfw_states.latitude;//记录初始经纬度
        Long_ini=thisfw_states.longitude;
        Lat_Target=Lat_Target1;             //载入第一航点经纬度
        Long_Target=Long_Target1;     
        R_Target  = 180.0;          //转弯半径
    }
    /************飞行时间************/
    t_Flight=current_time-start_time; //更新启控时间
      /************水平高度************/
    g_Height=thisfw_states.relative_alt;//更新高度
    u_Ymis_n=thisfw_states.relative_alt;//更新高度
      /************姿态更新************/
    u_GAMMA=thisfw_states.roll_angle*Rad2Deg;//更新滚转角
    //THETA_v=thisfw_states.pitch_angle;//问一下
      /************绝对速度************/
    V_MIS=sqrt(thisfw_states.ned_vel_x*thisfw_states.ned_vel_x+
                              thisfw_states.ned_vel_y*thisfw_states.ned_vel_y+
                              thisfw_states.ned_vel_z*thisfw_states.ned_vel_z);//更新飞行速度
    /************速度更新***********/
   VN_x=thisfw_states.ned_vel_x;
   VE_z=thisfw_states.ned_vel_y;
   u_VY_n=-thisfw_states.ned_vel_z;
   VY=-thisfw_states.ned_vel_z;
  /************经纬更新************/
    Lat=thisfw_states.latitude*Deg2Rad;    
    Long=thisfw_states.longitude*Deg2Rad;
  /************方位更新************/
    PSCI_t=thisfw_states.yaw_angle;//rad
    if(PSCI_t>=0.0&&PSCI_t<=PI)
           PSCI_t=-PSCI_t;
    else if(PSCI_t>PI&&PSCI_t<2.0*PI)
           PSCI_t=-1.0*(PSCI_t-2.0*PI);

}
void TASK_MAIN::RelativeLocation(double lat_m, double long_m, double lat_s, double long_s, double lat_e, double long_e)
{
	//rg:距离
	double xg, zg, sg, rg, angle_a;
	double I_s[3], I_e[3], I_v[3], I_n[3], I_se[3], I_ve[3], I_ht0[3], I_ht0_n[3], I_n0[3];
	double lat_wd, long_wd;
    //坐标系转换`
	I_s[0] = sin(lat_s);
	I_s[1] = cos(lat_s) * cos(long_s);
	I_s[2] = cos(lat_s) * sin(long_s);

	I_e[0] = sin(lat_e);
	I_e[1] = cos(lat_e) * cos(long_e);
	I_e[2] = cos(lat_e) * sin(long_e);

	I_v[0] = sin(lat_m);
	I_v[1] = cos(lat_m) * cos(long_m);
	I_v[2] = cos(lat_m) * sin(long_m);

	I_n[0] = cos(lat_m);
	I_n[1] = -sin(lat_m) * cos(long_m);
	I_n[2] = -sin(lat_m) * sin(long_m);

	I_se[0] = I_e[0] - I_s[0];
	I_se[1] = I_e[1] - I_s[1];
	I_se[2] = I_e[2] - I_s[2];



	I_ve[0] = I_e[0] - I_v[0];
	I_ve[1] = I_e[1] - I_v[1];
	I_ve[2] = I_e[2] - I_v[2];

	I_ht0[0] = I_e[1] * I_s[2] - I_e[2] * I_s[1];
	I_ht0[1] = I_e[2] * I_s[0] - I_e[0] * I_s[2];
	I_ht0[2]=  I_e[0] * I_s[1] - I_e[1] * I_s[0];

	I_n0[0] = 1;
	I_n0[1] = 0;
	I_n0[2] = 0;

	double norm_ht0 = sqrt(I_ht0[0] * I_ht0[0] + I_ht0[1] * I_ht0[1] + I_ht0[2]*I_ht0[2]);
	I_ht0[0] = I_ht0[0] / norm_ht0;
	I_ht0[1] = I_ht0[1] / norm_ht0;
	I_ht0[2] = I_ht0[2] / norm_ht0;

	double lat0 = acos(I_ht0[0] * I_n0[0] + I_ht0[1] * I_n0[1] + I_ht0[2] * I_n0[2]);
	if (lat0 > 3.1415926535897932384626433832795 / 2.0)
		lat0 = 3.1415926535897932384626433832795 - lat0;
	if (lat_s >= 0)
		lat_wd = fabs(lat0);
	else
		lat_wd = -fabs(lat0);

	double long0 = 500;
	double long1, long2;

	if (fabs(I_ht0[1]) < 1e-10)
		long1 = 3.1415926535897932384626433832795 / 2.0;
	else if (I_ht0[2] / I_ht0[1] >= 0)
		long1 = atan(I_ht0[2] / I_ht0[1]);
	else
		long1 = 3.1415926535897932384626433832795 + atan(I_ht0[2] / I_ht0[1]);

	long2 = long1 - 3.1415926535897932384626433832795;

	if (fabs(long_s - long_e) <= 3.1415926535897932384626433832795)
	{
		if (fabs(lat_wd) > 1e-10 && (long1 - long_s) * (long1 - long_e) < 0)
			long_wd = long1;
		else if (fabs(lat_wd) > 1e-10 && (long2 - long_s) * (long2 - long_e) < 0)
			long_wd = long2;
		else
			long_wd = long0;
	}
	else
	{
		if (fabs(lat_wd) > 1e-10 && (long1 - long_s) * (long1 - long_e) < 0)
			long_wd = long1;
		else if (fabs(lat_wd) > 1e-10 && (long2 - long_s) * (long2 - long_e) < 0)
			long_wd = long2;
		else
			long_wd = long0;
	}

	sg = 6378137.0 * acos(I_e[0] * I_v[0] + I_e[1] * I_v[1] + I_e[2] * I_v[2]);       //两点大圆弧距离
	zg = 6378137.0 * (I_ht0[0] * I_v[0] + I_ht0[1] * I_v[1] + I_ht0[2] * I_v[2]);   //到期望航迹侧向距离

	double norm_xg0 = fabs(sg * sg - zg * zg);
	if (norm_xg0 >= 0)
		xg = sqrt(norm_xg0) *TASK_MAIN::sign(I_se[0] * I_ve[0] + I_se[1] * I_ve[1] + I_se[2] * I_ve[2]);
	else
		xg = 0;
	double phi_y = acos(I_ht0[0] * I_n[0] + I_ht0[1] * I_n[1] + I_ht0[2] * I_n[2]);

	if (long_wd > 100.0)
	{
		if (lat_e >= lat_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 - phi_y;
		else if(long_e>=long_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 + phi_y-2.0* 3.1415926535897932384626433832795;
		else
			angle_a= 0.5 * 3.1415926535897932384626433832795 + phi_y;
	}
	else if (fabs(long_m - long_s) < fabs(long_m - long_wd) || (long_m - long_s) * (long_m - long_wd) <= 0)
	{
		if (lat_e >= lat_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 - phi_y;
		else if (long_e >= long_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 + phi_y - 2.0 * 3.1415926535897932384626433832795;
		else
			angle_a = 0.5 * 3.1415926535897932384626433832795 + phi_y;
	}

	double range = 6378137.0 * acos(I_e[0] * I_s[0] + I_e[1] * I_s[1] + I_e[2] * I_s[2]);
	rg = range - xg;

 z_control[0]=sqrt(fabs(sg*sg-zg*zg));
	z_control[1] = zg;  //当前位置与当前航线的侧偏距离
	z_control[2] = sg;  //当前位置与当前航线终点的距离
	z_control[3] = range;  //当前航线的距离
	z_control[4] = angle_a; //航线的方位角，符号应取反


  
//  z_control[0] = sqrt(fabs(sg*sg-zg*zg));   // 当前位置与航线末点的距离？
}

void TASK_MAIN::Func_Dictate()
{
    //导航解算
	RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);	//z_control 0  当前位置到期望航点的垂直距
                                                                        //          1 当前位置与当前航线的侧偏距离 
                                                                        //          2 当前位置到期望航的绝对距离
                                                                        //          3 当段航线长度
                                                                        //          4 期望航向角

	if (z_control[0]<R_Target*tan(angle_Target/2*Deg2Rad)+1&&flag_point<5)   //入弯与撞线判断
	{
        
		if (angle_Target>0)
		{
      //计算转弯圆心的大地方位角
			double turnangle = -z_control[4]*Rad2Deg+90;
      //反结算圆心的经纬度
			Vincent_Z(R_Target,turnangle,Long*Rad2Deg,Lat*Rad2Deg,&Turn_long,&Turn_lat);
      //转弯状态字变1
			flag_zw = 1;
		}
    //转弯成功，导航点加一。
		flag_point +=1;
		Lat_Target =  Lat_Target_arr[flag_point];
		Long_Target = Long_Target_arr[flag_point];
		angle_Target = angle_Target_arr[flag_point];
		Lat_ini = Lat_Target_arr[flag_point-1];
		Long_ini = Long_Target_arr[flag_point-1];

		//PSCI_c =90*Deg2Rad;  //可能没有用  调试用
	}

}
void TASK_MAIN::Func_ADRC_FY()
{
    if ((g_Height<50.0)&&flag_xh==0)     
			{
				if (t_Flight<1.0)    //1s内仅有姿态
				{
					//过渡函数  //从起控点记录的俯仰角过度到15°
          ALFA_cmd = ALPHA_start*(1.0-Transition(0.0,1.0,t_Flight))+15.0*(Transition(0.0,1.0,t_Flight)); //俯仰角起控点--->15
					//ALFA_cmd = 30.0*(1.0-Transition(0.0,1.0,t_Flight))+15.0*(Transition(0.0,1.0,t_Flight)); //俯仰角30--->15
					wz_cmd   =0;//俯仰角速度

					//记录上一步长的值             用于渐变
					time_old = t_Flight;     //记录时间
					VYold0 =VY;              //记录垂向速度
					H_cx =g_Height;          //记录垂向高度
					Hold =  g_Height;        //记录垂向高度
					ALFA_cmd_old0 = ALFA_cmd;//记录俯仰角指令
					
				}
				else if(g_Height<50.0)   
				{	
					
					//3s内过渡	  合理过渡   从第一阶段结束的垂速过渡到3M/S
					Vy_CX = VYold0*(1.0-Transition(time_old,time_old+6.0,t_Flight))+3.0*Transition(time_old,time_old+6.0,t_Flight);   //垂速指令
					H_cx = H_cx+Vy_CX*STEP_20ms;   //频率20ms，50HZ       垂直高度
					ALFA_cmd =  ALFA_cmd_old0* (1.0-Transition(time_old,time_old+1.0,t_Flight)) +
						( -(u_Ymis_n-H_cx)*1.0-(u_VY_n-Vy_CX)*2.0+1.0*(1.0-cos(u_GAMMA/57.3)) )*Transition(time_old,time_old+1.0,t_Flight);
					
					 
					//ALFA_cmd = limit2(ALFA_cmd,THETA_v*Rad2Deg+10,THETA_v*Rad2Deg-2);             //   弹道倾角+10   -2
					
					//保存原始
					VYold = VY;                                                                                                                                                       //纵向速度
					H_cx1 = g_Height;                                                                                                                                       //垂直高度
					time_old1 = t_Flight;                                                                                                                                 //     
					ALFA_cmd_old = ALFA_cmd ;                                                                                                                           
					wz_cmd=0;
					//增加过度过程
				}
				
			}
			else  //((g_Height>30))
			{
				//30M以上巡航  标志位
				flag_xh = 1;   
				//垂速控制指令  3S以内衰减为0
				Vy_CX = VYold*(1.0-Transition(time_old1,time_old1+3.0,t_Flight));  
				//时间
				H_cx1 = H_cx1+Vy_CX*STEP_20ms;
				JF_NY = (u_Ymis_n-H_cx1)*STEP_20ms+JF_NY;

				//1S内衰减	
				ALFA_cmd =ALFA_cmd_old*(1.0-Transition(time_old1,time_old1+1.0,t_Flight))+
					( 1.0-(u_Ymis_n-H_cx1)*1.0-(u_VY_n-Vy_CX)*2.0+1.0*(1.0-cos(u_GAMMA/57.3))-JF_NY*0)*Transition(time_old1,time_old1+1.0,t_Flight);
				

				wz_cmd= 0  ;

			}
			
			//速度控制
			if (V_MIS<30.0&&flag_sd==0)
			{
				Fai_real=1.0;   // 速度为1
			}
			else
			{
				flag_sd =1;  //速度标志位
        JF_ma=JF_ma+(V_MIS-30.0)*0.1;     
        Fai_real =  1.0 - 0.2*(V_MIS-30.0) - 2.0*JF_ma;  //速度PI
        Fai_real =limit2(Fai_real,1.0,0.0);
			}

}

 void TASK_MAIN::Func_ADRC_Gd()
 {
		RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);   
		double u_VZ = sin(z_control[4])*VN_x + cos(z_control[4])*VE_z ;                    
		//方位角
   if (t_Flight<3.0) 
		{
			GAMMAcx = 0.0;
		}
		else if(flag_zw == 0)    //直线控制  PID
		{
			JF_NZ = (z_control[1]-0.0)*STEP_20ms+JF_NZ;      
			GAMMAcx = -z_control[1]*1-u_VZ*2.0-JF_NZ*0.04;  
			JF_Zmis_n = 0 ; 
			t_turn = t_Flight;
		}
		else  
		{
			JF_NZ = 0;  //直线的积分项清零
      //文森反解
			Vincenty_F(Turn_long , Turn_lat, Long*Rad2Deg ,Lat*Rad2Deg,&disturn,&aturn); //反解距离目标点距离，与方位角
			u_VZ = cos(aturn*Deg2Rad)*VN_x + sin(aturn*Deg2Rad)*VE_z ;   //北向动向速度
			JF_Zmis_n = JF_Zmis_n + (R_Target-disturn)* STEP_20ms;     //积分向
			GAMMAcx = -1.0 * (R_Target-disturn) - 2.0*(u_VZ) - JF_Zmis_n * 0.1+30;
			GAMMAcx = limit1(GAMMAcx,45.0);   //滚转角限制45度
      //初始化判断：下一航线侧偏<2或航向角差2度  -->到下一个弯   出弯条件
      //这里正北逆时针-180，180     要转换  (0,360)        
 			if (z_control[1]<2.0||abs(PSCI_t-z_control[4])<2.0*Deg2Rad)
			{
				flag_zw = 0;
			}

 }
 
 };



#endif
