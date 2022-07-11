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
//#include "fixed_wing_formation_control/Fw_current_mode.h"
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
class TASK_MAIN: public FORMATION_CONTROLLER
{
private:
    int planeID{1};                                                   /*飞机编号*/
    string uavID{ "uav1/"}; /* 本机编号，用于命名空间 */

    string leaderID{ "uav0/"};/* 领机编号，用于命名空间 */

    void print_data(const struct ABS_FORMATION_CONTROLLER ::_s_fw_states *p); /*打印数据*/

    ros::NodeHandle nh; /*ros句柄*/
    ros::Time begin_time;
    float current_time;
    double ground_vel;
    float get_ros_time(ros::Time begin); /*获取当前时间*/

    ros::Subscriber fwmonitor_sub;               /*【订阅】来自fw_monitor任务状态的flags*/
    ros::Subscriber fw_states_sub;               /*【订阅】来自pack_fw_states固定翼全部状态量*/
    ros::Subscriber leader_states_sub;           /*【订阅】来自通讯节点，或者视觉节点领机全部状态量*/
    ros::Subscriber fw_cmd_mode_sub;             /*【订阅】来自commander状态机的控制指令*/
    ros::Subscriber way_points_sub;

    ros::Publisher formation_control_states_pub; /*【发布】编队控制状态量*/
    ros::Publisher fw_cmd_pub;                   /*【发布】飞机四通道控制量*/
    ros::Publisher fw_current_mode_pub;          /*【发布】飞机当前处于的任务状态*/

    fixed_wing_formation_control::FWstates fwstates;                                 /*自定义--飞机打包的全部状态*/
    fixed_wing_formation_control::FWcmd fw_4cmd;                                     /*自定义--飞机四通道控制量*/
    fixed_wing_formation_control::Leaderstates leaderstates;                         /*自定义--领机状态*/
    fixed_wing_formation_control::Formation_control_states formation_control_states; /*自定义--编队控制状态量*/
    fixed_wing_formation_control::Fwmonitor fwmonitor_flag;                          /*自定义--任务状态的flags*/
    fixed_wing_formation_control::Fw_cmd_mode fw_cmd_mode;                           /*自定义--来自commander的命令飞机模式*/
    fixed_wing_formation_control::Fw_current_mode fw_current_mode;                   /*自定义--要发布的飞机当前任务模式*/
    void Navigation_solution();
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
    int start_control=1;
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
     
    double Transition_function(double t_begin,double t_current,double t_end);
    void control_takeoff();                     /*起飞控制主函数*/
    void control_land();                     /*降落控制主函数*/
    float  time_test{0.0};
    void control_position(Point waypoint_sp); /*位置控制主函数*/
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
    float pitch_cmd;
    bool hv_contol_takeoff{false};
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
    };
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


int POINT_NUM_QGC{0};
double waypoint_qgc[MAX_POINT_NUM][2];
void get_waypoint_from_qgc_to_rk3399(const mavros_msgs::WaypointList points);
public:
  void run();
  void set_planeID(int id);
};

#endif
