/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: guanbin
 * @Email: 158242357@qq.com
 * @Description:  
 * 本程序的作用是将飞控的模式切换为需要的模式 
 */

#ifndef _SWITCH_FW_MODE_HPP_
#define _SWITCH_FW_MODE_HPP_

#include "../../fixed_wing_lib/syslib.hpp"
#include <fixed_wing_formation_control/Fw_cmd_mode.h>
#include <iostream>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#define SWITCH_FW_MODE_INFO(a) cout << "[SWUTCH_FW_MODE_INFO]:" << a << endl

using namespace std;

class SWITCH_FW_MODE {
public:
  void set_planeID(int id);
  void run();

private:
  int planeID{1};
  string uavID{"uav1/"};

  ros::NodeHandle nh;
  ros::Time begin_time;          /* 记录启控时间 */

  /* 无人机当前状态[包含上锁状态 模式] (从飞控中读取) */
  mavros_msgs::State current_state;
  mavros_msgs::SetMode mode_cmd; /*  模式容器 */
  mavros_msgs::CommandBool  arm_cmd;
  fixed_wing_formation_control::Fw_cmd_mode task_cmd_mode; /* 任务模式切换 */

    mavros_msgs::State my_current_state[5];
  mavros_msgs::SetMode my_mode_cmd[5]; /*  模式容器 */
  mavros_msgs::CommandBool  my_arm_cmd[5];
  fixed_wing_formation_control::Fw_cmd_mode my_task_cmd_mode[5]; /* 任务模式切换 */

  int times_out =50;  /* 次数限制 */
  int counters = 0;     /* 计数器 */
  bool outflag = false; /* 退出标志位 */


  float get_ros_time(ros::Time begin);

  void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
  }
};
#endif
