#ifndef _FORMATION_COMMANDER_HPP_
#define _FORMATION_COMMANDER_HPP_
#include "../../fixed_wing_lib/syslib.hpp"
#include <fixed_wing_formation_control/Formation_control_commander.h>
#include <iostream>
#include <ros/ros.h>
#define SWITCH_FW_MODE_INFO(a) cout << "[SWUTCH_FW_MODE_INFO]:" << a << endl
using namespace std;
class FORMATION_COMMANDER{
    public:
    void run();

    private:
      ros::NodeHandle nh;
      ros::Time begin_time;          /* 记录启控时间 */
      float get_ros_time(ros::Time begin);
}
#endif