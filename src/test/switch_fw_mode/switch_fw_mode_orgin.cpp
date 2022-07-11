/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: guanbin
 * @Email: 158242357@qq.com
 * @Description:  
 * 本程序的作用是将飞控的模式切换为需要的模式 
 */

#include "switch_fw_mode.hpp"

/**
 * @Input: int
 * @Output:
 * @Description: 设定当前飞机的ID
 */
void SWITCH_FW_MODE::set_planeID(int id) {
  planeID = id;
  switch (planeID) {
  case 1:
    uavID = "uav1/";
    break;
  case 2:
    uavID = "uav2/";
    break;
  case 3:
    uavID = "uav3/";
    break;
  case 4:
    uavID = "uav4/";
    break;
  }
}

void SWITCH_FW_MODE::run() {

  begin_time = ros::Time::now(); /* 记录启控时间 */
  ros::Rate rate(5.0);           /*  频率 [5Hz] */

  /* 【订阅】无人机当前状态 */
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
      add2str(uavID, "mavros/state"), 10, &SWITCH_FW_MODE::state_cb, this);

  /* 【订阅】编队控制器期望状态 */
  ros::Publisher task_cmd_mode_pub =
      nh.advertise<fixed_wing_formation_control::Fw_cmd_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_cmd_mode"), 10);

  /* 【服务】 修改锁定状态 */
  ros::ServiceClient arm_client =
      nh.serviceClient<mavros_msgs::CommandBool>(add2str(uavID, "mavros/cmd/arming"));

  /* 【服务】 修改系统模式 */
  ros::ServiceClient set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>(add2str(uavID, "mavros/set_mode"));

  while (ros::ok() && (!outflag)) {

    float current_time = get_ros_time(begin_time); /* 当前时间 */

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>SWITCH_FW_MODE<<<<<<<<<<<<<<<<<<<<<<<<<<"<< endl;
    SWITCH_FW_MODE_INFO("本机ID:"<<planeID);
    SWITCH_FW_MODE_INFO("当前时刻："<<current_time);
    SWITCH_FW_MODE_INFO("当前模式 : [ " << current_state.mode << " ]");
    SWITCH_FW_MODE_INFO("更换模式：0 disarm, 1 offboard_takeoff, 2 offboard_formation, 3 offboard_mission, 4 offboard_land, 5 offboard_idel,6 switch_shape, 999 protect");
    SWITCH_FW_MODE_INFO("更换模式：11 auto.take_off, 12 auto.mission, 13 auto.land, 14 out");


    int mode_type = 0;    /* 模式类型 */
    int formation_shape=0;
    cin >> mode_type;


      if(mode_type==0)
      {
        arm_cmd.request.value=true;
        arm_client.call(arm_cmd);
      }else if (mode_type == 1) {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_take_off = true;                           /*need_takeoff*/
      task_cmd_mode.need_formation = false;
      task_cmd_mode.need_mission = false;
      task_cmd_mode.need_land = false;
      task_cmd_mode.need_idel = false;
      task_cmd_mode.need_protected=false;
      task_cmd_mode_pub.publish(task_cmd_mode);
    } else if (mode_type == 2) {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_take_off = false;                        /*need_formation*/
      task_cmd_mode.need_formation = true;
      task_cmd_mode.need_mission = false;
      task_cmd_mode.need_land = false;
      task_cmd_mode.need_idel = false;
      task_cmd_mode.need_protected=false;
      task_cmd_mode_pub.publish(task_cmd_mode);
    } else if (mode_type == 3) {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_take_off = false;                        /*need_mission*/
      task_cmd_mode.need_formation = false;
      task_cmd_mode.need_mission = true;
      task_cmd_mode.need_land = false;
      task_cmd_mode.need_idel = false;
      task_cmd_mode.need_protected=false;
      task_cmd_mode_pub.publish(task_cmd_mode);
    } else if(mode_type==4)
    {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_take_off = false;                            /*need_land*/
      task_cmd_mode.need_formation = false;
      task_cmd_mode.need_mission = false;
      task_cmd_mode.need_land = true;
      task_cmd_mode.need_idel = false;
      task_cmd_mode.need_protected=false;
      task_cmd_mode_pub.publish(task_cmd_mode);
    } else if(mode_type==5)
    {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_take_off = false;                            /*need_idel*/
      task_cmd_mode.need_formation = false;
      task_cmd_mode.need_mission = false;
      task_cmd_mode.need_land = false;
      task_cmd_mode.need_idel = true;
      task_cmd_mode.need_protected=false;
      task_cmd_mode_pub.publish(task_cmd_mode);
    }else if(mode_type==6)
    {
      SWITCH_FW_MODE_INFO("选择队形：0 One_column, 1 Triangle, 2 One_row" );
      cin>>formation_shape;                                                              /*switch_shape*/
      task_cmd_mode.swarm_shape=formation_shape;
      task_cmd_mode_pub.publish(task_cmd_mode);
    }else if(mode_type==999)
    {
      mode_cmd.request.custom_mode = "OFFBOARD";
      task_cmd_mode.need_take_off = false;                            /*need_protected*/
      task_cmd_mode.need_formation = false;
      task_cmd_mode.need_mission = false;
      task_cmd_mode.need_land = false;
      task_cmd_mode.need_idel = false;
      task_cmd_mode.need_protected=true;
      task_cmd_mode_pub.publish(task_cmd_mode);
    }else if(mode_type==11)
    {
      mode_cmd.request.custom_mode = "AUTO.TAKEOFF";/*auto.takeoff*/
    }else if(mode_type==12)     
    {
      mode_cmd.request.custom_mode = "AUTO.MISSION";/*auto.mission*/
    }else if(mode_type==13)
    {
      mode_cmd.request.custom_mode = "AUTO.LAND";/*auto.land*/
    }else if(mode_type==14)
    {
     outflag = true;
      break;
    }

    /*  如果当前模式与设定模式不一致，切换，（有次数限制） */
    while ((mode_cmd.request.custom_mode != current_state.mode) &&
           (counters <= times_out)) {

      counters++;
      set_mode_client.call(mode_cmd);

    }

    ros::spinOnce(); /*  挂起一段时间，等待切换结果 */
    rate.sleep();

    counters = 0;
  }
}

float SWITCH_FW_MODE::get_ros_time(ros::Time begin) {
  ros::Time time_now = ros::Time::now();
  float currTimeSec = time_now.sec - begin.sec;
  float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
  return (currTimeSec + currTimenSec);
}
