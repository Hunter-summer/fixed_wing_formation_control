/*
 * @------------------------------------------1: 1------------------------------------------@
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
  case 0:
    uavID = "uav0/";
    break;    
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
 double t_test=0;
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


 ros::Publisher my_task_cmd_mode_pub[5];
 ros::ServiceClient my_set_mode_client[5];
 ros::ServiceClient my_arm_client[5];
 ros::Subscriber my_state_sub[5];
  for(int i=0;i<5;i++)
  {
    my_task_cmd_mode_pub[i]=nh.advertise<fixed_wing_formation_control::Fw_cmd_mode>( "uav"+std::to_string(i)+"/fixed_wing_formation_control/fw_cmd_mode", 10);
    my_set_mode_client[i]=nh.serviceClient<mavros_msgs::SetMode>("uav"+std::to_string(i)+"/mavros/set_mode");
    my_arm_client[i] =nh.serviceClient<mavros_msgs::CommandBool>("uav"+std::to_string(i)+ "/mavros/cmd/arming");
     my_state_sub[i] = nh.subscribe<mavros_msgs::State>("uav"+std::to_string(i)+ "/mavros/state", 10, &SWITCH_FW_MODE::state_cb, this);
  }
  while (ros::ok() && (!outflag)) {

    float current_time = get_ros_time(begin_time); /* 当前时间 */

    cout << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>SWITCH_FW_MODE<<<<<<<<<<<<<<<<<<<<<<<<<<"<< endl;
    SWITCH_FW_MODE_INFO("本机ID:"<<planeID);
    SWITCH_FW_MODE_INFO("当前时刻："<<current_time);
    //SWITCH_FW_MODE_INFO("当前模式 : [ " << current_state.mode << " ]");
    SWITCH_FW_MODE_INFO("更换模式: 0 解锁, 10 上锁, 999 启控rk3399, 1 测试程序, 2 编队formation, 3 空位, 5 开伞");
    //SWITCH_FW_MODE_INFO("更换模式：11 auto.take_off, 12 auto.mission, 13 auto.land, 14 out");


    int mode_type = 0;    /* 模式类型 */
    int formation_shape=0;
    cin >> mode_type;


      if(mode_type==0)
      {
        for(int i=0;i<5;i++)
        {
          my_arm_cmd[i].request.value=true;
          my_arm_client[i].call(my_arm_cmd[i]);
        }
      }
      else if (mode_type==10)
      {
          for(int i=0;i<5;i++)
          {
            my_arm_cmd[i].request.value=false;
            my_arm_client[i].call(my_arm_cmd[i]);
          }
      }
      else if (mode_type == 1) {

          for(int i=0;i<5;i++)
          {
            my_mode_cmd[i].request.custom_mode="OFFBOARD";
            my_task_cmd_mode[i].need_take_off=true;
            my_task_cmd_mode[i].need_formation = false;
            my_task_cmd_mode[i].need_mission = false;
            my_task_cmd_mode[i].need_land = false;
            my_task_cmd_mode[i].need_idel = false;
            my_task_cmd_mode[i].need_protected=false;
            // SWITCH_FW_MODE_INFO("输入油门" );
            // cin>>t_test;                                                              /*switch_shape*/
            // my_task_cmd_mode[i].t_test=t_test;
            my_task_cmd_mode_pub[i].publish(my_task_cmd_mode[i]);
          }
        
      // mode_cmd.request.custom_mode = "OFFBOARD";
      // task_cmd_mode.need_take_off = true;                           /*need_takeoff*/
      // task_cmd_mode.need_formation = false;
      // task_cmd_mode.need_mission = false;
      // task_cmd_mode.need_land = false;
      // task_cmd_mode.need_idel = false;
      // task_cmd_mode.need_protected=false;
      // task_cmd_mode_pub.publish(task_cmd_mode);
    } else if (mode_type == 2) {
          //编队模式始于从机1，不对领机进行编队
          for(int i=1;i<5;i++)
          {
            my_mode_cmd[i].request.custom_mode="OFFBOARD";
            my_task_cmd_mode[i].need_take_off=false;
            my_task_cmd_mode[i].need_formation = true;
            my_task_cmd_mode[i].need_mission = false;
            my_task_cmd_mode[i].need_land = false;
            my_task_cmd_mode[i].need_idel = false;
            my_task_cmd_mode[i].need_protected=false;
            my_task_cmd_mode_pub[i].publish(my_task_cmd_mode[i]);
            
          }
    } else if (mode_type == 3) {
         for(int i=0;i<5;i++)
          {
            my_mode_cmd[i].request.custom_mode="OFFBOARD";
            my_task_cmd_mode[i].need_take_off=false;
            my_task_cmd_mode[i].need_formation = false;
            my_task_cmd_mode[i].need_mission = true;
            my_task_cmd_mode[i].need_land = false;
            my_task_cmd_mode[i].need_idel = false;
            my_task_cmd_mode[i].need_protected=false;
            my_task_cmd_mode_pub[i].publish(my_task_cmd_mode[i]);
            
          }
      // mode_cmd.request.custom_mode = "OFFBOARD";
      // task_cmd_mode.need_take_off = false;                        /*need_mission*/
      // task_cmd_mode.need_formation = false;
      // task_cmd_mode.need_mission = true;
      // task_cmd_mode.need_land = false;
      // task_cmd_mode.need_idel = false;
      // task_cmd_mode.need_protected=false;
      // task_cmd_mode_pub.publish(task_cmd_mode);
    } else if(mode_type==4)
    {
          for(int i=0;i<5;i++)
          {
            my_mode_cmd[i].request.custom_mode="OFFBOARD";
            my_task_cmd_mode[i].need_take_off=false;
            my_task_cmd_mode[i].need_formation = false;
            my_task_cmd_mode[i].need_mission = false;
            my_task_cmd_mode[i].need_land = true;
            my_task_cmd_mode[i].need_idel = false;
            my_task_cmd_mode[i].need_protected=false;
            my_task_cmd_mode_pub[i].publish(my_task_cmd_mode[i]);
            
          }
      // mode_cmd.request.custom_mode = "OFFBOARD";
      // task_cmd_mode.need_take_off = false;                            /*need_land*/
      // task_cmd_mode.need_formation = false;
      // task_cmd_mode.need_mission = false;
      // task_cmd_mode.need_land = true;
      // task_cmd_mode.need_idel = false;
      // task_cmd_mode.need_protected=false;
      // task_cmd_mode_pub.publish(task_cmd_mode);
    } else if(mode_type==5)
    {
          for(int i=0;i<5;i++)
          {
            my_mode_cmd[i].request.custom_mode="OFFBOARD";
            my_task_cmd_mode[i].need_take_off=false;
            my_task_cmd_mode[i].need_formation = false;
            my_task_cmd_mode[i].need_mission = false;
            my_task_cmd_mode[i].need_land = false;
            my_task_cmd_mode[i].need_idel = true;
            my_task_cmd_mode[i].need_protected=false;
            my_task_cmd_mode_pub[i].publish(my_task_cmd_mode[i]);
          }
      // mode_cmd.request.custom_mode = "OFFBOARD";
      // task_cmd_mode.need_take_off = false;                            /*need_idel*/
      // task_cmd_mode.need_formation = false;
      // task_cmd_mode.need_mission = false;
      // task_cmd_mode.need_land = false;
      // task_cmd_mode.need_idel = true;
      // task_cmd_mode.need_protected=false;
      // task_cmd_mode_pub.publish(task_cmd_mode);
    }else if(mode_type==6)
    {
      SWITCH_FW_MODE_INFO("选择队形: 0 One_column, 1 Triangle, 2 One_row" );
      cin>>formation_shape;                                                              /*switch_shape*/
      for(int i=0;i<5;i++)
      {
        my_task_cmd_mode[i].swarm_shape=formation_shape;
        my_task_cmd_mode_pub[i].publish(my_task_cmd_mode[i]);
      }

    }else if(mode_type==999)
    {
        for(int i=0;i<5;i++)
        {
          my_mode_cmd[i].request.custom_mode="OFFBOARD";
          my_task_cmd_mode[i].need_take_off=false;
          my_task_cmd_mode[i].need_formation = false;
          my_task_cmd_mode[i].need_mission = false;
          my_task_cmd_mode[i].need_land = false;
          my_task_cmd_mode[i].need_idel = false;
          my_task_cmd_mode[i].need_protected=true;
          my_task_cmd_mode_pub[i].publish(my_task_cmd_mode[i]);
        }
      // mode_cmd.request.custom_mode = "OFFBOARD";
      // task_cmd_mode.need_take_off = false;                            /*need_protected*/
      // task_cmd_mode.need_formation = false;
      // task_cmd_mode.need_mission = false;
      // task_cmd_mode.need_land = false;
      // task_cmd_mode.need_idel = false;
      // task_cmd_mode.need_protected=true;
      // task_cmd_mode_pub.publish(task_cmd_mode);
    }else if(mode_type==11)
    {
    //  mode_cmd.request.custom_mode = "AUTO.TAKEOFF";/*auto.takeoff*/
    for(int i=0;i<5;i++)
    {
      my_mode_cmd[i].request.custom_mode = "AUTO.TAKEOFF";
    }
    
    }else if(mode_type==12)     
    {
      for(int i=0;i<5;i++)
    {
      my_mode_cmd[i].request.custom_mode = "AUTO.MISSION";
    }
      //mode_cmd.request.custom_mode = "AUTO.MISSION";/*auto.mission*/
    }else if(mode_type==13)
    {
        for(int i=0;i<5;i++)
        {
          my_mode_cmd[i].request.custom_mode = "AUTO.LAND";
        }
      //mode_cmd.request.custom_mode = "AUTO.LAND";/*auto.land*/
    }else if(mode_type==14)
    {
     outflag = true;
      break;
    }

    /*  如果当前模式与设定模式不一致，切换，（有次数限制） */
    for(int i=0;i<5;i++)
    {
         // while ((my_mode_cmd[i].request.custom_mode != my_current_state[i].mode) &&
               // (counters <= times_out)) {
                  //counters++;
                  my_set_mode_client[i].call(my_mode_cmd[i]);

//}
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
