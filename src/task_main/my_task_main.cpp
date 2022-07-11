/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: guanbin
 * @Email: 158242357@qq.com
 * @Description:  
 * 本程序的作用是将飞控的模式切换为需要的模式 
 */

#include "task_main.hpp"

/**
 * @Input: int
 * @Output: 
 * @Description: 设定当前飞机的ID
 */
void TASK_MAIN::set_planeID(int id) {
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


/**
 * @Input: WaypointList
 * @Output: 
 * @Description: //用于打印航迹点的子函数
 */
void TASK_MAIN::printwaypoint(const mavros_msgs::WaypointList points)
{
    cout<<"count:"<<points.waypoints.size()<<endl;
    for (size_t i = 0; i < points.waypoints.size(); i++)
    {

        cout<<i<<" "<<points.waypoints[i].command<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].x_lat<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].y_long<<" ";
        cout<<fixed<< setprecision(10)<<points.waypoints[i].z_alt<<endl;
    }
     cout<<fixed<< setprecision(10)<<points.current_seq<<endl;

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void TASK_MAIN::get_waypoint_from_qgc_to_rk3399(const mavros_msgs::WaypointList points)
{

     cout<<"count:"<<points.waypoints.size()<<endl;
    for (size_t i = 0; i < points.waypoints.size(); i++)
    {
        //points.waypoints[i].command::
        if(points.waypoints[i].command==16)//普通航路点
        {
            POINT_NUM_QGC++;
            waypoint_qgc[POINT_NUM_QGC][0]=points.waypoints[i].x_lat;
            waypoint_qgc[POINT_NUM_QGC][1]=points.waypoints[i].y_long;
        }
    }
}
/**
 * @Input: ros::Time begin
 * @Output: float time_now
 * @Description: 获取当前时间
 */
float TASK_MAIN::get_ros_time(ros::Time begin)
{
    ros::Time time_now = ros::Time::now();
    float currTimeSec = time_now.sec - begin.sec;
    float currTimenSec = time_now.nsec / 1e9 - begin.nsec / 1e9;
    return (currTimeSec + currTimenSec);
}


void TASK_MAIN::fw_state_cb(const fixed_wing_formation_control::FWstates::ConstPtr &msg)
{
    fwstates = *msg;
}
void TASK_MAIN::leader_states_cb(const fixed_wing_formation_control::Leaderstates::ConstPtr &msg)
{
    leaderstates = *msg;
}
void TASK_MAIN::fw_fwmonitor_cb(const fixed_wing_formation_control::Fwmonitor::ConstPtr &msg)
{
    fwmonitor_flag = *msg;
}

void TASK_MAIN::fw_cmd_mode_cb(const fixed_wing_formation_control::Fw_cmd_mode::ConstPtr &msg)
{
    fw_cmd_mode = *msg;
}

void TASK_MAIN::waypoints_cb(const mavros_msgs::WaypointList::ConstPtr& msg)
{
    current_waypoints = *msg;
}
/**
 * @Input: void
 * @Output: void
 * @Description: ros的订阅发布声明函数
 */
void TASK_MAIN::ros_sub_pub() {

  fw_states_sub = nh.subscribe /* 【订阅】固定翼全部状态量 */
                  <fixed_wing_formation_control::FWstates>(
                      add2str(uavID, "fixed_wing_formation_control/fw_states"),
                      10, &TASK_MAIN::fw_state_cb, this);

  leader_states_sub =
      nh.subscribe /* 【订阅】领机信息 */
      <fixed_wing_formation_control::Leaderstates>(
          add2str(leaderID, "fixed_wing_formation_control/leader_states"), 10,
          &TASK_MAIN::leader_states_cb, this);

  fwmonitor_sub =
      nh.subscribe /* 【订阅】监控节点飞机以及任务状态 */
      <fixed_wing_formation_control::Fwmonitor>(
          add2str(uavID, "fixed_wing_formation_control/fwmonitor_flags"), 10,
          &TASK_MAIN::fw_fwmonitor_cb, this);

  fw_cmd_mode_sub =
      nh.subscribe /* 【订阅】commander指定模式 */
      <fixed_wing_formation_control::Fw_cmd_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_cmd_mode"), 10,
          &TASK_MAIN::fw_cmd_mode_cb, this);

way_points_sub=
      nh.subscribe /* 【订阅】waypoint */
        <mavros_msgs::WaypointList>(
          add2str(uavID, "mavros/mission/waypoints"), 10,
          &TASK_MAIN::waypoints_cb, this);

  fw_cmd_pub = nh.advertise /* 【发布】固定翼四通道控制量 */
               <fixed_wing_formation_control::FWcmd>(
                   add2str(uavID, "fixed_wing_formation_control/fw_cmd"), 10);

  formation_control_states_pub =
      nh.advertise /* 【发布】编队控制器状态 */
      <fixed_wing_formation_control::Formation_control_states>(
          add2str(uavID,
                  "fixed_wing_formation_control/formation_control_states"),
          10);

  fw_current_mode_pub =
      nh.advertise /* 【发布】任务进所处阶段 */
      <fixed_wing_formation_control::Fw_current_mode>(
          add2str(uavID, "fixed_wing_formation_control/fw_current_mode"), 10);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 编队状态值发布，编队位置误差（机体系和ned），速度误差以及期望空速，gps，期望地速
 */
void TASK_MAIN::fw_state_pub() {
   fw_4cmd.Dis_y=Dis_y;
   fw_4cmd.Dis=Dis;
   fw_4cmd.DYr=DYr;
   fw_cmd_pub.publish(fw_4cmd); 
}
/**
 * @Input: void
 * @Output: void
 * @Description: 编队状态值发布，编队位置误差（机体系和ned），速度误差以及期望空速，gps，期望地速
 */
void TASK_MAIN::formation_states_pub() {
  formation_control_states.planeID = planeID;

  /* 本部分是关于编队的从机的自己与期望值的误差以及领从机偏差的赋值 */
  formation_control_states.err_P_N = formation_error.P_N;
  formation_control_states.err_P_E = formation_error.P_E;
  formation_control_states.err_P_D = formation_error.P_D;
  formation_control_states.err_P_NE = formation_error.P_NE;

  formation_control_states.err_PXb = formation_error.PXb;
  formation_control_states.err_PYb = formation_error.PYb;
  formation_control_states.err_PZb = formation_error.PZb;
  formation_control_states.err_VXb = formation_error.VXb;
  formation_control_states.err_VYb = formation_error.VYb;
  formation_control_states.err_VZb = formation_error.VZb;
  formation_control_states.led_fol_vxb = formation_error.led_fol_vxb;
  formation_control_states.led_fol_vyb = formation_error.led_fol_vyb;
  formation_control_states.led_fol_vzb = formation_error.led_fol_vzb;

  formation_control_states.err_PXk = formation_error.PXk;
  formation_control_states.err_PYk = formation_error.PYk;
  formation_control_states.err_PZk = formation_error.PZk;
  formation_control_states.err_VXk = formation_error.VXk;
  formation_control_states.err_VYk = formation_error.VYk;
  formation_control_states.err_VZk = formation_error.VZk;
  formation_control_states.led_fol_vxk = formation_error.led_fol_vxk;
  formation_control_states.led_fol_vyk = formation_error.led_fol_vyk;
  formation_control_states.led_fol_vzk = formation_error.led_fol_vzk;

  formation_control_states.led_fol_eta = formation_error.led_fol_eta;
  formation_control_states.eta_deg = rad_2_deg(formation_error.led_fol_eta);

  /* 本部分关于从机的期望值的赋值 */
  formation_control_states.sp_air_speed = formation_sp.air_speed;
  formation_control_states.sp_altitude = formation_sp.altitude;
  formation_control_states.sp_ground_speed = formation_sp.ground_speed;
  formation_control_states.sp_latitude = formation_sp.latitude;
  formation_control_states.sp_longitude = formation_sp.longitude;
  formation_control_states.sp_ned_vel_x = formation_sp.ned_vel_x;
  formation_control_states.sp_ned_vel_y = formation_sp.ned_vel_y;
  formation_control_states.sp_ned_vel_z = formation_sp.ned_vel_z;
  formation_control_states.sp_relative_alt = formation_sp.relative_alt;

  /* 发布编队控制器控制状态 */
  formation_control_states_pub.publish(formation_control_states);
}

/**
 * @Input: void
 * @Output: void
 * @Description: 更新本机与邻机飞机状态参数
 */
void TASK_MAIN::update_fwsates()
{
     fw_col_mode_current = fwstates.control_mode;
    /* 领机状态赋值 */
    leader_states.air_speed = leaderstates.airspeed;

    leader_states.altitude = leaderstates.altitude;
    leader_states.latitude = leaderstates.latitude;
    leader_states.longitude = leaderstates.longitude;
    leader_states.relative_alt = leaderstates.relative_alt;

    leader_states.global_vel_x = leaderstates.global_vel_x;
    leader_states.global_vel_y = leaderstates.global_vel_y;
    leader_states.global_vel_z = leaderstates.global_vel_z;

    leader_states.ned_vel_x = leaderstates.ned_vel_x;
    leader_states.ned_vel_y = leaderstates.ned_vel_y;
    leader_states.ned_vel_z = leaderstates.ned_vel_z;

    leader_states.pitch_angle = leaderstates.pitch_angle;
    leader_states.roll_angle = leaderstates.roll_angle;
    leader_states.yaw_angle = leaderstates.yaw_angle;
    leader_states.yaw_valid = false; /* 目前来讲，领机的yaw不能直接获得 */

    /* 从机状态赋值 */
    thisfw_states.flight_mode = fwstates.control_mode;

    thisfw_states.air_speed = fwstates.air_speed;
    thisfw_states.in_air = fwstates.in_air;

    thisfw_states.altitude = fwstates.altitude;
    thisfw_states.altitude_lock = true; /* 保证TECS */
    thisfw_states.in_air = true;        /* 保证tecs */
    thisfw_states.latitude = fwstates.latitude;
    thisfw_states.longitude = fwstates.longitude;

    thisfw_states.relative_alt = fwstates.relative_alt;

    thisfw_states.ned_vel_x = fwstates.ned_vel_x;
    thisfw_states.ned_vel_y = fwstates.ned_vel_y;
    thisfw_states.ned_vel_z = fwstates.ned_vel_z;

    thisfw_states.global_vel_x = fwstates.global_vel_x;
    thisfw_states.global_vel_y = fwstates.global_vel_y;
    thisfw_states.global_vel_z = fwstates.global_vel_z;

    thisfw_states.ground_speed=fwstates.ground_speed;
    
    thisfw_states.pitch_angle = fwstates.pitch_angle;
    thisfw_states.roll_angle = fwstates.roll_angle;
    thisfw_states.yaw_angle = fwstates.yaw_angle;
    thisfw_states.att_quat[0] = fwstates.att_quater.w;
    thisfw_states.att_quat[1] = fwstates.att_quater.x;
    thisfw_states.att_quat[2] = fwstates.att_quater.y;
    thisfw_states.att_quat[3] = fwstates.att_quater.z;
    quat_2_rotmax(thisfw_states.att_quat, thisfw_states.rotmat);

    thisfw_states.body_acc[0] = fwstates.body_acc_x;
    thisfw_states.body_acc[1] = fwstates.body_acc_y;
    thisfw_states.body_acc[2] = fwstates.body_acc_z;
    matrix_plus_vector_3(thisfw_states.ned_acc, thisfw_states.rotmat, thisfw_states.body_acc);

    thisfw_states.wind_estimate_x = fwstates.wind_estimate_x;
    thisfw_states.wind_estimate_y = fwstates.wind_estimate_y;
    thisfw_states.wind_estimate_z = fwstates.wind_estimate_z;

    //Navigation_solution();
}
/**
 * @Input: void
 * @Output: void
 * @Description: 编队控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */
void TASK_MAIN::control_formation()
{

    /* 设定编队形状 */
    formation_controller.set_formation_type(planeID,fw_cmd_mode.swarm_shape,5,3);

    /* 设定前向编队混合参数 */
    formation_controller.set_mix_Xerr_params(mix_Xerr_params);

    /* 设定侧向编队混合参数 */
    formation_controller.set_mix_Yerr_params(mix_Yerr_params);

    /* 模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！ */
    if (fw_col_mode_current != fw_col_mode_last)
    {
       formation_controller.reset_formation_controller();
    }
    /* 更新飞机状态，领机状态 */
    formation_controller.update_led_fol_states(&leader_states, &thisfw_states);
    /* 编队控制 */
    formation_controller.control_formation();

    /* 获得最终控制量 */
    formation_controller.get_formation_4cmd(formation_cmd);
    /* 获得编队控制期望值 */
    formation_controller.get_formation_sp(formation_sp);
    /* 获得编队误差信息 */
    formation_controller.get_formation_error(formation_error);

    /* 控制量赋值 */
    fw_4cmd.throttle_sp = formation_cmd.thrust;
    fw_4cmd.roll_angle_sp = formation_cmd.roll;
    fw_4cmd.pitch_angle_sp = formation_cmd.pitch;
    fw_4cmd.yaw_angle_sp = formation_cmd.yaw;

   // fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */
    formation_states_pub();      /* 发布编队控制器状态 */

    fw_col_mode_last = fw_col_mode_current; /* 上一次模式的纪录 */
}
/**
 * @Input: void
 * @Output: void
 * @Description: 起飞控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */

/**
 * @Input: void
 * @Output: void
 * @Description: 调用滤波器对输入的飞机原始状态进行滤波
 */
void TASK_MAIN::filter_states()
{
    fw_states_f=thisfw_states;
  if (use_the_filter)
  {
    fw_states_f.global_vel_x =
        fw_filter_gol_vx.one_order_filter(thisfw_states.global_vel_x);
    fw_states_f.global_vel_y =
        fw_filter_gol_vy.one_order_filter(thisfw_states.global_vel_y);
  }
}

//平滑处理函数
double TASK_MAIN::Transition_function(double t_begin,double t_current,double t_end)
{
    if(t_current<t_begin)
    {
        return 0;
    }
   else if(t_begin<t_current<t_end)
   {
       return 0.5*(sin(((t_current-t_begin)/(t_end-t_begin)-0.5)*PI)+1);
   }
   else 
    return 1;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 起飞流程，221
 */
void TASK_MAIN::control_takeoff()
{
 
   if(start_control==1)
   {

       //一定要等待航点装订好，因为从qgc获取航点会有一定延迟
        get_waypoint_from_qgc_to_rk3399(current_waypoints);
        //记录起控时间
       start_time=current_time;
       //记录初始角度
       roll_0=thisfw_states.roll_angle;
       pitch_0=thisfw_states.pitch_angle;
       yaw_0=thisfw_states.yaw_angle;
       //重置takeoff
       start_control=0;
       //重置积分器
       rest_pid_speed=true;
       rest_pid_height=true;
       rest_pid_takeoff_speed=true;
       waypoint_next=1;

        //弹射起飞
        //start_control=1时，输出弹射架的控制指令
        //连续三次判断Nx>10g
        // if(thisfw_states.body_acc[0]>10*9.81)
        // {
        //     cnt++;
        //     if(cnt==3)
        //     {
        //         cnt==0;
        //         t0 = current_time;        
        //     }
        // }
        // t1 = t0+0.5;

       t1=0;//启控零点
       
       t2=t1+1.5;
       //输入转弯半径、速度
       Rcmd=80;
       RVcmd=5;
       
       TASK_MAIN_INFO("启动控制");
   }
   //更新时间
    t=current_time-start_time;
    last_height=thisfw_states.relative_alt;
    if(t<t2)//启控点开始以30度飞1.5s
    {
            throttle_cmd= 1;
            roll_cmd=0;
            pitch_cmd=30;
            yaw_cmd=0;
          TASK_MAIN_INFO("启控点开始以30度飞1.5s");
    }
    else if(thisfw_states.relative_alt<=80)//在高度小于80m内从30度过渡到15度，时刻是5s,后面一直保持15度
    {

        if(t>=t2&&t<=t2+5)
        {
            pitch_cmd=30*(1-Transition_function(t2,t,t2+5))+15*Transition_function(t2,t,t2+5);
        }
        else
        {
            pitch_cmd=15;
        }

        throttle_cmd = 0.6;
        roll_cmd=0;
        yaw_cmd=0;
        //给垂速控制阶段赋初值
        Hcx=80;
        Vy0=-thisfw_states.ned_vel_z;
        TASK_MAIN_INFO("30度过渡到15度");
    }
    else//进入垂直速度控制    relative_alt>80
    {
            //垂速指令 Vy_CX = V_y0 * exp(-(H-80)/20*2)  vy0为NED的-z方向的速度，指向天
            Vycx=Vy0*exp(-(thisfw_states.relative_alt-80)/5);
            //高度指令 +80？
            Hcx+=Vycx*_dt;
            pitch_cmd=-(thisfw_states.relative_alt-Hcx)*1-(-thisfw_states.ned_vel_z-Vycx)*3.5+15;
            TASK_MAIN_INFO("进入垂直速度控制");
    }

     //在这个过程如果速度超过了18（小蜜蜂：28），开始对速度进行控制
    if(fw_states_f.ground_speed>18)
    {
                if(rest_pid_takeoff_speed)
            {
                rest_pid_takeoff_speed = false;
                takeoff_speed_ctl.reset_pid();
                cout<<"rest"<<endl;
                takeoff_speed_ctl.init_pid(0.3,0.01,0);
                TASK_MAIN_INFO("进入速度控制");
            }
        //上面全是全油门，如果速度达到，更新控制油门  保持20m/s
            throttle_cmd= takeoff_speed_ctl.pid_anti_saturated(20-thisfw_states.ground_speed,1,0,"throttle");
    }

   if(thisfw_states.relative_alt>100)
   {
       cnt++;
       if(cnt==3)
       {
           cnt==0;
           fw_cmd_mode.need_take_off=false;
           fw_cmd_mode.need_mission=true;
           t4=current_time-start_time;//记录巡航段启动控制点
           Dis=999;//防止已进入就切掉一个航点
           waypoint_qgc[0][0]=thisfw_states.latitude;
            waypoint_qgc[0][1]=thisfw_states.longitude;
 
       }
   }
        fw_4cmd.roll_angle_sp =deg_2_rad(roll_cmd);
        fw_4cmd.pitch_angle_sp =deg_2_rad(pitch_cmd);
        fw_4cmd.yaw_angle_sp =deg_2_rad(yaw_cmd);
         fw_4cmd.throttle_sp=throttle_cmd;

}


/**
 * @Input: void
 * @Output: void
 * @Description: 计算当前点与目标点的误差向量
 */
Point TASK_MAIN::get_plane_to_sp_vector(Point origin, Point target)
{
  Point out(deg_2_rad((target.x - origin.x)), deg_2_rad((target.y - origin.y) * cos(deg_2_rad(origin.x))));

  return out * double(CONSTANTS_RADIUS_OF_EARTH);
}
/**
 * @Input: void
 * @Output: void
 * @Description: 任务控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */

void TASK_MAIN::control_position(Point waypoint_sp)
{
   
            float fw_airspd_x = fw_states_f.wind_estimate_x + fw_states_f.global_vel_x;
            float fw_airspd_y = fw_states_f.wind_estimate_y + fw_states_f.global_vel_y;
             fw_arispd.set_vec_ele(fw_airspd_x, fw_airspd_y);                              /* 本机空速向量 */
             fw_gspeed_2d.set_vec_ele(fw_states_f.global_vel_x, fw_states_f.global_vel_y); /* 本机地速向量 */
              if ((fw_arispd.len() - fw_states_f.air_speed) >= 3.0) /* 计算获得的空速与读取的空速差距较大 */
                {
                    fw_airspd_states_valid = false;
                }
                else
                {
                    fw_airspd_states_valid = true;
                }

                if (fw_gspeed_2d.len() <= 3.0) /*本机地速太小的情况*/
                {
                    if (fw_states_f.yaw_valid)
                    {//本机地速太小，选用从机航向角
                    fw_cos_dir = cos(fw_states_f.yaw_angle);
                    fw_sin_dir = sin(fw_states_f.yaw_angle);
                    }
                    else
                    {//本机地速太小，且从机航向角未知
                    fw_cos_dir = 0;
                    fw_sin_dir = 0;
                    }
                }
                else
                {//本机地速正常，选用从机地速方向
                    fw_cos_dir = fw_gspeed_2d.x / fw_gspeed_2d.len();
                    fw_sin_dir = fw_gspeed_2d.y / fw_gspeed_2d.len();
                }

                  /* 保证归一化的结果，此向量十分重要，代表了本机速度方向*/
                      Vec fw_dir_unit(fw_cos_dir, fw_sin_dir);
                     fw_dir_unit = fw_dir_unit.normalized();

                Point current_pos(fw_states_f.latitude, fw_states_f.longitude);    /* 当前位置 */
                Vec vector_plane_sp=get_plane_to_sp_vector(current_pos,waypoint_sp);

               fw_NE_Distance=cov_lat_long_2_m(current_pos,waypoint_sp);
               Distance=sqrt(fw_NE_Distance.x*fw_NE_Distance.x+fw_NE_Distance.y*fw_NE_Distance.y);
                l1_controller.l1_controller(current_pos, waypoint_sp, fw_gspeed_2d, fw_states_f.air_speed);
                roll_cmd = l1_controller.nav_roll(); /* 获取期望控制滚转 */
                //roll_cmd=
    
                
}
/**
 * @Input: void
 * @Output: void
 * @Description:导航解算
 */
void TASK_MAIN::Navigation_solution()
{
    //******************导航解算********************//
        cout<<"******************************************************"<<endl;
        cout<<"当前上个航点："<<waypoint_next<<endl;
        cout<<"当前航点："<<waypoint_next+1<<endl;
        cout<<"当前下个航点："<<waypoint_next+2<<endl;
        //cout<<"Distance:"<<Distance<<endl;
        NAV_solution.RealativeLocation(cur_lat_lon[0],cur_lat_lon[1],pre_lat_lon[0],pre_lat_lon[1],sp_lat_lon[0], sp_lat_lon[1]);
        // cout<<"xg:\t"<<NAV_solution.out[0];
        // cout<<"zg:\t"<<NAV_solution.out[1];
        // cout<<"sg:\t"<<NAV_solution.out[2];
        // cout<<"range:\t"<<NAV_solution.out[3];
        // cout<<"angle_a:\t"<<NAV_solution.out[4];
                
       //当前纬经度    单位：弧度
        cur_lat_lon[0]=deg_2_rad(thisfw_states.latitude);
        cur_lat_lon[1]=deg_2_rad(thisfw_states.longitude);
        //期望经纬
        sp_lat_lon[0]=deg_2_rad(waypoint_qgc[waypoint_next][0]);
        sp_lat_lon[1]=deg_2_rad(waypoint_qgc[waypoint_next][1]);
        //前一个经纬
        pre_lat_lon[0]=deg_2_rad(waypoint_qgc[waypoint_next-1][0]);
        pre_lat_lon[1]=deg_2_rad(waypoint_qgc[waypoint_next-1][1]);
        //期望航点经纬度的下一个经纬度
        next_sp_lat_lon[0]=deg_2_rad(waypoint_qgc[waypoint_next+1][0]);
        next_sp_lat_lon[1]=deg_2_rad(waypoint_qgc[waypoint_next+1][1]);

        //当前位置      单位
        I_p[0]=cos(cur_lat_lon[0])*cos(cur_lat_lon[1]);
        I_p[1]=cos(cur_lat_lon[0])*sin(cur_lat_lon[1]);
        I_p[2]=sin(cur_lat_lon[0]);
        //期望位置
        I_t[0]=cos(sp_lat_lon[0])*cos(sp_lat_lon[1]);
        I_t[1]=cos(sp_lat_lon[0])*sin(sp_lat_lon[1]);
        I_t[2]=sin(sp_lat_lon[0]);
        //前一个位置
        I_s[0]=cos(pre_lat_lon[0])*cos(pre_lat_lon[1]);
        I_s[1]=cos(pre_lat_lon[0])*sin(pre_lat_lon[1]);
        I_s[2]=sin(pre_lat_lon[0]);

      //  Dis=(CONSTANTS_RADIUS_OF_EARTH+thisfw_states.relative_alt)*sqrt((I_t[0]-I_p[0])*(I_t[0]-I_p[0])+(I_t[1]-I_p[1])*(I_t[1]-I_p[1])+(I_t[2]-I_p[2])*(I_t[2]-I_p[2]))/57.3;
        //当前位置P与导航点T之间的距离：D = R_e*arccos(I_t * I_p);
        Dis=(CONSTANTS_RADIUS_OF_EARTH+thisfw_states.relative_alt)*acos( I_p[0]* I_t[0]+ I_p[1]* I_t[1]+ I_p[2]*I_t[2]);
        cout<<"当前位置与导航点的直线距离:"<<Dis<<endl;
        
        //计算航线偏差DY 以及航线偏离速度，当前位置P(Lamb,B) ，航线起始导航点S(Lamb_0,B_0)，末端航点E(Lamb_1,B_1)
        //计算 I_0, 两点叉乘/叉乘的模
        double i0,i1,i2,i_len;
        i0=I_t[1]*I_s[2]-I_t[2]*I_s[1];
        i1=I_t[2]*I_s[0]-I_t[0]*I_s[2];
        i2=I_t[0]*I_s[1]-I_t[1]*I_s[0];
        i_len=sqrt(i0*i0+i1*i1+i2*i2);
        
        I_0[0]=i0/i_len;
        I_0[1]=i1/i_len;
        I_0[2]=i2/i_len;
        //航向侧向偏差
        Dis_y=(CONSTANTS_RADIUS_OF_EARTH+thisfw_states.relative_alt)*(I_p[0]*I_0[0]+I_p[1]*I_0[1]+I_p[2]*I_0[2]);
        
        cout<<"侧偏距离:"<<Dis_y<<endl;
        //cout<<"zg:\t"<<NAV_solution.out[1]<<endl;
        cout<<"侧偏速度:"<<DYr<<endl;
        //得到期望航向角度   坐标系的定义
                a_pos.x= rad_2_deg(pre_lat_lon[0]);
                a_pos.y=rad_2_deg(pre_lat_lon[1]);
                b_pos.x=rad_2_deg(sp_lat_lon[0]);
                b_pos.y=rad_2_deg(sp_lat_lon[1]);
                dir=get_plane_to_sp_vector(a_pos,b_pos);
                 heading_sp=atan2(dir.y,dir.x);//（0->180 -180->0）
                if(heading_sp<0)
                 {
                     heading_sp=heading_sp+2*PI;
                 }
            cout<<"当前航路段方向角:"<<heading_sp<<endl;
            //cout<<"angle_a:\t"<<NAV_solution.out[4]<<endl;
        //得到期望下一个航向角度
                a_pos.x= rad_2_deg(sp_lat_lon[0]);
                a_pos.y=rad_2_deg(sp_lat_lon[1]);
                b_pos.x=rad_2_deg(next_sp_lat_lon[0]);
                b_pos.y=rad_2_deg(next_sp_lat_lon[1]);
                dir=get_plane_to_sp_vector(a_pos,b_pos);
                 heading_sp_next=atan2(dir.y,dir.x);//（0->180 -180->0）
                if(heading_sp_next<0)
                 {
                     heading_sp_next=heading_sp_next+2*PI;
                 }
              cout<<"下一航路段方向角:"<<heading_sp_next<<endl;
        //得到实际的航向角
             heading_cur=thisfw_states.yaw_angle;
              cout<<"当前飞机方向角:"<<heading_cur<<endl;
         //把当前飞机ned坐标系投影到期望航迹角下的速度差    DYr = Ve * cos(psi_c)-Vn * sin(psi_c);
        DYr = thisfw_states.global_vel_y*cos(heading_sp)-thisfw_states.global_vel_x*sin(heading_sp);


    //利用三个航点判断转弯角度，转弯提前距
    Turn_angle=heading_sp_next-heading_sp;  
//转弯只转小角度0-180
 if(Turn_angle>PI)
    {
        Turn_angle=Turn_angle-2*PI;
    }
    else if(Turn_angle<-PI)
    {
        Turn_angle=Turn_angle+2*PI;
    }
//提前转弯距离  Sd = R_turn * tan(|d_psi_c|/(2*57.3));
    //************转弯半径********************//
    //倾斜角30~50度
    //Rcmd = thisfw_states.air_speed * thisfw_states.air_speed / (9.81 *tan(Turn_angle) );//tan(Turn_angle) tan(deg_2_rad( thisfw_states.roll_angle))   0.55
    cout<<"Rcmd:"<<Rcmd<<endl;
    Turn_advance_distance=Rcmd*tan(abs(Turn_angle)/2);
     cout<<"转弯判断角度:"<<Turn_angle<<endl;
      cout<<"转弯判断转弯提前距离:"<<Turn_advance_distance<<endl;

     //计算转弯圆心
    double O_theta=PI/2+heading_sp_next+heading_sp/2;
    double O_d=Rcmd/cos(Turn_angle);
        //转弯圆心点 
         sp_o_lat_lon[0]=sp_lat_lon[0]+deg_2_rad(O_d*cos(O_theta));
         sp_o_lat_lon[1]=sp_lat_lon[1]+deg_2_rad(O_d*sin(O_theta));
        //验算转弯判断点0
        Point A,B;
        A.x=rad_2_deg(sp_lat_lon[0]);
        A.y=rad_2_deg(sp_lat_lon[1]);
        B.x=rad_2_deg(sp_o_lat_lon[0]);
        B.y=rad_2_deg(sp_o_lat_lon[1]);
        Vec angle_vec=get_plane_to_sp_vector(A,B);
        double angle=atan2(angle_vec.y,angle_vec.x);
        if(angle<0)
        {
            angle=angle+2*PI;
        }
        cout<<"圆心角度:"<<rad_2_deg(angle)<<endl; 

        //根据末端点E计算 撞线判断航点C 的经纬度  sin  cos 用弧度
        sp_c_lat_lon[0]=sp_lat_lon[0]+deg_2_rad(0.1*cos(heading_sp+PI/2));
        sp_c_lat_lon[1]=sp_lat_lon[1]+deg_2_rad(0.1*sin(heading_sp+PI/2));
        //sp_c_lat_lon[0]=sp_lat_lon[0]+deg_2_rad(0.1*cos(heading_sp));   // 维度 
        //sp_c_lat_lon[1]=sp_lat_lon[1]+deg_2_rad(0.1*sin(heading_sp)/cos(sp_lat_lon[0]));    // 经度  

        //验算转弯判断点C
     /*   Point A,B;
        A.x=rad_2_deg(sp_lat_lon[0]);
        A.y=rad_2_deg(sp_lat_lon[1]);
        B.x=rad_2_deg(sp_c_lat_lon[0]);
        B.y=rad_2_deg(sp_c_lat_lon[1]);
        Vec angle_vec=get_plane_to_sp_vector(A,B);
        double angle=atan2(angle_vec.y,angle_vec.x);
        if(angle<0)
        {
            angle=angle+2*PI;
        }
        cout<<"撞线角度:"<<rad_2_deg(angle)<<endl;*/

        //C的位置  判断撞线
        I_c[0]=cos( sp_c_lat_lon[0])*cos(sp_c_lat_lon[1]);
        I_c[1]=cos( sp_c_lat_lon[0])*sin(sp_c_lat_lon[1]);
        I_c[2]=sin(sp_c_lat_lon[0]);

        i0=I_c[1]*I_t[2]-I_c[2]*I_t[1];
        i1=I_c[2]*I_t[0]-I_c[0]*I_t[2];
        i2=I_c[0]*I_t[1]-I_c[1]*I_t[0];
        i_len=sqrt(i0*i0+i1*i1+i2*i2);
        I_0[0]=i0/i_len;
        I_0[1]=i1/i_len;
        I_0[2]=i2/i_len;

        //当前导航点到下一导航点判断线的距离gD
        gD=(CONSTANTS_RADIUS_OF_EARTH+thisfw_states.relative_alt)*(I_p[0]*I_0[0]+I_p[1]*I_0[1]+I_p[2]*I_0[2]);
        cout<<"撞线距离"<<gD<<endl;
         cout<<"******************************************************"<<endl;
        //三次判断gD<0，经过EC，到达航点E
         if(gD<0)
         {
            waypoint_next++;
            waypoint_sp.x=waypoint_qgc[waypoint_next][0];
            waypoint_sp.y=waypoint_qgc[waypoint_next][1];
         }
}
/**
 * @Input: void
 * @Output: void
 * @Description:限幅
 */
double  TASK_MAIN::constrain(int val,int min ,int max)
    {
            return (val < min) ? min : ((val > max) ? max : val);
    }

/**
 * @Input: void
 * @Output: void
 * @Description: 任务控制器主函数     起飞完成后进入巡航段，实现直线飞行以及转弯
 */
void TASK_MAIN::control_mission()
{
    //航点切换
    t=current_time-start_time;
    if(t>=t4)//进入巡航段
    {
    //横侧向控制
    //直线控制，转弯控制判断
    Navigation_solution();     //通过程序计算得到各变量的值

    Turn_flag=false;
    Straight_flag=true;
    //连续三次满足//当前位置到下一导航点的位置小于转弯提前距
    if(gD<Turn_advance_distance&&Turn_advance_distance&&waypoint_next!=POINT_NUM_QGC)
    {
         TASK_MAIN_INFO("入弯点！！！！！！");
         Turn_flag=true;
         Straight_flag=false;
         cout<<rad_2_deg(abs(Turn_angle))<<endl;
         cnt++;
         if(cnt==3)
         {
                    if(rad_2_deg(abs(Turn_angle))<4||Dis_y<5)    
                {
                    TASK_MAIN_INFO("出弯点！！！！！！");
                    Straight_flag=true;
                    Turn_flag=false;
                }
                cnt=0;
         }

     }

    //修改左右转弯的roll指令， 计算Rcmd的方法
    // cout<< "滚转角"<< thisfw_states.roll_angle<<endl;
    // cout<< "滚转角处理"<< deg_2_rad( thisfw_states.roll_angle)<<endl;
    
    //Rcmd = 75;//thisfw_states.air_speed * thisfw_states.air_speed / (9.81 * tan(Turn_angle));  //tan(Turn_angle)tan(deg_2_rad( thisfw_states.roll_angle))  0.55
     cout<<"期望转弯角度Turn_angle："<<Turn_angle<<endl;
    cout<<"期望转弯半径Rcmd："<<Rcmd<<endl;
    //开始转弯
    if(Turn_flag&&!Straight_flag)    //转弯标志为1
    {
         TASK_MAIN_INFO("转弯控制");
         if(Turn_angle>0)
         {
            TASK_MAIN_INFO("右转弯");
            if(abs(Dis_y)>1)
            {
                ki_a = 0;
            }
            else
            {
                ki_a = 1;
                Dis_y_i +=  Dis_y;
            }

             //roll_cmd=-0.7*(Dis_y-Rcmd)-0.7*(DYr-RVcmd)-0.01*Dis_y_i;
             roll_cmd=-0.7*(Dis_y-Rcmd)-0.7*(DYr-RVcmd)-0.01*Dis_y_i;
             cout<<"Roll_CMD："<<roll_cmd<<endl;
             cout<<"右转Dis_y-Rcmd："<<Dis_y-Rcmd<<endl;

         }
         else if(Turn_angle<0)
         {
            TASK_MAIN_INFO("左转弯");
            if(abs(Dis_y)>1)
            {
                ki_a = 0;
            }
            else
            {
                ki_a = 1;
                Dis_y_i +=  Dis_y;
            }
           // roll_cmd=-0.7*(Dis_y+Rcmd)-0.7*(DYr+RVcmd)-0.01*Dis_y_i;
             roll_cmd=-0.7*(Dis_y+Rcmd)-0.7*(DYr+RVcmd)-0.01*Dis_y_i;
             cout<<"Roll_CMD："<<roll_cmd<<endl;
             cout<<"左转Dis_y+Rcmd："<<Dis_y+Rcmd<<endl;
         }
    }
    else
    {
        TASK_MAIN_INFO("直线控制控制"); 
        //积分分离
        if(abs(Dis_y)>1)
        {
            ki_a = 0;
        }
        else
        {
            ki_a = 1;
            Dis_y_i +=  Dis_y;
        }

        //roll_cmd=-0.2*Dis_y-0.7*DYr  - 0.01 * ki_a* Dis_y_i;
         roll_cmd=-0.2*Dis_y-0.7*DYr  - 0.01 * ki_a* Dis_y_i;
         cout<<"直线控制指令roll_cmd："<<roll_cmd<<endl;
         cout<<"直线Dis_y："<<Dis_y<<endl;
    }

   roll_cmd=constrain(roll_cmd,-30,30);
        // control_position(waypoint_sp);
         //   Navigation_solution();
        //直线滚转
      //  if(abs(heading_cur-heading_sp)<=4 || abs(Dis_y)<10)
       // {

                     //roll_cmd=-0.5*Dis_y-0.1*DYr;
                  //   roll_cmd=-0.5*Dis_y-0.1*DYr;
                   //  TASK_MAIN_INFO("进入直线控制");

       // }
       // else  if(Dis<100*tan())
        


/*************************************************************************************/
//                                                                      纵向控制                                                                                                  //
/*************************************************************************************/
        speed_ctl.init_pid(0.5,0.01,0);
        h_err_i=h_err_i+thisfw_states.relative_alt-100;
        //   pitch_cmd=-(thisfw_states.relative_alt-100)*0.5-(-thisfw_states.ned_vel_z)*2+2*(1-cos(thisfw_states.yaw_angle-90))-h_err_i;
        height_ctl.init_pid(0.5,0.01,0);
         //  pitch_cmd=height_ctl.pid_anti_saturated(100-thisfw_states.relative_alt,1,0,"pitch");//暂时用高度的误差一个比例这个控制器
        throttle_cmd= speed_ctl.pid_anti_saturated(20-thisfw_states.ground_speed,1,0,"throttle");
        //t>t4的时候
        
        //积分项做处理
        if(abs(HH)>5)
        {
            ki_a = 0;
        }
        else
        {
            ki_a = 1;
            HH += (thisfw_states.relative_alt-100);
        }
        cout<<"HH:"<<HH<<endl; 
        pitch_cmd=-(thisfw_states.relative_alt-100)*0.5-(-thisfw_states.ned_vel_z)*2
        +0.1*(1-cos(deg_2_rad(thisfw_states.roll_angle)))-0.01*HH;
        /*对俯仰角进行限幅*/
         cout<<"pitch_cmd:"<<pitch_cmd<<endl;
//发布控制指令
        fw_4cmd.throttle_sp =throttle_cmd;
        //fw_4cmd.roll_angle_sp =roll_cmd;
       //cout<<"滚转指令1111："<<roll_cmd<<endl;
        fw_4cmd.roll_angle_sp=deg_2_rad(roll_cmd);
        fw_4cmd.pitch_angle_sp =deg_2_rad(pitch_cmd);
        fw_4cmd.yaw_angle_sp =0;
  
        if(waypoint_next==POINT_NUM_QGC+1)
        {
           fw_cmd_mode.need_mission=false;
           fw_cmd_mode.need_protected=true;
        }

    }

}


void TASK_MAIN::control_land()
{

        fw_4cmd.throttle_sp =0;
        fw_4cmd.roll_angle_sp =deg_2_rad(0);
        fw_4cmd.pitch_angle_sp =deg_2_rad(0);
        fw_4cmd.yaw_angle_sp =0;
        fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */
      
}

/**
 * @Input: void
 * @Output: void
 * @Description: 任务控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */
void TASK_MAIN::control_protected()
{
       /* time_test+=_dt;
        pitch_cmd=7.5*time_test-30;
        if(pitch_cmd>=30)
        {
            time_test=0;
            pitch_cmd=-30;
        }*/
        fw_4cmd.throttle_sp =0.6;
        fw_4cmd.roll_angle_sp =deg_2_rad(0);
        fw_4cmd.pitch_angle_sp =deg_2_rad(15);
        fw_4cmd.yaw_angle_sp =0;
        fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */
}



/**1
 * @Input: void
 * @Output: void
 * @Description: 外部文件传入的ros_params向编队控制器（外壳，tecs+横侧向控制器）
 */
void TASK_MAIN::input_params()
{
    /* 速度产生的参数调参 */
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/kv_p"      , mix_Xerr_params.kv_p   , 0.1);//0.2  0.5
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/kp_p"      , mix_Xerr_params.kp_p   , 1.5 );//0.5    1
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/mix_kp"    , mix_Xerr_params.mix_kp ,1.5);//0.4  0.5
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/mix_ki"    , mix_Xerr_params.mix_ki , 0.001);//0  0
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_V_sp/mix_kd"    , mix_Xerr_params.mix_kd , 0.1);//0.1  0.1


    /* 滚转角产生的参数调参 */
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/keta_p" , mix_Yerr_params.keta_p , 0.5);//0.5  0.5
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/kp_p"   , mix_Yerr_params.kp_p   ,1);//0.2  1
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/mix_kp" , mix_Yerr_params.mix_kp ,1);//0.4 1
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/mix_ki" , mix_Yerr_params.mix_ki , 0.001);//0 0.001
    nh.param<float>("fixwing_formation_control/abs_formation_controller/gen_roll_sp/mix_kd" , mix_Yerr_params.mix_kd , 0.1);//0.1 0.1



    TASK_MAIN_INFO(mix_Xerr_params.kv_p  );
    TASK_MAIN_INFO(mix_Xerr_params.kp_p  );
    TASK_MAIN_INFO(mix_Xerr_params.mix_kp);
    TASK_MAIN_INFO(mix_Xerr_params.mix_ki);
    TASK_MAIN_INFO(mix_Xerr_params.mix_kd);

    TASK_MAIN_INFO(mix_Yerr_params.keta_p);
    TASK_MAIN_INFO(mix_Yerr_params.kp_p  );
    TASK_MAIN_INFO(mix_Yerr_params.mix_kp);
    TASK_MAIN_INFO(mix_Yerr_params.mix_ki);
    TASK_MAIN_INFO(mix_Yerr_params.mix_kd);


}

/**
 * @Input: struct FORMATION_CONTROLLER::_s_fw_states *p
 * @Output: void
 * @Description: TODO:飞机状态打印函数，应该写作模板函数
 */
void TASK_MAIN::print_data(const struct FORMATION_CONTROLLER::_s_fw_states *p)
{
    cout << "***************以下是本飞机状态******************" << endl;
    cout << "***************以下是本飞机状态******************" << endl;

    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;
    cout << "飞机当前姿态欧美系【roll，pitch，yaw】" << rad_2_deg(p->roll_angle) << " [deg] "
         << rad_2_deg(p->pitch_angle) << " [deg] "
         << rad_2_deg(p->yaw_angle) << " [deg] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "飞机当前姿态的旋转矩阵【第2行】" << p->rotmat[2][0] << " [] "
         << p->rotmat[2][1] << " [] "
         << p->rotmat[2][2] << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "body下的加速度【XYZ】" << p->body_acc[0] << " [m/ss] "
         << p->body_acc[1] << " [m/ss] "
         << p->body_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的速度【XYZ】" << p->ned_vel_x << " [m/s] "
         << p->ned_vel_y << " [m/s] "
         << p->ned_vel_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "ned下的加速度【XYZ】(由旋转矩阵得来)" << p->ned_acc[0] << " [m/ss] "
         << p->ned_acc[1] << " [m/ss] "
         << p->ned_acc[2] << " [m/ss] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "GPS位置【lat,long,alt,rel_alt】" << p->latitude << " [] "
         << p->longitude << " [] "
         << p->altitude << " [] "
         << p->relative_alt << " [] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "风估计【x,y,z】" << p->wind_estimate_x << " [m/s] "
         << p->wind_estimate_y << " [m/s] "
         << p->wind_estimate_z << " [m/s] " << endl;
    for (int i = 1; i <= the_space_between_lines; i++)
        cout << endl;

    cout << "***************以上是本飞机状态******************" << endl;
    cout << "***************以上是本飞机状态******************" << endl;
    for (int i = 1; i <= the_space_between_blocks; i++)
        cout << endl;
}

/**
 * @Input: void
 * @Output: void
 * @Description: 总任务循环函数
 */
void TASK_MAIN::run()
{
    ros::Rate rate(50.0);  //设置频率
    begin_time = ros::Time::now(); /* 记录启控时间 */
    ros_sub_pub();            //

    input_params();// /1* 读取参数 *1/ 

    while (ros::ok())
    {
        /**
        * 任务大循环，根据来自switch_mode的控制指令来进行响应的控制动作
       */
         //cout<<"heading_deg"<<thisfw_states.yaw_angle<<endl;
        current_time = get_ros_time(begin_time); /*此时刻，只作为纪录，不用于控制*/
        TASK_MAIN_INFO("Time:" << current_time);
        /*更新飞机状态*/
         update_fwsates();
        filter_states();
        if(current_time>10)
        {
            if(record==1)
                {
                    record=0;
                    formation_controller.get_launch_point(leader_states.latitude,leader_states.longitude,leader_states.altitude);
                }
        }
       // Navigation_solution();
        if (!fw_cmd_mode.need_take_off &&
            !fw_cmd_mode.need_formation &&
            !fw_cmd_mode.need_land &&
            !fw_cmd_mode.need_idel &&
            fw_cmd_mode.need_protected&&
            !fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("保护子程序");
            /**
             * TODO:保护子程序
             */

            control_protected();
           // fw_current_mode.mode =
            //    fixed_wing_formation_control::Fw_current_mode::FW_IN_PROTECT;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 !fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("空闲子程序");
            /**
                 * TODO:空闲子程序
                */
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_IDEL;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 !fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("降落子程序");
            /**
                 * TODO:降落子程序
                */
             control_land();
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_LANDING;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                !fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("编队子程序");
            /**
                 * TODO:虽然完成了节点参数的输入函数以及各个通路，但是节点的参数并没有加载进来
                */
            control_formation();

            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_FORMATION;
        }
        else if (fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 !fw_cmd_mode.need_mission)
        {

            TASK_MAIN_INFO("起飞子程序");
            /**
                 * TODO:起飞子程序
                */
           // fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_TAKEOFF;
           control_takeoff();
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("任务子程序");
            /**
                 * TODO:任务子程序
                */
           // fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_MISSION;
          control_mission();
        }
        else
        {
            TASK_MAIN_INFO("错误，飞机当前状态有误");
        }

        /**
         * 发布飞机当前状态
        */
        fw_current_mode_pub.publish(fw_current_mode);
        fw_state_pub();
       // cout<<"leader_states.relative_alt:"<<leader_states.relative_alt<<endl;
      //  print_data(&thisfw_states);

     // cout<<"leaderstates.relative_hight:"<<leaderstates.relative_hight<<endl;
        //printwaypoint(current_waypoints);
        //      cout<<"POINT_NUM_QGC"<<POINT_NUM_QGC<<endl;
/*
                       for(int i=0;i<POINT_NUM;i++)
      {
          way_point_sp[i].x=waypoint_cin[i][0];
          way_point_sp[i].y=waypoint_cin[i][1];
      }*/

   // waypoint_cin[0][0]=47.3977509000;
    //waypoint_cin[0][1]=8.5510530000;
    //waypoint_cin[1][0]=47.3977394000;
    //waypoint_cin[1][1]=8.5550530000;
                //for(int i=0;i<15;i++)
                //{
               
                //}

//for(int i=0;i<15;i++)
  //  cout<<"waypoint_cin.x"<<":"<<waypoint_cin[i][0]<<"waypoint_cin.y"<<":"<<waypoint_cin[i][1]<<endl;

     //   cout<<"waypoint_cin.x:"<< way_point_sp[0].x<<"waypoint_cin.y:"<<way_point_sp[0].y<<endl;
        ros::spinOnce();
        rate.sleep();
    }

    return;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "task_main");
    TASK_MAIN _task_main;
    if (true)
    {
        _task_main.run();
    }
    return 0;
}