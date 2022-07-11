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

    //cout<<"count:"<<points.waypoints.size()<<endl;
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
    leaderstates_update = 1;
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

 mc_local_att_sp_pub = 
    nh.advertise  /*   【发布】 mc控制量*/
    <fixed_wing_formation_control::my_ctrl>
    (add2str(uavID, "fixed_wing_formation_control/mc_ctrl"), 10);

}

/**
 * @Input: void
 * @Output: void
 * @Description: 编队状态值发布，编队位置误差（机体系和ned），速度误差以及期望空速，gps，期望地速
 */
void TASK_MAIN::fw_state_pub() {
//    fw_4cmd.Dis_y=Dis_y;
//    fw_4cmd.Dis=Dis;
//    fw_4cmd.DYr=DYr;
//     fw_4cmd.Dis_y=u_Ymis_n-H_cx;
//    fw_4cmd.Dis=u_VY_n-Vy_CX;
//    fw_4cmd.DYr=DYr;

   fw_cmd_pub.publish(fw_4cmd);   //需要把这个改成iris的。
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

 /*控制量*/
 cout<<"控制量存储"<<endl;
 float ctrl_ax1,ctrl_ay1;
formation_controller.get_adaptive_ctrl(&ctrl_ax1,&ctrl_ay1);
 cout<<"ctrl_ax:"<<ctrl_ax1<<endl;
 formation_control_states.ctrl_ax = ctrl_ax1;
  cout<<"ctrl_ay:"<<ctrl_ay1<<endl;
formation_control_states.ctrl_ay = ctrl_ay1;

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
     thisfw_states.air_speed=fwstates.air_speed;
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
    
    thisfw_states.relative_hight=fwstates.relative_hight;
    formation_control_states.err_PZb = formation_error.PZb;

    //Navigation_solution();
}

void TASK_MAIN::control_formation2()
{
     /*1、获取形状*/
    /* 设定编队形状  根据模式切换得到  id,swarm_shape,swarm_size,swarm_num */
    formation_controller.set_formation_type(planeID,fw_cmd_mode.swarm_shape,5,5);

}
/**
 * @Input: void
 * @Output: void
 * @Description: 编队控制器主函数，完成对于领机从机状态的赋值，传入编队控制器
 */
void TASK_MAIN::control_formation()
{
    /*1、获取形状*/
    /* 设定编队形状  根据模式切换得到  id,swarm_shape,swarm_size,swarm_num */
    formation_controller.set_formation_type(planeID,fw_cmd_mode.swarm_shape,20,5);
    /*2、获取控制器参数 kv_p  kp_p   mix_kp/ki/kd*/
    /* 设定前向编队混合参数 */
    formation_controller.set_mix_Xerr_params(mix_Xerr_params);

    /* 设定侧向编队混合参数 */
    formation_controller.set_mix_Yerr_params(mix_Yerr_params);
    
    /* 模式不一致，刚切换进来的话，重置一下控制器，还得做到控制连续！！ */
    if (fw_col_mode_current != fw_col_mode_last)
    {
       formation_controller.reset_formation_controller();
    }

    /*3、更新飞机状态，领机和当前飞机，当前的飞机用指针，更加方便，避免内存浪费*/
    /* 更新飞机状态，领机状态 */
    formation_controller.update_led_fol_states(&leader_states, &thisfw_states);

    /* 4、编队控制 */
    formation_controller.control_formation();

    /* 5、获得最终控制量 */
    formation_controller.get_formation_4cmd(formation_cmd);
    /*6、 获得编队控制期望值   更新 */
    formation_controller.get_formation_sp(formation_sp);
    /*7、 获得编队误差信息  更新*/
    formation_controller.get_formation_error(formation_error);

    /* 8、控制量赋值 */
    fw_4cmd.throttle_sp = formation_cmd.thrust;
    fw_4cmd.roll_angle_sp = formation_cmd.roll;   //使用的是航机系X轴误差
    fw_4cmd.pitch_angle_sp = formation_cmd.pitch; 
    fw_4cmd.yaw_angle_sp = formation_cmd.yaw;

    /*不在这里发布*/
  //fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量  编队控制器得到内环期望的姿态角度 */


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
/************************************************************************/
/* Vincenty正解公式，据目标点大地距离与大地方位角求目标经纬度

参数说明：
S_MT           [in]		: 距目标点大地距离
Alf_MT         [in]		: 与目标点大地方位角，单位：度
g_Longitude_T  [in/out]	: 目标点经度，单位：度
g_Latitude_T   [in/out]	: 目标点纬度，单位：度
g_A21_TM       [in/out]	: 与目标点反方位角，单位：度
/************************************************************************/
void TASK_MAIN::Vincent_Z(double S_MT,double Alf_MT,double g_Longitude,double g_Latitude,double *g_Longitude_T,double *g_Latitude_T)
{
  double E1,Aa,B,D,c,m,sigmam_2,tan_u1;
	double k1,sin_m;
	double u1,sigma1,sigma,sigma_b,delta_sigma,lamda;
	double x1,y1,x2,y2;
	double E = (1.0 / 298.257223563);
	double Pi = 3.14159265358979;
	double AtoR = 180. / Pi;
	double Re = 6378137.0;                //地球半径，(m)
	double Rb = (Re * (1 - E));			//Rb地球短半径
	double ee = (Re * Re - Rb * Rb ) / (Rb * Rb);


	tan_u1 = (1 - E) * tan(g_Latitude / AtoR);
	u1 = atan(tan_u1);
	sigma1 = atan(tan_u1 / cos(Alf_MT / AtoR));

	if( cos(Alf_MT / AtoR) < 0.0 && tan_u1 > 0.0 ) 
	{
		sigma1 = sigma1 + Pi; 
	}
	if( cos(Alf_MT / AtoR) < 0.0 && tan_u1 < 0.0 ) 
	{
		sigma1 = sigma1-Pi;
	}

	sin_m = cos(u1) * sin(Alf_MT / AtoR);
	m = asin(sin_m);

	k1 = (sqrt(1.0 + ee * (1.0 - sin_m * sin_m)) - 1.0) / (sqrt(1.0 + ee*(1.0 - sin_m * sin_m)) + 1.0);
	
	Aa = (1.0 + k1 * k1 / 4.0) / (1.0 - k1);
	B = k1 * (1.0 - 3.0 * k1 * k1 / 8.0);
	sigma = S_MT / (Rb * Aa);
	do
	{
		sigma_b  = sigma;
		sigmam_2 = 2 * sigma1 + sigma;
		
		
		delta_sigma = B * sin(sigma) * (cos(sigmam_2) + B * (E1 * cos(sigma) - D) / 4.0);
		sigma = S_MT / (Rb * Aa) + delta_sigma;

	} while(fabs(sigma - sigma_b) > 0.3e-11);
	
	*g_Latitude_T = atan((sin(u1) * cos(sigma) + cos(u1) * sin(sigma) * cos(Alf_MT / AtoR))	/ ((1 - E) * sqrt(sin_m * sin_m	+ pow(sin(u1) * sin(sigma)- cos(u1) * cos(sigma) * cos(Alf_MT / AtoR),2.0))));
	
	x1 = sin(sigma) * sin(Alf_MT / AtoR);
	y1 = cos(u1) * cos(sigma) - sin(u1) * sin(sigma) * cos(Alf_MT / AtoR);
	lamda = atan(x1 / y1);
	
	if( y1 < 0.0 && x1 > 0.0)
	{
		lamda = lamda+Pi;
	}
	else if( y1 < 0 && x1 < 0.0 )
	{ 
		lamda = lamda-Pi;
	}
	
	c = E * cos(m) * cos(m) * (4.0 + E * (4.0 - 3.0 * cos(m) * cos(m))) / 16.0;
	
	*g_Longitude_T = g_Longitude / AtoR + lamda - (1.0 - c) * E * sin_m * (sigma + c * sin(sigma) * (cos(sigmam_2) + E1 * c * cos(sigma)));
	
	if(fabs(*g_Longitude_T) > Pi)
	{
		if(*g_Longitude_T > 0.0)
		{
			*g_Longitude_T = *g_Longitude_T - 2.0 * Pi;
		}
		else
		{
			*g_Longitude_T = *g_Longitude_T + 2.0 * Pi;
		}
	}
	
	x2 = -sin_m;
	y2 = sin(u1) * sin(sigma) - cos(u1) * cos(sigma) * cos(Alf_MT / AtoR);

	*g_Longitude_T = *g_Longitude_T * AtoR;
	*g_Latitude_T  = *g_Latitude_T * AtoR;

}

/************************************************************************/
/* Vincenty反解公式，据目标经纬度求目标点大地距离与大地方位角
参数说明：
Long_T         [in]	    : 目标点经度，单位：度
Lat_T          [in]	    : 目标点纬度，单位：度
g_R_MT         [out]	: 距目标点大地距离
g_Alpha12_MT   [out]	: 与目标点大地方位角，单位：度
/************************************************************************/
void TASK_MAIN::Vincenty_F(double Long_T,double Lat_T,double g_Longitude,double g_Latitude,double *g_R_MT,double *g_Alpha12_MT)
{
    double U1,U2,detLong,detLongd,sigma,sigamm_2,sigma_sin,sigma_cos,mm;	//单位：弧度
	double C,E1,K11,B,AA,D,detsigam,Alf_MT1,Alf_MT2;
	double E = (1.0 / 298.257223563);
	double Pi = 3.14159265358979;
	double AtoR = 180. / Pi;
	double Re = 6378137.0;                //地球半径，(m)
	double Rb = (Re * (1 - E));			//Rb地球短半径
	double ee = (Re * Re - Rb * Rb ) / (Rb * Rb);

	
	U1 = atan((1.0- E) * tan(g_Latitude / AtoR));	//单位：弧度
	U2 = atan((1.0- E) * tan(Lat_T / AtoR));	    //单位：弧度
	
	detLong	= (Long_T - g_Longitude) / AtoR;
	while (1) 
	{
		detLongd	= detLong;	
		sigma_sin	= sqrt(cos(U2) * cos(U2) * sin(detLong) * sin(detLong) + (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong)) * (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong)));
		sigma_cos	= sin(U1) * sin(U2) + cos(U1) * cos(U2) * cos(detLong);
		sigma		= atan(sigma_sin / sigma_cos);
		
		mm = asin( cos(U1) * cos(U2) * sin(detLong) / sigma_sin);
		C  = E * cos(mm) * cos(mm) * (4.0 + E * (4.0 - 3.0 * cos(mm) * cos(mm))) / 16.0;
		
		sigamm_2 = acos(sigma_cos - 2.0 * sin(U1) * sin(U2) / (cos(mm) * cos(mm)));
		E1  = 2.0 * cos(sigamm_2) * cos(sigamm_2) - 1.0;
		
		detLong = (Long_T - g_Longitude) / AtoR + (1.0 - C) * E * sin(mm) * (sigma + C * sigma_sin * (cos(sigamm_2) + E1 * C * sigma_cos));
      cout<<"fabs(detLong - detLongd) "<<fabs(detLong - detLongd) <<endl;
		if (fabs(detLong - detLongd) <= 0.3e-11)
		{
			break;
		}
	}

	K11 = (sqrt(1.0 + ee * cos(mm) * cos(mm)) - 1.0) / (sqrt(1.0 + ee * cos(mm) * cos(mm)) + 1.0);
	B   = K11 * (1.0 - 3.0 * K11 * K11 / 8.0);
	AA  = (1.0 + K11 * K11 / 4.0) / (1.0 - K11);
	D	= B * cos(sigamm_2) * (4.0 * sigma_sin * sigma_sin - 3.0) * (2.0 * E1 - 1) / 6.0;
	
	detsigam = B * sigma_sin * (cos(sigamm_2) + B * (E1 * sigma_cos - D) / 4.0);
	
	//大地距离
	*g_R_MT = (sigma - detsigam) * Rb * AA;
	
	//大地方位角（正方位角）
	Alf_MT1 = cos(U2) * sin(detLong);
	Alf_MT2 = cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong);
	*g_Alpha12_MT	= atan(Alf_MT1 / Alf_MT2);
	
	if (Alf_MT2 < 0.0)
	{
		*g_Alpha12_MT = Pi + *g_Alpha12_MT;
	}
	else if (Alf_MT1 < 0.0)
	{
		*g_Alpha12_MT = 2.0 * Pi + *g_Alpha12_MT;
	}
	
	*g_Alpha12_MT = *g_Alpha12_MT * AtoR;

}

 void TASK_MAIN:: Navigation(double lat_m, double long_m, double lat_s, double long_s, double lat_e, double long_e)
 {
       //******************导航解算********************//            
       //当前纬经度    单位：弧度
        cur_lat_lon[0]=lat_m;
        cur_lat_lon[1]=long_m;
        //期望经纬
        sp_lat_lon[0]=lat_e;
        sp_lat_lon[1]=long_e;
        //前一个经纬
        pre_lat_lon[0]=lat_s;
        pre_lat_lon[1]=long_s;

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

        Dis=(CONSTANTS_RADIUS_OF_EARTH+thisfw_states.relative_alt)*acos( I_p[0]* I_t[0]+ I_p[1]* I_t[1]+ I_p[2]*I_t[2]);

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

        //根据末端点E计算 撞线判断航点C 的经纬度  sin  cos 用弧度
        sp_c_lat_lon[0]=sp_lat_lon[0]+deg_2_rad(0.1*cos(heading_sp+PI/2));
        sp_c_lat_lon[1]=sp_lat_lon[1]+deg_2_rad(0.1*sin(heading_sp+PI/2));

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


                                if(heading_sp>=0.0&&heading_sp<=PI)
                                    heading_sp=-heading_sp;
                                else if(heading_sp>PI&&heading_sp<2.0*PI)
                                    heading_sp=-1.0*(heading_sp-2.0*PI);
        
        z_control[0]=gD;
         z_control[1]=Dis_y;
          z_control[2]=Dis;
           z_control[3]=0;
            z_control[4]=heading_sp;


 }

void TASK_MAIN::control_takeoff22()
{
      cout<<"1111"<<endl;
        fw_4cmd.roll_angle_sp=deg_2_rad(0);
        fw_4cmd.pitch_angle_sp =deg_2_rad(15);
        fw_4cmd.yaw_angle_sp =deg_2_rad(0);
        fw_4cmd.throttle_sp=0.65;
        fw_cmd_pub.publish(fw_4cmd);
}



// void TASK_MAIN::control_takeoff_iris()
// {
//         // float roll,pitch,yaw,thrust;
//         // roll = thisfw_states.roll_angle*Rad2Deg;
//         // pitch = thisfw_states.pitch_angle*Rad2Deg;
//         // yaw = thisfw_states.yaw_angle*Rad2Deg;

//        // x,y,z,vx,vy,vz,yaw,yaw_speed,acc,thrust;
        
//         float x_ned,y_ned,z_ned;
//         float vx_ned,vy_ned,vz_ned;
//         float x_sp,y_sp,z_sp,yaw_sp;
//         float ax,ay,az;
//         float kp_x{1.0},ki_x{1.0},kd_x{1.0};
//         float kp_y{1.0},ki_y{1.0},kd_y{1.0};
//         float kp_z{1.0},ki_z{1.0},kd_z{1.0};

//         // 期望位置
//         ax=kp_x*(x_ned-x_sp)+kd_x*vx_ned;
//         ay=kp_y*(y_ned-y_sp)+kd_y*vy_ned;
//         az=kp_z*(z_ned-z_sp)+kd_z*vz_ned;
//         yaw_sp=0;
        
//         //acc2attitude
//         //mc_ctrl.roll_sp = ay;
        
//         //mc_ctrl.pitch_sp=ax;
        
//         //mc_ctrl.yaw_sp=yaw_sp;

//         //mc_ctrl.thrust_sp = az;

//         mc_ctrl.vx = 0;
//         mc_ctrl.vy = 0;
//         mc_ctrl.vz = 1;
//         mc_ctrl.x_sp=0;
//         mc_ctrl.y_sp=0;
//         mc_ctrl.z_sp=5;
//         mc_ctrl.ax=0;
//         mc_ctrl.ay=0;
//         mc_ctrl.az=0;

//         cout<<"iris_takeoff"<<endl;
//         cout<<"fw_4cmd_roll_x_takeoff:"<<mc_ctrl.vz<<endl;
//         cout<<"fw_4cmd_pitch_y_takeoff:"<<mc_ctrl.z_sp<<endl;
//         cout<<"fw_4cmd_yaw_z_takeoff:"<<mc_ctrl.ay<<endl;
//         mc_local_att_sp_pub.publish(mc_ctrl); 
    
// }

void TASK_MAIN::control_zcontrol1()
{

    if(start_control==0)
    {
             cout<<"*********计算领机的侧偏位置*************"<<endl;
             if(thisfw_states.pitch_angle*Rad2Deg>=18.0)
                {
                        start_control=1;
                        //waypoint_qgc数组都是弧度
                        //角度都是弧度
                        //thisfw_status中经纬为°，角度是弧度
                        /*开始前，需要将航点载入地面站中，然后通过地面站进行上传*/
                        for (size_t i = 0; i < current_waypoints.waypoints.size(); i++)
                        {
                            if(current_waypoints.waypoints[i].command==16)//16普通航路点   22起飞航点 单位rad
                            {
                                POINT_NUM_QGC++;//0为无人机开始位置，1为第一个普通航点，起飞点不要  Sin cos计算用rad
                                waypoint_qgc[POINT_NUM_QGC][0]=current_waypoints.waypoints[i].x_lat*Deg2Rad;
                                waypoint_qgc[POINT_NUM_QGC][1]=current_waypoints.waypoints[i].y_long*Deg2Rad;
                            }
                        }
                        waypoint_qgc[0][0]=thisfw_states.latitude*Deg2Rad;
                        waypoint_qgc[0][1]=thisfw_states.longitude*Deg2Rad;
                        start_time=current_time;
                        Lat_ini=thisfw_states.latitude*Deg2Rad;
                        Long_ini=thisfw_states.longitude*Deg2Rad;
                        Lat_Target=waypoint_qgc[1][0];
                        Long_Target=waypoint_qgc[1][1];
                        ALPHA_start=thisfw_states.pitch_angle*Rad2Deg;
                        R_Target  = 100.0;          //转弯半径
                        angle_Target=90;
                        start_height=thisfw_states.relative_alt;
                        }
     }
     

     if(start_control==1)
     {
                cout<<"*******进入起控start_control=11111**************"<<endl;
                /**************************状态更新********************************/
                            t_Flight=current_time-start_time;
                            g_Height=thisfw_states.relative_alt-start_height;
                            u_Ymis_n=thisfw_states.relative_alt-start_height;
                            u_GAMMA=thisfw_states.roll_angle*Rad2Deg;
                            u_THETA=thisfw_states.pitch_angle*Rad2Deg;
                            V_MIS=thisfw_states.air_speed;
                            VN_x=thisfw_states.ned_vel_x;
                            VE_z=thisfw_states.ned_vel_y;
                            u_VY_n=-thisfw_states.ned_vel_z;
                            Lat=thisfw_states.latitude*Deg2Rad;
                            Long=thisfw_states.longitude*Deg2Rad;
                            PSCI_t=thisfw_states.yaw_angle;
                            if(PSCI_t>=0.0&&PSCI_t<=PI)
                                PSCI_t=-PSCI_t;
                           else if(PSCI_t>PI&&PSCI_t<2.0*PI)
                                PSCI_t=-1.0*(PSCI_t-2.0*PI);
                /**************************状态更新********************************/
                /**************************俯仰控制********************************/
                                    //X_MIS_n_L = X_MIS_n_L+25*STEP_20ms;
                                    if(g_Height<50.0&&flag_xh==0)
                                    {
                                            if (t_Flight<1.0)    //1s内仅有姿态
                                            {
                                                //过渡函数  //从起控点记录的俯仰角过度到15°
                                                ALFA_cmd = ALPHA_start*(1.0-TASK_MAIN::Transition(0.0,1.0,t_Flight))+15.0*(TASK_MAIN::Transition(0.0,1.0,t_Flight)); //俯仰角起控点--->15
                                                //记录上一步长的值             用于渐变
                                                time_old = t_Flight;     //记录时间
                                                VYold0 =u_VY_n;              //记录垂向速度
                                                H_cx =g_Height;          //记录垂向高度
                                                Hold =  g_Height;        //记录垂向高度
                                                ALFA_cmd_old0 = ALFA_cmd;//记录俯仰角指令
                                                date_hcmd=0;
                                                date_vycmd=0;
                                            }
                                            else if(g_Height<50.0)
                                            {
                                                        //3s内过渡	  合理过渡   从第一阶段结束的垂速过渡到3M/S
                                                        Vy_CX = VYold0*(1.0-TASK_MAIN::Transition(time_old,time_old+6.0,t_Flight))+3.0*TASK_MAIN::Transition(time_old,time_old+6.0,t_Flight);   //垂速指令
                                                        H_cx = H_cx+Vy_CX*STEP_20ms;   //频率20ms，50HZ       垂直高度  STEP_20ms
                                                        // ALFA_cmd =  ALFA_cmd_old0* (1.0-TASK_MAIN::Transition_function(time_old,time_old+1.0,t_Flight)) +
                                                        //     ( 11.0-(u_Ymis_n-H_cx)*0.8-(u_VY_n-Vy_CX)*0.6+1.0*(1.0-cos(u_GAMMA/57.3)) )*TASK_MAIN::Transition(time_old,time_old+1.0,t_Flight) ;
                                                        ALFA_cmd =   ALFA_cmd_old0-(u_Ymis_n-H_cx)*4.0-(u_VY_n-Vy_CX)*0.8+1.0*(1.0-cos(u_GAMMA/57.3));
                                                        //保存原始
                                                        VYold = u_VY_n;                                                                                                                                                       //纵向速度
                                                        H_cx1 = g_Height;                                                                                                                                       //垂直高度
                                                        time_old1 = t_Flight;                                                                                                                                 //
                                                        ALFA_cmd_old = ALFA_cmd ;
                                                        wz_cmd=0;
                                                        date_hcmd=H_cx;
                                                        date_vycmd=Vy_CX;
                                                        //增加过度过程
                                            }
                                    } 
                                    else{
                                        //30M以上巡航  标志位
                                        flag_xh = 1;
                                        //垂速控制指令  3S以内衰减为0
                                        Vy_CX = VYold*(1.0-TASK_MAIN::Transition(time_old1,time_old1+3.0,t_Flight));
                                        //时间
                                        H_cx1 = H_cx1+Vy_CX*STEP_20ms;//STEP_20ms
                                        JF_NY = (u_Ymis_n-H_cx1)*STEP_20ms+JF_NY;
                                        //1S内衰减
                                        ALFA_cmd =ALFA_cmd_old-(u_Ymis_n-H_cx1)*4.0-(u_VY_n-Vy_CX)*0.8+1.0*(1.0-cos(u_GAMMA/57.3))-JF_NY*0;
                                        date_hcmd=H_cx1;
                                        date_vycmd=Vy_CX;
                                }
                /**************************俯仰控制********************************/

                /**************************速度控制********************************/
                                    if (V_MIS<18.0&&flag_sd==0)
                                    {
                                            Fai_real=0.5;     // 油门为0.65   920kv   9*5桨
                                            date_vcmd=0;    
                                    }
                                    else
                                    {
                                            flag_sd =1;  //速度标志位
                                            JF_ma=JF_ma+(V_MIS-20.0)*STEP_20ms;//
                                            Fai_real = 0.5 - 0.1*(V_MIS-20.0) -0.08*JF_ma;  //速度PI
                                            Fai_real =limit2(Fai_real,1,0.2);
                                            date_vcmd=20*1.5;


                                            // Error_X_MIS = X_MIS_n_L-X_MIS_n;
                                            // JF_Error_X_MIS = JF_Error_X_MIS+Error_X_MIS*STEP_5ms;
                                            // V_MIS_F = 4*(Error_X_MIS - 30) + 1*(25-V_MIS)+ 0.01 * JF_Error_X_MIS ;   // 从机速度指令
                                            // V_MIS_F = limit2(V_MIS_F,30.0,23.0) ;  //从机限幅
                                            // JF_ma=JF_ma+(V_MIS-V_MIS_F)*STEP_5ms;   //速度控制  积分项
					                        // Fai_realpr =  1 - 0.2*(V_MIS-V_MIS_F) - 0.3*JF_ma;    //油门指令
                                    }
                /**************************速度控制********************************/
                /*************************切换航点********************************/
               TASK_MAIN::RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
               TASK_MAIN:: Navigation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
               //z_control 0  当前位置到期望航点的垂直距
				//          1 当前位置与当前航线的侧偏距离
				//          2 当前位置到期望航的绝对距离
				//          3 当段航线长度
				//          4 期望航向角
            //     cout<<"z0 当前位置到期望航点的垂直距:"<<z_control[0]<<endl;
            //     cout<<"m0:"<<m_control[0]<<endl;
            //    cout<<"z1 当前位置与当前航线的侧偏距离："<<z_control[1]<<endl;
            //    cout<<"m1:"<<m_control[1]<<endl;
            //     cout<<"z2 当前位置到期望航点的绝对距离："<<z_control[2]<<endl;
            //     cout<<"m2:"<<m_control[2]<<endl;
            //     cout<<"z4 期望航向角:"<<z_control[4]*Rad2Deg<<endl;
            //     cout<<"m4:"<<m_control[4]*Rad2Deg<<endl;
            //     //     cout<<"z2 当前位置到期望航的绝对距离"<<z_control[2]<<endl;
            //     //     cout<<"z3 当段航线长度"<<z_control[3]<<endl;
            //         // cout<<"z4 期望航向角"<<z_control[4]*Rad2Deg<<endl;
            //         cout<<"PSCI_t:"<<PSCI_t*Rad2Deg<<endl;
            //         cout<<"当前期望航点："<<flag_point+1<<endl;
            u_VZ = sin(z_control[4])*VN_x + cos(z_control[4])*VE_z ;
             if (z_control[0]<R_Target*tan(angle_Target/2*Deg2Rad)+1)   //入弯与撞线判断
                {
                    if (angle_Target>0)
                    {
                //计算转弯圆心的大地方位角   角度
                        double turnangle = -z_control[4]*Rad2Deg+90;
                //反结算圆心的经纬度    都是度 输出也是度
                                              //  TASK_MAIN::Vincent_Z(R_Target,turnangle,Long*Rad2Deg,Lat*Rad2Deg,&Turn_long,&Turn_lat);
                                                                             double S_MT=R_Target;
                                                                             double Alf_MT=turnangle;
                                                                             double g_Longitude=Long*Rad2Deg;
                                                                             double g_Latitude=Lat*Rad2Deg;
                                                                             double g_Longitude_T=Turn_long;
                                                                             double g_Latitude_T=Turn_lat;
                                                                            double E1,Aa,B,D,c,m,sigmam_2,tan_u1;
                                                                            double k1,sin_m;
                                                                            double u1,sigma1,sigma,sigma_b,delta_sigma,lamda;
                                                                            double x1,y1,x2,y2;
                                                                            double E = (1.0 / 298.257223563);
                                                                            double Pi = 3.14159265358979;
                                                                            double AtoR = 180. / Pi;
                                                                            double Re = 6378137.0;                //地球半径，(m)
                                                                            double Rb = (Re * (1 - E));			//Rb地球短半径
                                                                            double ee = (Re * Re - Rb * Rb ) / (Rb * Rb);


                                                                            tan_u1 = (1 - E) * tan(g_Latitude / AtoR);
                                                                            u1 = atan(tan_u1);
                                                                            sigma1 = atan(tan_u1 / cos(Alf_MT / AtoR));

                                                                            if( cos(Alf_MT / AtoR) < 0.0 && tan_u1 > 0.0 ) 
                                                                            {
                                                                                sigma1 = sigma1 + Pi; 
                                                                            }
                                                                            if( cos(Alf_MT / AtoR) < 0.0 && tan_u1 < 0.0 ) 
                                                                            {
                                                                                sigma1 = sigma1-Pi;
                                                                            }

                                                                            sin_m = cos(u1) * sin(Alf_MT / AtoR);
                                                                            m = asin(sin_m);

                                                                            k1 = (sqrt(1.0 + ee * (1.0 - sin_m * sin_m)) - 1.0) / (sqrt(1.0 + ee*(1.0 - sin_m * sin_m)) + 1.0);
                                                                            
                                                                            Aa = (1.0 + k1 * k1 / 4.0) / (1.0 - k1);
                                                                            B = k1 * (1.0 - 3.0 * k1 * k1 / 8.0);
                                                                            sigma = S_MT / (Rb * Aa);
                                                                            do
                                                                            {
                                                                                sigma_b  = sigma;
                                                                                sigmam_2 = 2 * sigma1 + sigma;
                                                                                
                                                                                
                                                                                delta_sigma = B * sin(sigma) * (cos(sigmam_2) + B * (E1 * cos(sigma) - D) / 4.0);
                                                                                sigma = S_MT / (Rb * Aa) + delta_sigma;

                                                                            } while(fabs(sigma - sigma_b) > 0.3e-11);
                                                                            
                                                                            g_Latitude_T = atan((sin(u1) * cos(sigma) + cos(u1) * sin(sigma) * cos(Alf_MT / AtoR))	/ ((1 - E) * sqrt(sin_m * sin_m	+ pow(sin(u1) * sin(sigma)- cos(u1) * cos(sigma) * cos(Alf_MT / AtoR),2.0))));
                                                                            
                                                                            x1 = sin(sigma) * sin(Alf_MT / AtoR);
                                                                            y1 = cos(u1) * cos(sigma) - sin(u1) * sin(sigma) * cos(Alf_MT / AtoR);
                                                                            lamda = atan(x1 / y1);
                                                                            
                                                                            if( y1 < 0.0 && x1 > 0.0)
                                                                            {
                                                                                lamda = lamda+Pi;
                                                                            }
                                                                            else if( y1 < 0 && x1 < 0.0 )
                                                                            { 
                                                                                lamda = lamda-Pi;
                                                                            }
                                                                            
                                                                            c = E * cos(m) * cos(m) * (4.0 + E * (4.0 - 3.0 * cos(m) * cos(m))) / 16.0;
                                                                            
                                                                            g_Longitude_T = g_Longitude / AtoR + lamda - (1.0 - c) * E * sin_m * (sigma + c * sin(sigma) * (cos(sigmam_2) + E1 * c * cos(sigma)));
                                                                            
                                                                            if(fabs(g_Longitude_T) > Pi)
                                                                            {
                                                                                if(g_Longitude_T > 0.0)
                                                                                {
                                                                                    g_Longitude_T = g_Longitude_T - 2.0 * Pi;
                                                                                }
                                                                                else
                                                                                {
                                                                                    g_Longitude_T = g_Longitude_T + 2.0 * Pi;
                                                                                }
                                                                            }
                                                                            
                                                                            x2 = -sin_m;
                                                                            y2 = sin(u1) * sin(sigma) - cos(u1) * cos(sigma) * cos(Alf_MT / AtoR);

                                                                            Turn_long = g_Longitude_T * AtoR;
                                                                            Turn_lat  = g_Latitude_T * AtoR;
                //转弯状态字变1      0的话始终直线飞行
                        flag_zw = 1;
                    }
                        flag_point +=1;
                        Lat_Target =  waypoint_qgc[flag_point][0];
                        Long_Target = waypoint_qgc[flag_point][1];
                        Lat_ini =  waypoint_qgc[flag_point-1][0];
                        Long_ini = waypoint_qgc[flag_point-1][1];
                }
                    //     cout<<fixed<< setprecision(10)<<Turn_long<<" "<<endl;
        // //发布数据
        //  fw_4cmd.roll_angle_sp=0;
        //  fw_4cmd.pitch_angle_sp =15*Deg2Rad;
        //  fw_4cmd.yaw_angle_sp =0;
        //  fw_4cmd.throttle_sp=0.65;
                    //  cout<<fixed<< setprecision(10)<<Turn_lat<<" "<<endl;
                 /*****************************滚转控制**************************/
                  // TASK_MAIN::RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
                    TASK_MAIN:: Navigation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
                  int ki_a=0;
                 if(flag_zw==0)
                 {
                        if(abs(z_control[1])>2)
                        {
                            ki_a=0;
                            JF_NZ=0;
                        }
                        else
                        {
                            ki_a=1;
                            JF_NZ = (z_control[1]-0.0)*STEP_20ms+JF_NZ;
                        }  
                        GAMMAcx = -z_control[1]*1.0-u_VZ*2.0-JF_NZ*0.04*ki_a;  //积分
                        cout<<"直线控制："<<endl;
                        JF_Zmis_n = 0 ;
                        date_delta_perr=-z_control[1]/1.5;
                        date_delta_verr=-u_VZ;

                        // F_NZ = (z_control[1]-20)*STEP_5ms+JF_NZ;   //使用侧向位置进行控制
			            // GAMMAcxpr = -(z_control[1]-20)*0.75-u_VZ*1.5-JF_NZ*0.04;  //滚转角指令
			
			            // GAMMAcxpr = limit1(GAMMAcxpr,45);


                 }else
                 {
                        JF_NZ = 0; 
                        //Vincenty_F(Turn_long , Turn_lat, Long*Rad2Deg ,Lat*Rad2Deg,&disturn,&aturn); //反解距离目标点距离，与方位角
                                                                                    double Long_T=Turn_long;
                                                                                    double Lat_T=Turn_lat;
                                                                                    double g_Longitude= Long*Rad2Deg;
                                                                                    double g_Latitude=Lat*Rad2Deg;
                                                                                    double g_R_MT=disturn;
                                                                                    double g_Alpha12_MT=aturn;
                                                                                    double U1,U2,detLong,detLongd,sigma,sigamm_2,sigma_sin,sigma_cos,mm;	//单位：弧度
                                                                                    double C,E1,K11,B,AA,D,detsigam,Alf_MT1,Alf_MT2;
                                                                                    double E = (1.0 / 298.257223563);
                                                                                    double Pi = 3.14159265358979;
                                                                                    double AtoR = 180. / Pi;
                                                                                    double Re = 6378137.0;                //地球半径，(m)
                                                                                    double Rb = (Re * (1 - E));			//Rb地球短半径
                                                                                    double ee = (Re * Re - Rb * Rb ) / (Rb * Rb);

                                                                                    
                                                                                    U1 = atan((1.0- E) * tan(g_Latitude / AtoR));	//单位：弧度
                                                                                    U2 = atan((1.0- E) * tan(Lat_T / AtoR));	    //单位：弧度
                                                                                    
                                                                                    detLong	= (Long_T - g_Longitude) / AtoR;
                                                                                    while (1) 
                                                                                    {
                                                                                        detLongd	= detLong;	
                                                                                        sigma_sin	= sqrt(cos(U2) * cos(U2) * sin(detLong) * sin(detLong) + (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong)) * (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong)));
                                                                                        sigma_cos	= sin(U1) * sin(U2) + cos(U1) * cos(U2) * cos(detLong);
                                                                                        sigma		= atan(sigma_sin / sigma_cos);
                                                                                        
                                                                                        mm = asin( cos(U1) * cos(U2) * sin(detLong) / sigma_sin);
                                                                                        C  = E * cos(mm) * cos(mm) * (4.0 + E * (4.0 - 3.0 * cos(mm) * cos(mm))) / 16.0;
                                                                                        
                                                                                        sigamm_2 = acos(sigma_cos - 2.0 * sin(U1) * sin(U2) / (cos(mm) * cos(mm)));
                                                                                        E1  = 2.0 * cos(sigamm_2) * cos(sigamm_2) - 1.0;
                                                                                        
                                                                                        detLong = (Long_T - g_Longitude) / AtoR + (1.0 - C) * E * sin(mm) * (sigma + C * sigma_sin * (cos(sigamm_2) + E1 * C * sigma_cos));
                                                                                        if (fabs(detLong - detLongd) <= 0.3e-11)
                                                                                        {
                                                                                            break;
                                                                                        }
                                                                                    }

                                                                                    K11 = (sqrt(1.0 + ee * cos(mm) * cos(mm)) - 1.0) / (sqrt(1.0 + ee * cos(mm) * cos(mm)) + 1.0);
                                                                                    B   = K11 * (1.0 - 3.0 * K11 * K11 / 8.0);
                                                                                    AA  = (1.0 + K11 * K11 / 4.0) / (1.0 - K11);
                                                                                    D	= B * cos(sigamm_2) * (4.0 * sigma_sin * sigma_sin - 3.0) * (2.0 * E1 - 1) / 6.0;
                                                                                    
                                                                                    detsigam = B * sigma_sin * (cos(sigamm_2) + B * (E1 * sigma_cos - D) / 4.0);
                                                                                    
                                                                                    //大地距离
                                                                                    disturn = (sigma - detsigam) * Rb * AA;
                                                                                    
                                                                                    //大地方位角（正方位角）
                                                                                    Alf_MT1 = cos(U2) * sin(detLong);
                                                                                    Alf_MT2 = cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong);
                                                                                    g_Alpha12_MT	= atan(Alf_MT1 / Alf_MT2);
                                                                                    
                                                                                    if (Alf_MT2 < 0.0)
                                                                                    {
                                                                                        g_Alpha12_MT = Pi + g_Alpha12_MT;
                                                                                    }
                                                                                    else if (Alf_MT1 < 0.0)
                                                                                    {
                                                                                        g_Alpha12_MT = 2.0 * Pi + g_Alpha12_MT;
                                                                                    }
                                                                                    
                                                                                    aturn = g_Alpha12_MT * AtoR;                        
                        u_VZ_zw = cos(aturn*Deg2Rad)*VN_x + sin(aturn*Deg2Rad)*VE_z ;   //北向动向速度
                        JF_Zmis_n = JF_Zmis_n + (R_Target-disturn)* STEP_20ms;     //积分
                        GAMMAcx = -2.5 * (R_Target-disturn) - 2.0*(u_VZ_zw) - JF_Zmis_n * 0.1+30;
                        cout<<"转弯控制："<<endl;
                         JF_NZ=0;

                        date_delta_perr=-(R_Target-disturn)/1.5;
                        date_delta_verr=-u_VZ_zw;
                        if (z_control[1]<2.0||abs(PSCI_t-z_control[4])<2.0*Deg2Rad)
                        {
                            flag_zw = 0;
                        }

                 }

                    // roll_cmd=GAMMAcx;
                    // pitch_cmd=ALFA_cmd;
                    // throttle_cmd=Fai_real;
     }

}

void TASK_MAIN::control_takeoff2()
{

    if(start_control==0)
    {
             cout<<"*********计算领机的侧偏位置*************"<<endl;
             throttle_cmd= 0.6;
            roll_cmd=0.0;
            pitch_cmd=20.0;
            yaw_cmd=0.0;
             if(thisfw_states.pitch_angle*Rad2Deg>=18.0)
                {
                        start_control=1;
                        //waypoint_qgc数组都是弧度
                        //角度都是弧度
                        //thisfw_status中经纬为°，角度是弧度
                        for (size_t i = 0; i < current_waypoints.waypoints.size(); i++)
                        {
                            if(current_waypoints.waypoints[i].command==16)//16普通航路点   22起飞航点 单位rad
                            {
                                POINT_NUM_QGC++;//0为无人机开始位置，1为第一个普通航点，起飞点不要  Sin cos计算用rad
                                waypoint_qgc[POINT_NUM_QGC][0]=current_waypoints.waypoints[i].x_lat*Deg2Rad;
                                waypoint_qgc[POINT_NUM_QGC][1]=current_waypoints.waypoints[i].y_long*Deg2Rad;
                            }
                        }
                        waypoint_qgc[0][0]=thisfw_states.latitude*Deg2Rad;
                        waypoint_qgc[0][1]=thisfw_states.longitude*Deg2Rad;
                        start_time=current_time;
                        Lat_ini=thisfw_states.latitude*Deg2Rad;
                        Long_ini=thisfw_states.longitude*Deg2Rad;
                        Lat_Target=waypoint_qgc[1][0];
                        Long_Target=waypoint_qgc[1][1];
                        ALPHA_start=thisfw_states.pitch_angle*Rad2Deg;
                        R_Target  = 100.0;          //转弯半径
                        angle_Target=90;
                        start_height=thisfw_states.relative_alt;
                        }
     }
     

     if(start_control==1)
     {
                cout<<"*******进入起控start_control=11111**************"<<endl;
                /**************************状态更新********************************/
                            t_Flight=current_time-start_time;
                            g_Height=thisfw_states.relative_alt-start_height;
                            u_Ymis_n=thisfw_states.relative_alt-start_height;
                            u_GAMMA=thisfw_states.roll_angle*Rad2Deg;
                            u_THETA=thisfw_states.pitch_angle*Rad2Deg;
                            V_MIS=thisfw_states.air_speed;
                            VN_x=thisfw_states.ned_vel_x;
                            VE_z=thisfw_states.ned_vel_y;
                            u_VY_n=-thisfw_states.ned_vel_z;
                            Lat=thisfw_states.latitude*Deg2Rad;
                            Long=thisfw_states.longitude*Deg2Rad;
                            PSCI_t=thisfw_states.yaw_angle;
                            if(PSCI_t>=0.0&&PSCI_t<=PI)
                                PSCI_t=-PSCI_t;
                           else if(PSCI_t>PI&&PSCI_t<2.0*PI)
                                PSCI_t=-1.0*(PSCI_t-2.0*PI);
                /**************************状态更新********************************/
                /**************************俯仰控制********************************/
                                    //X_MIS_n_L = X_MIS_n_L+25*STEP_20ms;
                                    if(g_Height<50.0&&flag_xh==0)
                                    {
                                            if (t_Flight<1.0)    //1s内仅有姿态
                                            {
                                                //过渡函数  //从起控点记录的俯仰角过度到15°
                                                ALFA_cmd = ALPHA_start*(1.0-TASK_MAIN::Transition(0.0,1.0,t_Flight))+15.0*(TASK_MAIN::Transition(0.0,1.0,t_Flight)); //俯仰角起控点--->15
                                                //记录上一步长的值             用于渐变
                                                time_old = t_Flight;     //记录时间
                                                VYold0 =u_VY_n;              //记录垂向速度
                                                H_cx =g_Height;          //记录垂向高度
                                                Hold =  g_Height;        //记录垂向高度
                                                ALFA_cmd_old0 = ALFA_cmd;//记录俯仰角指令
                                                date_hcmd=0;
                                                date_vycmd=0;
                                            }
                                            else if(g_Height<50.0)
                                            {
                                                        //3s内过渡	  合理过渡   从第一阶段结束的垂速过渡到3M/S
                                                        Vy_CX = VYold0*(1.0-TASK_MAIN::Transition(time_old,time_old+6.0,t_Flight))+3.0*TASK_MAIN::Transition(time_old,time_old+6.0,t_Flight);   //垂速指令
                                                        H_cx = H_cx+Vy_CX*STEP_20ms;   //频率20ms，50HZ       垂直高度  STEP_20ms
                                                        // ALFA_cmd =  ALFA_cmd_old0* (1.0-TASK_MAIN::Transition_function(time_old,time_old+1.0,t_Flight)) +
                                                        //     ( 11.0-(u_Ymis_n-H_cx)*0.8-(u_VY_n-Vy_CX)*0.6+1.0*(1.0-cos(u_GAMMA/57.3)) )*TASK_MAIN::Transition(time_old,time_old+1.0,t_Flight) ;
                                                        ALFA_cmd =   ALFA_cmd_old0-(u_Ymis_n-H_cx)*4.0-(u_VY_n-Vy_CX)*0.8+1.0*(1.0-cos(u_GAMMA/57.3));
                                                        //保存原始
                                                        VYold = u_VY_n;                                                                                                                                                       //纵向速度
                                                        H_cx1 = g_Height;                                                                                                                                       //垂直高度
                                                        time_old1 = t_Flight;                                                                                                                                 //
                                                        ALFA_cmd_old = ALFA_cmd ;
                                                        wz_cmd=0;
                                                        date_hcmd=H_cx;
                                                        date_vycmd=Vy_CX;
                                                        //增加过度过程
                                            }
                                    } 
                                    else{
                                        //30M以上巡航  标志位
                                        flag_xh = 1;
                                        //垂速控制指令  3S以内衰减为0
                                        Vy_CX = VYold*(1.0-TASK_MAIN::Transition(time_old1,time_old1+3.0,t_Flight));
                                        //时间
                                        H_cx1 = H_cx1+Vy_CX*STEP_20ms;//STEP_20ms
                                        JF_NY = (u_Ymis_n-H_cx1)*STEP_20ms+JF_NY;
                                        //1S内衰减
                                        ALFA_cmd =ALFA_cmd_old-(u_Ymis_n-H_cx1)*4.0-(u_VY_n-Vy_CX)*0.8+1.0*(1.0-cos(u_GAMMA/57.3))-JF_NY*0;
                                        date_hcmd=H_cx1;
                                        date_vycmd=Vy_CX;
                                }
                /**************************俯仰控制********************************/

                /**************************速度控制********************************/
                                    if (V_MIS<18.0&&flag_sd==0)
                                    {
                                            Fai_real=0.5;     // 油门为0.65   920kv   9*5桨
                                            date_vcmd=0;    
                                    }
                                    else
                                    {
                                            flag_sd =1;  //速度标志位
                                            JF_ma=JF_ma+(V_MIS-20.0)*STEP_20ms;//
                                            Fai_real = 0.5 - 0.1*(V_MIS-20.0) -0.08*JF_ma;  //速度PI
                                            Fai_real =limit2(Fai_real,1,0.2);
                                            date_vcmd=20*1.5;


                                            // Error_X_MIS = X_MIS_n_L-X_MIS_n;
                                            // JF_Error_X_MIS = JF_Error_X_MIS+Error_X_MIS*STEP_5ms;
                                            // V_MIS_F = 4*(Error_X_MIS - 30) + 1*(25-V_MIS)+ 0.01 * JF_Error_X_MIS ;   // 从机速度指令
                                            // V_MIS_F = limit2(V_MIS_F,30.0,23.0) ;  //从机限幅
                                            // JF_ma=JF_ma+(V_MIS-V_MIS_F)*STEP_5ms;   //速度控制  积分项
					                        // Fai_realpr =  1 - 0.2*(V_MIS-V_MIS_F) - 0.3*JF_ma;    //油门指令
                                    }
                /**************************速度控制********************************/
                /*************************切换航点********************************/
               TASK_MAIN::RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
               TASK_MAIN:: Navigation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
               //z_control 0  当前位置到期望航点的垂直距
				//          1 当前位置与当前航线的侧偏距离
				//          2 当前位置到期望航的绝对距离
				//          3 当段航线长度
				//          4 期望航向角
                cout<<"z0 当前位置到期望航点的垂直距:"<<z_control[0]<<endl;
                cout<<"m0:"<<m_control[0]<<endl;
               cout<<"z1 当前位置与当前航线的侧偏距离："<<z_control[1]<<endl;
               cout<<"m1:"<<m_control[1]<<endl;
                cout<<"z2 当前位置到期望航点的绝对距离："<<z_control[2]<<endl;
                cout<<"m2:"<<m_control[2]<<endl;
                cout<<"z4 期望航向角:"<<z_control[4]*Rad2Deg<<endl;
                cout<<"m4:"<<m_control[4]*Rad2Deg<<endl;
                //     cout<<"z2 当前位置到期望航的绝对距离"<<z_control[2]<<endl;
                //     cout<<"z3 当段航线长度"<<z_control[3]<<endl;
                    // cout<<"z4 期望航向角"<<z_control[4]*Rad2Deg<<endl;
                    cout<<"PSCI_t:"<<PSCI_t*Rad2Deg<<endl;
                    cout<<"当前期望航点："<<flag_point+1<<endl;
            u_VZ = sin(z_control[4])*VN_x + cos(z_control[4])*VE_z ;
             if (z_control[0]<R_Target*tan(angle_Target/2*Deg2Rad)+1)   //入弯与撞线判断
                {
                    if (angle_Target>0)
                    {
                //计算转弯圆心的大地方位角   角度
                        double turnangle = -z_control[4]*Rad2Deg+90;
                //反结算圆心的经纬度    都是度 输出也是度
                                              //  TASK_MAIN::Vincent_Z(R_Target,turnangle,Long*Rad2Deg,Lat*Rad2Deg,&Turn_long,&Turn_lat);
                                                                             double S_MT=R_Target;
                                                                             double Alf_MT=turnangle;
                                                                             double g_Longitude=Long*Rad2Deg;
                                                                             double g_Latitude=Lat*Rad2Deg;
                                                                             double g_Longitude_T=Turn_long;
                                                                             double g_Latitude_T=Turn_lat;
                                                                            double E1,Aa,B,D,c,m,sigmam_2,tan_u1;
                                                                            double k1,sin_m;
                                                                            double u1,sigma1,sigma,sigma_b,delta_sigma,lamda;
                                                                            double x1,y1,x2,y2;
                                                                            double E = (1.0 / 298.257223563);
                                                                            double Pi = 3.14159265358979;
                                                                            double AtoR = 180. / Pi;
                                                                            double Re = 6378137.0;                //地球半径，(m)
                                                                            double Rb = (Re * (1 - E));			//Rb地球短半径
                                                                            double ee = (Re * Re - Rb * Rb ) / (Rb * Rb);


                                                                            tan_u1 = (1 - E) * tan(g_Latitude / AtoR);
                                                                            u1 = atan(tan_u1);
                                                                            sigma1 = atan(tan_u1 / cos(Alf_MT / AtoR));

                                                                            if( cos(Alf_MT / AtoR) < 0.0 && tan_u1 > 0.0 ) 
                                                                            {
                                                                                sigma1 = sigma1 + Pi; 
                                                                            }
                                                                            if( cos(Alf_MT / AtoR) < 0.0 && tan_u1 < 0.0 ) 
                                                                            {
                                                                                sigma1 = sigma1-Pi;
                                                                            }

                                                                            sin_m = cos(u1) * sin(Alf_MT / AtoR);
                                                                            m = asin(sin_m);

                                                                            k1 = (sqrt(1.0 + ee * (1.0 - sin_m * sin_m)) - 1.0) / (sqrt(1.0 + ee*(1.0 - sin_m * sin_m)) + 1.0);
                                                                            
                                                                            Aa = (1.0 + k1 * k1 / 4.0) / (1.0 - k1);
                                                                            B = k1 * (1.0 - 3.0 * k1 * k1 / 8.0);
                                                                            sigma = S_MT / (Rb * Aa);
                                                                            do
                                                                            {
                                                                                sigma_b  = sigma;
                                                                                sigmam_2 = 2 * sigma1 + sigma;
                                                                                
                                                                                
                                                                                delta_sigma = B * sin(sigma) * (cos(sigmam_2) + B * (E1 * cos(sigma) - D) / 4.0);
                                                                                sigma = S_MT / (Rb * Aa) + delta_sigma;

                                                                            } while(fabs(sigma - sigma_b) > 0.3e-11);
                                                                            
                                                                            g_Latitude_T = atan((sin(u1) * cos(sigma) + cos(u1) * sin(sigma) * cos(Alf_MT / AtoR))	/ ((1 - E) * sqrt(sin_m * sin_m	+ pow(sin(u1) * sin(sigma)- cos(u1) * cos(sigma) * cos(Alf_MT / AtoR),2.0))));
                                                                            
                                                                            x1 = sin(sigma) * sin(Alf_MT / AtoR);
                                                                            y1 = cos(u1) * cos(sigma) - sin(u1) * sin(sigma) * cos(Alf_MT / AtoR);
                                                                            lamda = atan(x1 / y1);
                                                                            
                                                                            if( y1 < 0.0 && x1 > 0.0)
                                                                            {
                                                                                lamda = lamda+Pi;
                                                                            }
                                                                            else if( y1 < 0 && x1 < 0.0 )
                                                                            { 
                                                                                lamda = lamda-Pi;
                                                                            }
                                                                            
                                                                            c = E * cos(m) * cos(m) * (4.0 + E * (4.0 - 3.0 * cos(m) * cos(m))) / 16.0;
                                                                            
                                                                            g_Longitude_T = g_Longitude / AtoR + lamda - (1.0 - c) * E * sin_m * (sigma + c * sin(sigma) * (cos(sigmam_2) + E1 * c * cos(sigma)));
                                                                            
                                                                            if(fabs(g_Longitude_T) > Pi)
                                                                            {
                                                                                if(g_Longitude_T > 0.0)
                                                                                {
                                                                                    g_Longitude_T = g_Longitude_T - 2.0 * Pi;
                                                                                }
                                                                                else
                                                                                {
                                                                                    g_Longitude_T = g_Longitude_T + 2.0 * Pi;
                                                                                }
                                                                            }
                                                                            
                                                                            x2 = -sin_m;
                                                                            y2 = sin(u1) * sin(sigma) - cos(u1) * cos(sigma) * cos(Alf_MT / AtoR);

                                                                            Turn_long = g_Longitude_T * AtoR;
                                                                            Turn_lat  = g_Latitude_T * AtoR;
                //转弯状态字变1      0的话始终直线飞行
                        flag_zw = 1;
                    }
                        flag_point +=1;
                        Lat_Target =  waypoint_qgc[flag_point][0];
                        Long_Target = waypoint_qgc[flag_point][1];
                        Lat_ini =  waypoint_qgc[flag_point-1][0];
                        Long_ini = waypoint_qgc[flag_point-1][1];
                }
                    //     cout<<fixed<< setprecision(10)<<Turn_long<<" "<<endl;
        // //发布数据
        //  fw_4cmd.roll_angle_sp=0;
        //  fw_4cmd.pitch_angle_sp =15*Deg2Rad;
        //  fw_4cmd.yaw_angle_sp =0;
        //  fw_4cmd.throttle_sp=0.65;
                    //  cout<<fixed<< setprecision(10)<<Turn_lat<<" "<<endl;
                 /*****************************滚转控制**************************/
                  // TASK_MAIN::RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
                    TASK_MAIN:: Navigation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);
                  int ki_a=0;
                 if(flag_zw==0)
                 {
                        if(abs(z_control[1])>2)
                        {
                            ki_a=0;
                            JF_NZ=0;
                        }
                        else
                        {
                            ki_a=1;
                            JF_NZ = (z_control[1]-0.0)*STEP_20ms+JF_NZ;
                        }  
                        GAMMAcx = -z_control[1]*1.0-u_VZ*2.0-JF_NZ*0.04*ki_a;  //积分
                        cout<<"直线控制："<<endl;
                        JF_Zmis_n = 0 ;
                        date_delta_perr=-z_control[1]/1.5;
                        date_delta_verr=-u_VZ;

                        // F_NZ = (z_control[1]-20)*STEP_5ms+JF_NZ;   //使用侧向位置进行控制
			            // GAMMAcxpr = -(z_control[1]-20)*0.75-u_VZ*1.5-JF_NZ*0.04;  //滚转角指令
			
			            // GAMMAcxpr = limit1(GAMMAcxpr,45);


                 }else
                 {
                        JF_NZ = 0; 
                        //Vincenty_F(Turn_long , Turn_lat, Long*Rad2Deg ,Lat*Rad2Deg,&disturn,&aturn); //反解距离目标点距离，与方位角
                                                                                    double Long_T=Turn_long;
                                                                                    double Lat_T=Turn_lat;
                                                                                    double g_Longitude= Long*Rad2Deg;
                                                                                    double g_Latitude=Lat*Rad2Deg;
                                                                                    double g_R_MT=disturn;
                                                                                    double g_Alpha12_MT=aturn;
                                                                                    double U1,U2,detLong,detLongd,sigma,sigamm_2,sigma_sin,sigma_cos,mm;	//单位：弧度
                                                                                    double C,E1,K11,B,AA,D,detsigam,Alf_MT1,Alf_MT2;
                                                                                    double E = (1.0 / 298.257223563);
                                                                                    double Pi = 3.14159265358979;
                                                                                    double AtoR = 180. / Pi;
                                                                                    double Re = 6378137.0;                //地球半径，(m)
                                                                                    double Rb = (Re * (1 - E));			//Rb地球短半径
                                                                                    double ee = (Re * Re - Rb * Rb ) / (Rb * Rb);

                                                                                    
                                                                                    U1 = atan((1.0- E) * tan(g_Latitude / AtoR));	//单位：弧度
                                                                                    U2 = atan((1.0- E) * tan(Lat_T / AtoR));	    //单位：弧度
                                                                                    
                                                                                    detLong	= (Long_T - g_Longitude) / AtoR;
                                                                                    while (1) 
                                                                                    {
                                                                                        detLongd	= detLong;	
                                                                                        sigma_sin	= sqrt(cos(U2) * cos(U2) * sin(detLong) * sin(detLong) + (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong)) * (cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong)));
                                                                                        sigma_cos	= sin(U1) * sin(U2) + cos(U1) * cos(U2) * cos(detLong);
                                                                                        sigma		= atan(sigma_sin / sigma_cos);
                                                                                        
                                                                                        mm = asin( cos(U1) * cos(U2) * sin(detLong) / sigma_sin);
                                                                                        C  = E * cos(mm) * cos(mm) * (4.0 + E * (4.0 - 3.0 * cos(mm) * cos(mm))) / 16.0;
                                                                                        
                                                                                        sigamm_2 = acos(sigma_cos - 2.0 * sin(U1) * sin(U2) / (cos(mm) * cos(mm)));
                                                                                        E1  = 2.0 * cos(sigamm_2) * cos(sigamm_2) - 1.0;
                                                                                        
                                                                                        detLong = (Long_T - g_Longitude) / AtoR + (1.0 - C) * E * sin(mm) * (sigma + C * sigma_sin * (cos(sigamm_2) + E1 * C * sigma_cos));
                                                                                        if (fabs(detLong - detLongd) <= 0.3e-11)
                                                                                        {
                                                                                            break;
                                                                                        }
                                                                                    }

                                                                                    K11 = (sqrt(1.0 + ee * cos(mm) * cos(mm)) - 1.0) / (sqrt(1.0 + ee * cos(mm) * cos(mm)) + 1.0);
                                                                                    B   = K11 * (1.0 - 3.0 * K11 * K11 / 8.0);
                                                                                    AA  = (1.0 + K11 * K11 / 4.0) / (1.0 - K11);
                                                                                    D	= B * cos(sigamm_2) * (4.0 * sigma_sin * sigma_sin - 3.0) * (2.0 * E1 - 1) / 6.0;
                                                                                    
                                                                                    detsigam = B * sigma_sin * (cos(sigamm_2) + B * (E1 * sigma_cos - D) / 4.0);
                                                                                    
                                                                                    //大地距离
                                                                                    disturn = (sigma - detsigam) * Rb * AA;
                                                                                    
                                                                                    //大地方位角（正方位角）
                                                                                    Alf_MT1 = cos(U2) * sin(detLong);
                                                                                    Alf_MT2 = cos(U1) * sin(U2) - sin(U1) * cos(U2) * cos(detLong);
                                                                                    g_Alpha12_MT	= atan(Alf_MT1 / Alf_MT2);
                                                                                    
                                                                                    if (Alf_MT2 < 0.0)
                                                                                    {
                                                                                        g_Alpha12_MT = Pi + g_Alpha12_MT;
                                                                                    }
                                                                                    else if (Alf_MT1 < 0.0)
                                                                                    {
                                                                                        g_Alpha12_MT = 2.0 * Pi + g_Alpha12_MT;
                                                                                    }
                                                                                    
                                                                                    aturn = g_Alpha12_MT * AtoR;                        
                        u_VZ_zw = cos(aturn*Deg2Rad)*VN_x + sin(aturn*Deg2Rad)*VE_z ;   //北向动向速度
                        JF_Zmis_n = JF_Zmis_n + (R_Target-disturn)* STEP_20ms;     //积分
                        GAMMAcx = -2.5 * (R_Target-disturn) - 2.0*(u_VZ_zw) - JF_Zmis_n * 0.1+30;
                        cout<<"转弯控制："<<endl;
                         JF_NZ=0;

                        date_delta_perr=-(R_Target-disturn)/1.5;
                        date_delta_verr=-u_VZ_zw;
                        if (z_control[1]<2.0||abs(PSCI_t-z_control[4])<2.0*Deg2Rad)
                        {
                            flag_zw = 0;
                        }

                 }

                    roll_cmd=GAMMAcx;
                    pitch_cmd=ALFA_cmd;
                    throttle_cmd=Fai_real;
     }




        //数据保存
        date_pitch=u_THETA;
        date_pitchcmd=pitch_cmd;
        date_roll=u_GAMMA;
        date_rollcmd=roll_cmd;
        date_v=V_MIS*1.5;
        date_h=g_Height;
        date_vy=u_VY_n;
        date_yaw=PSCI_t*Rad2Deg;
        date_yawcmd=heading_sp*Rad2Deg;
        date_delta_gd=gD/1.5;



        // fw_4cmd.date_flag_zw=flag_zw;
        //  fw_4cmd.t_flight=t_Flight;
        // fw_4cmd.date_pitch=date_pitch;
        // fw_4cmd.date_pitchcmd=date_pitchcmd;
        // fw_4cmd.date_roll=date_roll;
        // fw_4cmd.date_rollcmd=date_rollcmd;
        // fw_4cmd.date_v=date_v;
        // fw_4cmd.date_vcmd=date_vcmd;
        // fw_4cmd.date_h=date_h;
        // fw_4cmd.date_hcmd=date_hcmd;
        // fw_4cmd.date_vy=date_vy;
        // fw_4cmd.date_vycmd=date_vycmd;
        // fw_4cmd.date_delta_perr=date_delta_perr;
        // fw_4cmd.date_delta_verr=date_delta_verr;
        // fw_4cmd.date_delta_gd=date_delta_gd;
        // fw_4cmd.date_yaw=date_yaw;
        // fw_4cmd.date_yawcmd=date_yawcmd;

         fw_4cmd.z0=z_control[0];
         fw_4cmd.z1=z_control[1];
         fw_4cmd.z2=z_control[2];
         fw_4cmd.z3=z_control[3];
         fw_4cmd.z4=z_control[4];

         fw_4cmd.roll_angle_sp=roll_cmd*Deg2Rad;
        fw_4cmd.pitch_angle_sp =pitch_cmd*Deg2Rad;
        fw_4cmd.yaw_angle_sp =0;
         fw_4cmd.throttle_sp=throttle_cmd;//0.65;//


        // //发布数据
        //  fw_4cmd.roll_angle_sp=0;
        //  fw_4cmd.pitch_angle_sp =15*Deg2Rad;
        //  fw_4cmd.yaw_angle_sp =0;
        //  fw_4cmd.throttle_sp=0.65;

          fw_cmd_pub.publish(fw_4cmd); 
}


void TASK_MAIN::control_takeoff1()
{
    if(start_control==0)
    {
            throttle_cmd= 1;
            roll_cmd=0;
            pitch_cmd=30;
            yaw_cmd=0;
             cout<<"thisfw_states.pitch_angle*Rad2Deg"<<thisfw_states.pitch_angle*Rad2Deg<<endl;
             cout<<"start_control"<<start_control<<endl;
             if(thisfw_states.pitch_angle*Rad2Deg>=29.0)
                    {
                        cout<<"333333333333333333333"<<endl;
                        start_control=1;
                        //waypoint_qgc数组都是弧度
                        //角度都是弧度
                        //thisfw_status中经纬为°，角度是弧度
                        waypoint_qgc[0][0]=thisfw_states.latitude*Deg2Rad;
                        waypoint_qgc[0][1]=thisfw_states.longitude*Deg2Rad;
                        start_time=current_time;
                        Lat_ini=thisfw_states.latitude*Deg2Rad;
                        Long_ini=thisfw_states.longitude*Deg2Rad;
                        Lat_Target=waypoint_qgc[1][0];
                        Long_Target=waypoint_qgc[1][1];
                        ALPHA_start=thisfw_states.pitch_angle*Rad2Deg;
                        R_Target  = 180.0;          //转弯半径
                        angle_Target=90;
                        start_height=thisfw_states.relative_alt;
                        cout<<"start_control"<<start_control<<endl;
                            cout<<"start_time"<<start_control<<endl;
                        }
            }
  if(start_control>1)
    {
              t_Flight=current_time-start_time;
              g_Height=thisfw_states.relative_alt-start_height;
              u_Ymis_n=thisfw_states.relative_alt-start_height;
              u_GAMMA=thisfw_states.roll_angle*Rad2Deg;
              V_MIS=thisfw_states.air_speed;
              VN_x=thisfw_states.ned_vel_x;
              VE_z=thisfw_states.ned_vel_y;
              u_VY_n=-thisfw_states.ned_vel_z;
              Lat=thisfw_states.latitude*Deg2Rad;
              Long=thisfw_states.longitude*Deg2Rad;
              PSCI_t=thisfw_states.yaw_angle;
                if(PSCI_t>=0.0&&PSCI_t<=PI)
                    PSCI_t=-PSCI_t;
                else if(PSCI_t>PI&&PSCI_t<2.0*PI)
                    PSCI_t=-1.0*(PSCI_t-2.0*PI);
 	           // RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target);	//z_control 0  当前位置到期望航点的垂直距
																			//          1 当前位置与当前航线的侧偏距离
																			//          2 当前位置到期望航的绝对距离
																			//          3 当段航线长度
																			//          4 期望航向角
            //  if (z_control[0]<R_Target*tan(angle_Target/2*Deg2Rad)+1&&flag_point<5)   //入弯与撞线判断
            //     {

            //         if (angle_Target>0)
            //         {
            //     //计算转弯圆心的大地方位角   角度
            //             double turnangle = -z_control[4]*Rad2Deg+90;
            //     //反结算圆心的经纬度    都是度 输出也是度
            //            // Vincent_Z(R_Target,turnangle,Long*Rad2Deg,Lat*Rad2Deg,&Turn_long,&Turn_lat);
            //     //转弯状态字变1      0的话始终直线飞行
            //             flag_zw = 1;
            //         }
            //         flag_point +=1;
            //         Lat_Target =  waypoint_qgc[flag_point][0];
            //         Long_Target = waypoint_qgc[flag_point][1];
            //         //angle_Target = angle_Target_arr[flag_point];
            //         Lat_ini =  waypoint_qgc[flag_point-1][0];
            //         Long_ini = waypoint_qgc[flag_point-1][0];
            //     }

                if ((g_Height<50.0)&&flag_xh==0)
                {
                            cout<<"11111111111111111111111"<<endl;
                            cout<<"t_Flight"<<t_Flight<<endl;

                            if (t_Flight<5.0)    //1s内仅有姿态
                            {
                                cout<<"222222222222222222222"<<endl;
                                //过渡函数  //从起控点记录的俯仰角过度到15°
                                ALFA_cmd = ALPHA_start*(1.0-Transition_function(0.0,5.0,t_Flight))+15.0*(Transition_function(0.0,5.0,t_Flight)); //俯仰角起控点--->15
                                //ALFA_cmd = 30.0*(1.0-Transition(0.0,1.0,t_Flight))+15.0*(Transition(0.0,1.0,t_Flight)); //俯仰角30--->15
                                //ALFA_cmd = limit2(ALFA_cmd,THETA_v*Rad2Deg+10,THETA_v*Rad2Deg-2);             //   弹道倾角+10   -2
                                wz_cmd   =0;//俯仰角速度
                                //记录上一步长的值             用于渐变
                                time_old = t_Flight;     //记录时间
                                VYold0 =VY;              //记录垂向速度
                                H_cx =g_Height;          //记录垂向高度
                                Hold =  g_Height;        //记录垂向高度
                                ALFA_cmd_old0 = ALFA_cmd;//记录俯仰角指令
                                cout<<"ALPHA_start*(1.0-Transition_function(0.0,1.0,t_Flight))"<<ALPHA_start*(1.0-Transition_function(0.0,1.0,t_Flight))<<endl;
                                cout<<"15.0*(Transition_function(0.0,1.0,t_Flight))"<<15.0*(Transition_function(0.0,1.0,t_Flight))<<endl;
                            }
                            else if(g_Height<50.0)
                            {
                                         //3s内过渡	  合理过渡   从第一阶段结束的垂速过渡到3M/S
                                        Vy_CX = VYold0*(1.0-Transition_function(time_old,time_old+6.0,t_Flight))+3.0*Transition_function(time_old,time_old+6.0,t_Flight);   //垂速指令
                                        H_cx = H_cx+Vy_CX*STEP_20ms;   //频率20ms，50HZ       垂直高度  STEP_20ms
                                        ALFA_cmd =  ALFA_cmd_old0* (1.0-Transition_function(time_old,time_old+1.0,t_Flight)) +
                                            ( 11.0-(u_Ymis_n-H_cx)*0.8-(u_VY_n-Vy_CX)*1.6+1.0*(1.0-cos(u_GAMMA/57.3)) )*Transition_function(time_old,time_old+1.0,t_Flight) ;

                                      //  ALFA_cmd = limit2(ALFA_cmd,THETA_v*Rad2Deg+10,THETA_v*Rad2Deg-2);             //   弹道倾角+10   -2

                                        //保存原始
                                        VYold = VY;                                                                                                                                                       //纵向速度
                                        H_cx1 = g_Height;                                                                                                                                       //垂直高度
                                        time_old1 = t_Flight;                                                                                                                                 //
                                        ALFA_cmd_old = ALFA_cmd ;
                                        wz_cmd=0;
                                        //增加过度过程
                            }
                            cout<<"ALFA_cmd"<<ALFA_cmd<<endl;
                }
                else{
                        //30M以上巡航  标志位
                        flag_xh = 1;
                        //垂速控制指令  3S以内衰减为0
                        Vy_CX = VYold*(1.0-Transition_function(time_old1,time_old1+3.0,t_Flight));
                        //时间
                        H_cx1 = H_cx1+Vy_CX*STEP_20ms;//STEP_20ms
                        JF_NY = (u_Ymis_n-H_cx1)*STEP_20ms+JF_NY;

                        //1S内衰减
                        ALFA_cmd =ALFA_cmd_old*(1.0-Transition_function(time_old1,time_old1+1.0,t_Flight))+
                            ( 1.0-(u_Ymis_n-H_cx1)*0.8-(u_VY_n-Vy_CX)*1.6+1.0*(1.0-cos(u_GAMMA/57.3))-JF_NY*0)*Transition_function(time_old1,time_old1+1.0,t_Flight);
                        wz_cmd= 0  ;
                }
                                if (V_MIS<18.0&&flag_sd==0)
                                {
                                    Fai_real=1;     // 油门为0.65   920kv   9*5桨    ******************************************
                                }
                                else
                                {
                                    flag_sd =1;  //速度标志位
                                    JF_ma=JF_ma+(V_MIS-20.0)*STEP_20ms;//
                                    Fai_real = 1 - 0.2*(V_MIS-20.0) -2.0*JF_ma;  //速度PI
                                    Fai_real =limit2(Fai_real,1,0.2);
                                }
                                  double u_VZ = sin(z_control[4])*VN_x + cos(z_control[4])*VE_z ;
            	if (t_Flight<5.0)
                {
                    GAMMAcx = 0.0;
                }
                else if(flag_zw == 0)
                {
                    JF_NZ = (z_control[1]-0.0)*STEP_20ms+JF_NZ;
                    GAMMAcx = -z_control[1]*1.0-u_VZ*2.0-JF_NZ*0.04;  //积分
                    JF_Zmis_n = 0 ;
                    t_turn = t_Flight;
                }
                else{
                    JF_NZ = 0; 
                    Vincenty_F(Turn_long , Turn_lat, Long*Rad2Deg ,Lat*Rad2Deg,&disturn,&aturn); //反解距离目标点距离，与方位角
                    u_VZ = cos(aturn*Deg2Rad)*VN_x + sin(aturn*Deg2Rad)*VE_z ;   //北向动向速度
                    JF_Zmis_n = JF_Zmis_n + (R_Target-disturn)* STEP_20ms;     //积分
                    GAMMAcx = -1.0 * (R_Target-disturn) - 2.0*(u_VZ) - JF_Zmis_n * 0.1+30;
                    GAMMAcx = limit1(GAMMAcx,45.0);   //滚转角限制45度
    			  //初始化判断：下一航线侧偏<2或航向角差2度  -->到下一个弯   出弯条件
    			  //这里正北逆时针-180，180     要转换  (0,360)
                    if (z_control[1]<2.0||abs(PSCI_t-z_control[4])<2.0*Deg2Rad)
                    {
                        flag_zw = 0;
                    }
                }
                roll_cmd=GAMMAcx;
                pitch_cmd=ALFA_cmd;
                yaw_cmd=0;
                throttle_cmd=Fai_real;
        }
         fw_4cmd.roll_angle_sp=deg_2_rad(0);
        fw_4cmd.pitch_angle_sp =deg_2_rad(pitch_cmd);
        fw_4cmd.yaw_angle_sp =deg_2_rad(0);
         fw_4cmd.throttle_sp=throttle_cmd;
          fw_cmd_pub.publish(fw_4cmd); 
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
       start_control=1;
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
          fw_cmd_pub.publish(fw_4cmd); 
          cout<<"pitch_angle"<<thisfw_states.pitch_angle<<endl;

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
                roll_cmd=roll_cmd*Rad2Deg;
                cout<<"roll_cmd"<<roll_cmd<<endl;
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
        //提前标定航点，逆风，飞机上电
        /************第一层判断：加速度与速度判断    第二层：高度和加速度判断**************/
        /******************************连续三次判断 判断成功进入*************************/
        if((thisfw_states.body_acc[0]<0&&run_flag==0&&thisfw_states.ground_speed>10)||(thisfw_states.relative_hight>1.6&&thisfw_states.ground_speed>10))//速度(地速>10m/s)与加速度<0 三拍   
        {
             cnt++;
            if(cnt==3)
            {
                cnt==0;
                run_flag=1;
                start_control=1; 
                ALPHA_start = thisfw_states.pitch_angle;       
            }
        }
        // 高度>1.6m  且速度大于10
        if(start_control==1)
        {
            /*状态更新*/
            status_update();
             /*载入航点*/
            Func_Dictate();
             /*俯仰控制*/
            Func_ADRC_FY();
             /*滚转控制*/
            Func_ADRC_Gd();
             /* 发布四通道控制量 */
            fw_4cmd.throttle_sp =Fai_real;
            fw_4cmd.roll_angle_sp =deg_2_rad(GAMMAcx);
            fw_4cmd.pitch_angle_sp =deg_2_rad(ALFA_cmd);
            /***********************由于只能传已定义的状态值，故将此作为标志位开始启控，置为1*************************************/
            fw_4cmd.yaw_angle_sp =-1.0;
            fw_cmd_pub.publish(fw_4cmd); 
        }
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

void TASK_MAIN::test1()
{
       /************通讯测试**************/
       if(thisfw_states.pitch_angle>0)
       {
             fw_4cmd.throttle_sp=1;
             fw_4cmd.roll_angle_sp=0;
             fw_4cmd.pitch_angle_sp =0;
             fw_4cmd.yaw_angle_sp =1;
       }
       else if(thisfw_states.pitch_angle<0)
       {
             fw_4cmd.throttle_sp=-1;
             fw_4cmd.roll_angle_sp=0;
             fw_4cmd.pitch_angle_sp =0;
             fw_4cmd.yaw_angle_sp =-1;
       }
          fw_cmd_pub.publish(fw_4cmd); 
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
    //0 解锁, 10 上锁, 999 启控rk3399, 1 测试程序, 2 编队formation, 3 空位, 6 切换队形, 9 开伞
    while (ros::ok())
    {
        /**
        * 任务大循环，根据来自switch_mode的控制指令来进行响应的控制动作
       */
        current_time = get_ros_time(begin_time); /*此时刻，只作为纪录，不用于控制*/
        TASK_MAIN_INFO("Time:" << current_time);
        /*更新飞机状态*/
         update_fwsates();


        /*任务大循环*/
        if (!fw_cmd_mode.need_take_off &&
            !fw_cmd_mode.need_formation &&
            !fw_cmd_mode.need_land &&
            !fw_cmd_mode.need_idel &&
            fw_cmd_mode.need_protected&&
            !fw_cmd_mode.need_mission)
        {
            // 999: 解锁3399
            TASK_MAIN_INFO("解锁3399  起飞主程序");
            /**
             * TODO:保护子程序
             */
            control_zcontrol1();
            fw_4cmd.roll_angle_sp =0;
            fw_4cmd.pitch_angle_sp =0;
            fw_4cmd.yaw_angle_sp =0;
            fw_4cmd.throttle_sp =1;
            fw_cmd_pub.publish(fw_4cmd); 
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_PROTECT;
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 !fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("开伞子程序");
            fw_4cmd.roll_angle_sp =0;
            fw_4cmd.pitch_angle_sp =0;
            fw_4cmd.yaw_angle_sp =0;
            fw_4cmd.throttle_sp =5;


            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_IDEL;
         }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 !fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("上伞子程序");
            /**
                 * TODO:降落子程序
                */
             //control_land();
                float  zs=fw_cmd_mode.swarm_shape;
               
                fw_4cmd.roll_angle_sp =1;//deg_2_rad(0);
                fw_4cmd.pitch_angle_sp =1;
                fw_4cmd.yaw_angle_sp =zs;
                 fw_4cmd.throttle_sp =0;
                fw_cmd_pub.publish(fw_4cmd); /* 发布四通道控制量 */

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
                 * TODO:从机的编队程序
                */
            /*计算领机的侧偏距离*/
            //control_zcontrol1();
            
            /*计算编队的航机系误差和领机速度*/
            control_formation();
            float V_leader;
            V_leader =sqrt(leader_states.ned_vel_x*leader_states.ned_vel_x+leader_states.ned_vel_y*leader_states.ned_vel_y+leader_states.ned_vel_z*leader_states.ned_vel_z);
            fw_4cmd.roll_angle_sp =fw_error.PXk;     //航迹系误差
            fw_4cmd.pitch_angle_sp =V_leader;         //领机速度
            fw_4cmd.yaw_angle_sp =z_control[1];     //领机侧偏距离
            fw_4cmd.throttle_sp =10;
            fw_cmd_pub.publish(fw_4cmd);
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_FORMATION;  //用这个判断通讯？
        }
        else if (fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 !fw_cmd_mode.need_mission)
        {

            TASK_MAIN_INFO("测试子程序");
            /**
                 * TODO:进入起飞大循环
                */
           fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_TAKEOFF;

            /*仿真可以走这个*/
             control_takeoff2();
            cout<<"起飞起飞"<<endl;

            // /* 发布四通道控制量 */
            // float  t_test=fw_cmd_mode.t_test; 
            // fw_4cmd.roll_angle_sp =0;
            // fw_4cmd.pitch_angle_sp =1;
            // fw_4cmd.yaw_angle_sp =t_test;
            // fw_4cmd.throttle_sp =15;
            fw_cmd_pub.publish(fw_4cmd);
        }
        else if (!fw_cmd_mode.need_take_off &&
                 !fw_cmd_mode.need_formation &&
                 !fw_cmd_mode.need_land &&
                 !fw_cmd_mode.need_idel &&
                 !fw_cmd_mode.need_protected&&
                 fw_cmd_mode.need_mission)
        {
            TASK_MAIN_INFO("空位程序");
            /**
                 * TODO:任务子程序
                */
            fw_4cmd.roll_angle_sp =0;
            fw_4cmd.pitch_angle_sp =0;
            fw_4cmd.yaw_angle_sp =0;
            fw_4cmd.throttle_sp =20;
            fw_cmd_pub.publish(fw_4cmd);
            fw_current_mode.mode = fixed_wing_formation_control::Fw_current_mode::FW_IN_MISSION;

        }
        else
        {
            //正常上电运行后进入空位
            fw_4cmd.roll_angle_sp =0;
            fw_4cmd.pitch_angle_sp =0;
            fw_4cmd.yaw_angle_sp =0;
            fw_4cmd.throttle_sp =0;
            fw_cmd_pub.publish(fw_4cmd);
            TASK_MAIN_INFO("错误，飞机当前状态有误");
            TASK_MAIN_INFO("test0");
        }

        /*通讯稳定性测试*/    

        fw_4cmd.date_pitch =  fwmonitor_flag.fw_is_connected; //PIX
        fw_4cmd.date_roll = 0;    
        fw_4cmd.date_yaw = leaderstates_update;
        if(leaderstates_update >0.1)
        {
            fwstates.wind_estimate_x =  fwmonitor_flag.fw_is_connected; //PIX
            fwstates.wind_estimate_y = 1;
            mc_ctrl.ax = fwmonitor_flag.fw_is_connected;
            mc_ctrl.ay = 1;
            fw_4cmd.date_pitch =  fwmonitor_flag.fw_is_connected; //PIX
            fw_4cmd.date_roll = 1;
        }
        else if(leaderstates_update<0.1){
            fwstates.wind_estimate_x =  fwmonitor_flag.fw_is_connected; //PIX
            fwstates.wind_estimate_y = 0;          
            mc_ctrl.ax = fwmonitor_flag.fw_is_connected;
            mc_ctrl.ay = 0; 
            fw_4cmd.date_pitch =  fwmonitor_flag.fw_is_connected; //PIX
            fw_4cmd.date_roll = 0;             
        }


        /**
         * 发布飞机当前状态
        */
        fw_current_mode_pub.publish(fw_current_mode);
        fw_state_pub();
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