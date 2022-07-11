//北天东坐标系       x: 机头         y: 纵向      z:横向 与俯仰角有关
//200HZ
//机体：y偏航  z俯仰  
void CMathModel_Ctrl::Func_ADRC_Gd(void)
{
	

		kpgd = 0.3; 
		kdgd = 0.4;
		kigd = 0.02;
		//不要了可以double kq = 15/V_MIS*15/V_MIS;

        //计算导航点之间的侧向           所有量都为rad？？待定  度  核算一下
		RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target,z_control);   //算侧偏位移？
		
		double u_VZ = sin(z_control[4])*VN_x + cos(z_control[4])*VE_z ;                                        //z_control?  里面是什么？   z横向  VEZ动向速度
		//方位角
        if (t_Flight<3) 
		{
			GAMMAcx = 0;
		}
		else if(flag_zw == 0)    //直线控制  PID
		{
			JF_NZ = (z_control[1]-0)*STEP_5ms+JF_NZ;            //?  侧偏位置
			GAMMAcx = -z_control[1]*1-u_VZ*2-JF_NZ*0.04;   //  
			JF_Zmis_n = 0 ;                                                                         //?
			t_turn = t_Flight;
		}
		else       //
		{
			JF_NZ = 0;  //直线的积分项
            //文森反解
			Vincenty_F(Turn_long , Turn_lat, Long*Rad2Deg ,Lat*Rad2Deg,&disturn,&aturn);    //?   aturn？
			
            // // ？ 没有
			// if (t_Flight>74)
			// {
			// 	t_Flight = t_Flight;
			// }

            //
			u_VZ = cos(aturn*Deg2Rad)*VN_x + sin(aturn*Deg2Rad)*VE_z ;   //北向动向速度
		    
			JF_Zmis_n = JF_Zmis_n + (R_Target-disturn)* STEP_5ms;     //积分向
			GAMMAcx = -1 * (R_Target-disturn) - 2*(u_VZ) - JF_Zmis_n * 0.1+30;
			GAMMAcx = limit1(GAMMAcx,45);   //滚转角限制45度

			//GAMMAcx =  GAMMAcx* Transition(t_turn,t_turn+2,t_Flight);
            //初始化判断：下一航线侧偏<2或航向角差2度  -->到下一个弯   出完条件？
            //航向角定义  看是否一样
            //这里正北逆时针-180，180     要转换  (0,360)        
            //要把定义对的上
 			if (z_control[1]<2||abs(PSCI_t-z_control[4])<2*Deg2Rad)
			{
				flag_zw = 0;
			}
		}

		

        //前面是生成滚转角指令   后面是姿态滚转角控制
        //滚转角控制参数  自抗扰

		kq = 1;
		kpgd = kpgd*kq * 0.8*3.5*1.2/4*0.15;
		kdgd = kdgd*kq * 0.8*1.0*1.5/4*0.15;
		BX = -300;    //?
		omega_x = 5;


		dz1_gd = -2*omega_x*z1_gd +  z2_gd + 2*omega_x*WX_f+BX*U_gd_r;   //WX_F滚转角速率
		dz2_gd = -omega_x*omega_x * z1_gd + omega_x*omega_x*WX_f;
		z1_gd = dz1_gd*STEP_5ms + z1_gd; //z1_gd  初始化为0
		z2_gd = dz2_gd*STEP_5ms + z2_gd; //z2_gd

		U_gd_1 = kpgd * (u_GAMMA-GAMMAcx);
		U_gd_2 = kdgd * u_WX_b;
		JF_gd  = JF_gd+(u_GAMMA-GAMMAcx)*STEP_5ms;
		//U_gd_3 =  kigd * JF_gd;   这个积分不用了
		U_gd_3 = -z2_gd/BX;
		U_gd_r = 	limit1(U_gd_1 ,10)+ limit1(U_gd_2,15)+limit1(U_gd_3,10);
		//把U_gd_r记录下俩
        //U_gd_r = 0;
