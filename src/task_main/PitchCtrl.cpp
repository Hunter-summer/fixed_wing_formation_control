//北天东坐标系       x: 机头         y: 纵向      z:横向 与俯仰角有关
//200HZ

void CMathModel_Ctrl::Func_ADRC_FY(double t_Flight,double g_Height,double VY,double)
{		
			//启控

			if ((g_Height<30)&&flag_xh==0)     
			{
				if (t_Flight<1)    //1s内仅有姿态
				{
					//过渡函数
					ALFA_cmd = 30*(1-Transition(0,1,t_Flight))+15*(Transition(0,1,t_Flight));     //俯仰角30--->15   11        
					
					wz_cmd   = -6*cos((t_Flight-0.5)*PI)*PI;                                                                             //俯仰角速度

					//记录上一步长的值             用于渐变
					time_old = t_Flight;     //
					VYold0 = VY;                     //北东地要变符号
					H_cx = g_Height;          //纵向高度
					Hold = g_Height;          
					ALFA_cmd_old0 = ALFA_cmd;
					
				}
				else if(g_Height<30)     //属于哪个过程
				{	
					
					//3s内过渡	  合理过渡   从第一阶段结束的垂速过渡到3M/S
					Vy_CX = VYold0*(1-Transition(time_old,time_old+3,t_Flight))+3*Transition(time_old,time_old+3,t_Flight);   //垂速指令

					
					H_cx = H_cx+Vy_CX*STEP_5ms;   //频率5ms，20HZ       垂直高度


					ALFA_cmd =  ALFA_cmd_old0* (1-Transition(time_old,time_old+1,t_Flight)) +
						( -(u_Ymis_n-H_cx)*1-(u_VY_n-Vy_CX)*2+1*(1-cos(u_GAMMA/57.3)) )*Transition(time_old,time_old+1,t_Flight)  ;    //u_YY_n 传感器测得的速度   ?   u_Ymis_n    高度？
					
					 
					ALFA_cmd = limit2(ALFA_cmd,THETA_v*Rad2Deg+10,THETA_v*Rad2Deg-2);             //   弹道倾角+10   -2
					
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
				Vy_CX = VYold*(1-Transition(time_old1,time_old1+3,t_Flight))+0*Transition(time_old1,time_old1+10,t_Flight);  
				//时间
				H_cx1 = H_cx1+Vy_CX*STEP_5ms;
				//JF
				JF_NY = (u_Ymis_n-H_cx1)*STEP_5ms+JF_NY;

				//1S内衰减	
				ALFA_cmd =ALFA_cmd_old*(1-Transition(time_old1,time_old1+1,t_Flight))+
					( 1-(u_Ymis_n-H_cx1)*1-(u_VY_n-Vy_CX)*2+1*(1-cos(u_GAMMA/57.3))-JF_NY*0)*Transition(time_old1,time_old1+1,t_Flight);
				

				wz_cmd= 0  ;

			}
			


			//空速能用最好  判断空速是否失效

			// //V_MIS？什么速度？地速还是空速？

			// double kq = 15/V_MIS ;
			// //kq = 1;
			// kalfa=-2.0*kq*0.8*2*1.2/50;//*0.05
			// kdfy=-0.8*kq*0.7*1.2;//*0.7
			// kify=-0.5*kq*0;
			// kq = 1; 
			// kalfa=-1*kq*0.6;
			// kdfy = -0.5*kq*0.8;
			

			
			//速度控制
			if (V_MIS<30&&flag_sd==0)
			{
				Fai_real=1.0;   // 速度为1
			}
			else
			{
				flag_sd =1;  //速度标志位

						JF_ma=JF_ma+(V_MIS-30)*0.1;     //?JF_ma
						Fai_real =  1 - 1*(V_MIS-30) - 100*JF_ma;  //速度PI
 						Fai_real =limit2(Fai_real,1.0,0.0);


			}

			//姿态控制
			kq = 15/V_MIS ;   //动压调节
			kalfa=-0.2*kq*kq;
			kdfy = -0.8*kq*kq;
			kalfa= (-0.2 + 0.1* sqrt((V_MIS-15)/15))*0.8;//
			kdfy = (-0.75 + 0.6 * sqrt((V_MIS-15)/15))*0.8;

			omega_z = 5;        //扩张状态观测器的系数
			BZ = 45;//-150;    

	
			//观测器的内容   积分初始化需要变为0，要注意
			dz1_fy = -2*omega_z*z1_fy +  z2_fy + 2*omega_z*(sin(u_GAMMA*Deg2Rad) * u_WY_b + cos(u_GAMMA*Deg2Rad) * u_WZ_b)+BZ*U_fy_r;
			dz2_fy = -omega_z*omega_z * z1_fy + omega_z*omega_z*(sin(u_GAMMA*Deg2Rad) * u_WY_b + cos(u_GAMMA*Deg2Rad) * u_WZ_b);
			//积分初始化需要变为0，要注意   
			z1_fy = dz1_fy*STEP_5ms + z1_fy;  
			z2_fy = dz2_fy*STEP_5ms + z2_fy;
	
	
			
			if (t_Flight<200)   
			{
			
			U_fy_1 = kalfa * (u_THETA-ALFA_cmd);

			//北天东坐标系下的角度
			U_fy_2 = kdfy *  ((sin(u_GAMMA*Deg2Rad) * u_WY_b + cos(u_GAMMA*Deg2Rad) * u_WZ_b)-wz_cmd)	;			
			

			U_fy_3 =  -z2_fy/BZ  ;
	
			
			}
			// else
			// {
			// 	U_fy_1 = 3.5* kalfa * (u_ALFA-ALFA_cmd);
			// 	U_fy_2 = 2*kdfy *  ((sin(u_GAMMA*Deg2Rad) * u_WY_b + cos(u_GAMMA*Deg2Rad) * u_WZ_b)-wz_cmd)	;

			// 	U_fy_3 =  -z2_fy/BZ   ;
			// }
	
			//限幅   看对应的舵偏  归一化      后缘向下为正    取决于指令1的时候偏多少   舵面的极性
 			U_fy_r =  limit1(U_fy_1,15) + limit1(U_fy_2,15) + limit1(U_fy_3,15) ;

			//Flag是每个切换点？   到这个点可以开伞
			if (flag_point>4)//
			{
				_FLAG_END_TRAJ_RUN = True;
			}
