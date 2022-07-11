void CMathModel_Ctrl::Func_Dictate(void)
{
    //侧偏计算
	//RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target,z_control);

	//这五个点分别是什么？ 懂了，
	Long_Target1    = 123.368644   * Deg2Rad;;
	Lat_Target1     = 42.384428* Deg2Rad;
	angle_Target1   = 90;
	R_Target        = 180;          //转弯半径


	Long_Target2  = 123.364229        * Deg2Rad;
	Lat_Target2   = 42.386320* Deg2Rad;
	angle_Target2 = 90 ;      //转弯角度

	Long_Target3  = 123.370308        * Deg2Rad;
	Lat_Target3   =  42.394114 * Deg2Rad;
	angle_Target3 =  90;

	Long_Target4  = 123.374724        * Deg2Rad;
	Lat_Target4   =  42.392221* Deg2Rad;
	angle_Target4 =  90;

	Long_Target5  = 123.374722    * Deg2Rad;
	Lat_Target5   =   42.392222* Deg2Rad;
	angle_Target5 = 0;

	double Long_Target_arr[5]= {Long_Target1,Long_Target2,Long_Target3,Long_Target4,Long_Target1     };
	double Lat_Target_arr [5] = {Lat_Target1,Lat_Target2,Lat_Target3,Lat_Target4,Lat_Target1};
	double angle_Target_arr[5] = {angle_Target1,angle_Target2,angle_Target3,angle_Target4,angle_Target5};

	if (z_control[0]<R_Target*tan(angle_Target/2*Deg2Rad)+1&&flag_point<5)   //入完判断  跟滚转通道写一块，写到前面 提前一米转
	{
        //
		if (angle_Target>0)
		{
            //计算转弯圆心的大地方位角
			double turnangle = -z_control[4]*Rad2Deg+90;
            //反结算圆心的经纬度
			Vincenty_Z(R_Target,turnangle,Long*Rad2Deg,Lat*Rad2Deg,&Turn_long,&Turn_lat);
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

		RelativeLocation(Lat,Long, Lat_ini, Long_ini,Lat_Target,Long_Target,zw_canshu);
		//PSCI_c =90*Deg2Rad;  //可能没有用  调试用
	}

}
