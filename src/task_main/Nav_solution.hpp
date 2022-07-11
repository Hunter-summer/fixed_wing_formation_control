#include "../fixed_wing_lib/mathlib.hpp"
class Nav_solution
{
	
public:
	double  out[5];
	void RealativeLocation(double lat_m, double long_m, double lat_s, double long_s, double lat_e, double long_e);
};
//m: 当前的经纬度
//s:   start  起始点的经纬度
//e：end    终点的经纬度

void Nav_solution::RealativeLocation(double lat_m, double long_m, double lat_s, double long_s, double lat_e, double long_e)
{
	//rg:距离
	double xg, zg, sg, rg, angle_a;
	double I_s[3], I_e[3], I_v[3], I_n[3], I_se[3], I_ve[3], I_ht0[3], I_ht0_n[3], I_n0[3];
	double lat_wd, long_wd;
    //坐标系转换
	I_s[0] = sin(lat_s);
	I_s[1] = cos(lat_s) * cos(long_s);
	I_s[2] = cos(lat_s) * sin(long_s);

	I_e[0] = sin(lat_e);
	I_e[1] = cos(lat_e) * cos(long_e);
	I_e[2] = cos(lat_e) * sin(long_e);

	I_v[0] = sin(lat_m);
	I_v[1] = cos(lat_m) * cos(long_m);
	I_v[2] = cos(lat_m) * sin(long_m);

	I_n[0] = cos(lat_m);
	I_n[1] = -sin(lat_m) * cos(long_m);
	I_n[2] = -sin(lat_m) * sin(long_m);

	I_se[0] = I_e[0] - I_s[0];
	I_se[1] = I_e[1] - I_s[1];
	I_se[2] = I_e[2] - I_s[2];



	I_ve[0] = I_e[0] - I_v[0];
	I_ve[1] = I_e[1] - I_v[1];
	I_ve[2] = I_e[2] - I_v[2];

	I_ht0[0] = I_e[1] * I_s[2] - I_e[2] * I_s[1];
	I_ht0[1] = I_e[2] * I_s[0] - I_e[0] * I_s[2];
	I_ht0[2]=  I_e[0] * I_s[1] - I_e[1] * I_s[0];

	I_n0[0] = 1;
	I_n0[1] = 0;
	I_n0[2] = 0;

	double norm_ht0 = sqrt(I_ht0[0] * I_ht0[0] + I_ht0[1] * I_ht0[1] + I_ht0[2]*I_ht0[2]);
	I_ht0[0] = I_ht0[0] / norm_ht0;
	I_ht0[1] = I_ht0[1] / norm_ht0;
	I_ht0[2] = I_ht0[2] / norm_ht0;

	double lat0 = acos(I_ht0[0] * I_n0[0] + I_ht0[1] * I_n0[1] + I_ht0[2] * I_n0[2]);
	if (lat0 > 3.1415926535897932384626433832795 / 2.0)
		lat0 = 3.1415926535897932384626433832795 - lat0;
	if (lat_s >= 0)
		lat_wd = fabs(lat0);
	else
		lat_wd = -fabs(lat0);

	double long0 = 500;
	double long1, long2;

	if (fabs(I_ht0[1]) < 1e-10)
		long1 = 3.1415926535897932384626433832795 / 2.0;
	else if (I_ht0[2] / I_ht0[1] >= 0)
		long1 = atan(I_ht0[2] / I_ht0[1]);
	else
		long1 = 3.1415926535897932384626433832795 + atan(I_ht0[2] / I_ht0[1]);

	long2 = long1 - 3.1415926535897932384626433832795;

	if (fabs(long_s - long_e) <= 3.1415926535897932384626433832795)
	{
		if (fabs(lat_wd) > 1e-10 && (long1 - long_s) * (long1 - long_e) < 0)
			long_wd = long1;
		else if (fabs(lat_wd) > 1e-10 && (long2 - long_s) * (long2 - long_e) < 0)
			long_wd = long2;
		else
			long_wd = long0;
	}
	else
	{
		if (fabs(lat_wd) > 1e-10 && (long1 - long_s) * (long1 - long_e) < 0)
			long_wd = long1;
		else if (fabs(lat_wd) > 1e-10 && (long2 - long_s) * (long2 - long_e) < 0)
			long_wd = long2;
		else
			long_wd = long0;
	}

	sg = 6378137.0 * acos(I_e[0] * I_v[0] + I_e[1] * I_v[1] + I_e[2] * I_v[2]);       //两点大圆弧距离
	zg = 6378137.0 * (I_ht0[0] * I_v[0] + I_ht0[1] * I_v[1] + I_ht0[2] * I_v[2]);   //到期望航迹侧向距离

	double norm_xg0 = fabs(sg * sg - zg * zg);
	if (norm_xg0 >= 0)
		xg = sqrt(norm_xg0) * sign(I_se[0] * I_ve[0] + I_se[1] * I_ve[1] + I_se[2] * I_ve[2]);
	else
		xg = 0;

	double phi_y = acos(I_ht0[0] * I_n[0] + I_ht0[1] * I_n[1] + I_ht0[2] * I_n[2]);

	if (long_wd > 100)
	{
		if (lat_e >= lat_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 - phi_y;
		else if(long_e>=long_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 + phi_y-2.0* 3.1415926535897932384626433832795;
		else
			angle_a= 0.5 * 3.1415926535897932384626433832795 + phi_y;
	}
	else if (fabs(long_m - long_s) < fabs(long_m - long_wd) || (long_m - long_s) * (long_m - long_wd) <= 0)
	{
		if (lat_e >= lat_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 - phi_y;
		else if (long_e >= long_s)
			angle_a = 0.5 * 3.1415926535897932384626433832795 + phi_y - 2.0 * 3.1415926535897932384626433832795;
		else
			angle_a = 0.5 * 3.1415926535897932384626433832795 + phi_y;
	}

	double range = 6378137.0 * acos(I_e[0] * I_s[0] + I_e[1] * I_s[1] + I_e[2] * I_s[2]);
	rg = range - xg;
	out[0] = xg;   // 当前位置与航线末点的距离？
	out[1] = zg;  //当前位置与当前航线的侧偏距离
	out[2] = sg;  //当前位置与当前航线终点的距离
	out[3] = range;  //当前航线的距离
	out[4] = angle_a; //航线的方位角
}

