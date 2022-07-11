/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-12 23:23:34
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  
 *  本程序是虚拟领机，领机从西向东按照一定的速度飞行
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-02-17 15:23:55
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */

#include "vir_dir_leader.hpp"

void VIR_DIR_LEADER::ros_sub_pub()
{
    vir_leader_pub = nh.advertise<fixed_wing_formation_control::Leaderstates>("/uav0/fixed_wing_formation_control/leader_states", 10);
    uav1_pub=nh.advertise<fixed_wing_formation_control::FWcmd>("/uav1/fixed_wing_formation_control/fw_cmd",10);
    uav2_pub=nh.advertise<fixed_wing_formation_control::FWcmd>("/uav2/fixed_wing_formation_control/fw_cmd",10);
    uav3_pub=nh.advertise<fixed_wing_formation_control::FWcmd>("/uav3/fixed_wing_formation_control/fw_cmd",10);
    uav4_pub=nh.advertise<fixed_wing_formation_control::FWcmd>("/uav4/fixed_wing_formation_control/fw_cmd",10);
}

void VIR_DIR_LEADER::test1()
{
            t+=0.01;
        if(t<1)
        {
                leaderstates.pitch_angle=1;
        }
        else if(1<=t&&t<2)
        {
                leaderstates.pitch_angle=-1;
        }else 
        {
              t=0;
        }
        vir_leader_pub.publish(leaderstates);

}

void VIR_DIR_LEADER::test2()
{
         t+=0.01;
        if(t<0.5)
        {
                leaderstates.pitch_angle=1;
                vir_leader_pub.publish(leaderstates);
                fw1_cmd.pitch_angle_sp=1;
                uav1_pub.publish(fw1_cmd);
                fw2_cmd.pitch_angle_sp=1;
                uav2_pub.publish(fw2_cmd);
                fw3_cmd.pitch_angle_sp=1;
                uav3_pub.publish(fw3_cmd);
                fw4_cmd.pitch_angle_sp=1;
                uav4_pub.publish(fw4_cmd);
        }
        else if(0.5<=t&&t<1)
        {
                leaderstates.pitch_angle=-1;
                vir_leader_pub.publish(leaderstates);
                fw1_cmd.pitch_angle_sp=-1;
                uav1_pub.publish(fw1_cmd);
                fw2_cmd.pitch_angle_sp=-1;
                 uav2_pub.publish(fw2_cmd);
                fw3_cmd.pitch_angle_sp=-1;
                uav3_pub.publish(fw3_cmd);
                fw4_cmd.pitch_angle_sp=-1;
                uav4_pub.publish(fw4_cmd);
        }else 
        {
              t=0;
        }
            
}

void VIR_DIR_LEADER::run(int argc, char **argv)
{
    ros::Rate rate(10.0);
    ros_sub_pub();
    while (ros::ok())
    {

        //test1();
        test2();
        ros::spinOnce(); //挂起一段时间，保证周期的速度
        rate.sleep();

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vir_dir_leader");

    VIR_DIR_LEADER _vir_leader;

    if (true)
    {
        _vir_leader.run(argc, argv);
    }

    return 0;
}
