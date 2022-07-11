#include"../../task_main/task_main.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "follower3_main");
    TASK_MAIN follower3_main;
    if (true)
    {
        follower3_main.set_planeID(3);
        follower3_main.run();
    }
    return 0;
}
