#include <ros/ros.h>
#include "dl_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "dl_base_node");
    DlBase dl;
    ros::spin();
    return 0;
}
