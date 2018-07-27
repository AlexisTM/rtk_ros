/****************************************************************************
 *
 *   Copyright (C) 2018. All rights reserved.
 *   Author: Alexis Paques <alexis.paques@gmail.com>
 * 
 ****************************************************************************/

#include <ros/ros.h>
#include <rtk_ros/rtk_node.hpp>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rtk_base_station_node_cpp");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string port = "/dev/ttyUSB0";
    int32_t baud = 9600;
    float surveyAccuracy = 1.0;
    float surveyDuration = 90.0;

    pnh.param<std::string>("port", port, port);
    pnh.param<int32_t>("baud", baud, baud);
    pnh.param<float>("survey/accuracy", surveyAccuracy, surveyAccuracy);
    pnh.param<float>("survey/duration", surveyDuration, surveyDuration);

    RTKNode rtknode(baud, port, surveyAccuracy, surveyDuration);

    rtknode.connect();
    rtknode.connect_gps();
    rtknode.configure();

    ros::spin();
    return 0;
}
