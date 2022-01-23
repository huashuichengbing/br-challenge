/**
 * @file main.cc
 * @author Teshan Liyanage (teshanuka@gmail.com)
 * @brief Run door detection ROS node
 * @version 0.1
 * @date 2022-01-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <ros/ros.h>

#include "door_detector/ros_connector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    door_detector::ROSConnector conn;
    conn.spin(10);

    return 0;
}
