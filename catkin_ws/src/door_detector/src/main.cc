#include <ros/ros.h>

#include "door_detector/ros_connector.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    door_detector::ROSConnector conn;
    conn.spin(10);

    return 0;
}
