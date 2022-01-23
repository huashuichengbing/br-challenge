#ifndef DOOR_DETECTOR_ROS_CONNECTOR_H
#define DOOR_DETECTOR_ROS_CONNECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include "door_detector/cloud_processor.h"

namespace door_detector {

using std::string;

class ROSConnector {
     private:
        ros::NodeHandle nh_;
        laser_geometry::LaserProjection projector_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        CloudProcessor *cloud_processor_;
        pcl::PointCloud<Point>::Ptr pcl_cloud_;

        string scan_topic_;
        string path_topic_;
        float min_door_width_;
        float cluster_min_size_, cluster_max_size_, cluster_tolerance_;
        float line_min_points_, line_exact_min_points_;

     public:
        ROSConnector();

        ~ROSConnector();

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        void spin(int rate);

        void parseParams();

        template <class T>
        T find_param(const std::string &node_full_name, const std::string &param_name);
};

}  // namespace door_detector

#endif // DOOR_DETECTOR_ROS_CONNECTOR_H