#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <sstream>
#include <string>

#include "door_detector/ros_connector.h"

namespace door_detector {

using std::add_rvalue_reference;

ROSConnector::ROSConnector() :
    pcl_cloud_(new pcl::PointCloud<Point>)
{
    parseParams();

    cloud_processor_ = new CloudProcessor(min_door_width_, cluster_min_size_, cluster_max_size_, cluster_tolerance_, 
                                          line_min_points_, line_exact_min_points_);
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan> (scan_topic_, 1, &ROSConnector::scanCallback, this);
    point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> (path_topic_, 100, false);
}

void ROSConnector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    // point_cloud_publisher_.publish(cloud);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud_);
}

void ROSConnector::spin(int rate) {
    ros::Rate loop_rate(rate);
    while (nh_.ok()) {
        ros::spinOnce();
        cloud_processor_->spinOnce();
        cloud_processor_->detect(pcl_cloud_);

        loop_rate.sleep();
    }
}

template<class T>
T ROSConnector::find_param(const string &node_full_name, const string &param_name) {
    T param_value;
    if (!nh_.getParam(node_full_name + "/" + param_name, param_value))
        throw std::runtime_error("parameter `" + param_name + "` is not found");
    DEBUG_PRINTLN("\t" << param_name << ": " << param_value);
    return param_value;
}

void ROSConnector::parseParams() {
    const string &node_name = ros::this_node::getName();
    const string &ns = ros::this_node::getNamespace();
    const string full_name = (ns == "/") ? node_name : (ns + node_name);

    DEBUG_PRINTLN("Loading parameters...");

    scan_topic_ = find_param<string>(full_name, "ip_scan_topic");
    path_topic_ = find_param<string>(full_name, "op_path_topic");
    min_door_width_ = find_param<float>(full_name, "min_door_width");
    cluster_min_size_ = find_param<float>(full_name, "cluster_min_size");
    cluster_max_size_ = find_param<float>(full_name, "cluster_max_size");
    cluster_tolerance_ = find_param<float>(full_name, "cluster_tolerance");
    line_min_points_ = find_param<float>(full_name, "line_min_points");
    line_exact_min_points_ = find_param<float>(full_name, "line_exact_min_points");
}

ROSConnector::~ROSConnector() {
    free(cloud_processor_);
}

}  // namespace door_detector
