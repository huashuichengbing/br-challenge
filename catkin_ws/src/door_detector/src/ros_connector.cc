/**
 * @file ros_connector.cc
 * @author Teshan Liyanage (teshanuka@gmail.com)
 * @brief Connecting door detector with ROS
 * @version 0.1
 * @date 2022-01-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

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
    path_publisher_ = nh_.advertise<door_detector::Paths> (path_topic_, 100, false);
}


void ROSConnector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud_);
    last_header_ = scan->header;
}


door_detector::Paths ROSConnector::doors2path(const std::vector<LineSegment>& doors) {
    door_detector::Paths paths;
    static int seq = 0;
    auto this_header = last_header_;
    this_header.seq = seq;
    this_header.stamp = ros::Time::now();
    
    for (const auto& door: doors){
        nav_msgs::Path path;
        path.header = this_header;

        auto points = getPerpendicularPoints(door, entry_path_offset_);
        std::sort(points.begin(), points.end(), [&points](const Point& p1, const Point& p2){
            return lineLengthXY(LineSegment(p1, Point(0, 0, 0))) < lineLengthXY(LineSegment(p2, Point(0, 0, 0)));
        });

        geometry_msgs::Quaternion orientation;
        orientation.x = points[1].x-points[0].x;
        orientation.y = points[1].y-points[0].y;
        orientation.z = points[1].z;
        orientation.w = 1;
        int sub_seq = 0;
        for (const auto& pt: points){
            geometry_msgs::PoseStamped pose;
            pose.header = this_header;
            pose.header.seq = sub_seq++;

            pose.pose.orientation = orientation;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.position.z = pt.z;

            path.poses.push_back(pose);
        }
        paths.paths.push_back(path);
    }
    seq++;
    return paths;
}


void ROSConnector::spin(int rate) {
    ros::Rate loop_rate(rate);
    while (nh_.ok()) {
        std::vector<LineSegment> doors = cloud_processor_->detect(pcl_cloud_);
        door_detector::Paths paths = doors2path(doors);
        path_publisher_.publish(paths);

        ros::spinOnce();
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
    entry_path_offset_ = find_param<float>(full_name, "entry_path_offset");
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
