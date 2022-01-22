#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <sstream>
#include <string>

#include "door_detector/cloud_processor.h"

using std::string;
using namespace cloud_processor;

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void spin(int rate);
     private:
        ros::NodeHandle nh_;
        laser_geometry::LaserProjection projector_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
        CloudProcessor cloud_processor_;
        pcl::PointCloud<Point>::Ptr pcl_cloud_;
};

My_Filter::My_Filter() :
    pcl_cloud_(new pcl::PointCloud<Point>)
{
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan> ("/scan", 100, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.projectLaser(*scan, cloud);
    point_cloud_publisher_.publish(cloud);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud_);
}

void My_Filter::spin(int rate) {
    ros::Rate loop_rate(rate);
    while (nh_.ok()) {
        cloud_processor_.run(pcl_cloud_);
        ros::spinOnce();
        cloud_processor_.spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");

    My_Filter filter;

    filter.spin(10);

    return 0;
}
