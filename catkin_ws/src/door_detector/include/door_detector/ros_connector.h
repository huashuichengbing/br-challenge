/**
 * @file ros_connector.h
 * @author Teshan Liyanage (teshanuka@gmail.com)
 * @briefConnecting door detector with ROS
 * @version 0.1
 * @date 2022-01-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DOOR_DETECTOR_ROS_CONNECTOR_H
#define DOOR_DETECTOR_ROS_CONNECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include "door_detector/Paths.h"

#include "door_detector/cloud_processor.h"

namespace door_detector {

using std::string;


/**
 * @brief A support class to connect the `CloudProcessor`'s door detections with ROS
 */
class ROSConnector {
   private:
      ros::NodeHandle nh_;
      laser_geometry::LaserProjection projector_;

      ros::Publisher path_publisher_;
      ros::Subscriber scan_sub_;
      CloudProcessor *cloud_processor_;
      pcl::PointCloud<Point>::Ptr pcl_cloud_;
      std_msgs::Header last_header_;

      string scan_topic_;
      string path_topic_;
      float min_door_width_, entry_path_offset_;
      float cluster_min_size_, cluster_max_size_, cluster_tolerance_;
      float line_min_points_, line_exact_min_points_;


      /**
       * @brief Convert detected doors to path messages
       * 
       * @param doors                  Detected doors
       * @return door_detector::Paths 
       */
      door_detector::Paths doors2path(const std::vector<LineSegment>& doors);


      /**
       * @brief Read parameters from ROS params
       */
      void parseParams();

      /**
       * @brief Load various types of parameters without a hassle. Exit with an errpr if a required parameter is not found
       * 
       * @tparam T               Parameter data type
       * @param node_full_name   Node name
       * @param param_name       Parameter name
       * @return T               Parameter value
       */
      template <class T>
      T find_param(const std::string &node_full_name, const std::string &param_name);

   public:
      ROSConnector();

      ~ROSConnector();

      /**
       * @brief Laser scan callback function
       * 
       * @param scan Scan ROS topic
       */
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);


      /**
       * @brief ROS spin adapted for flexibility
       * 
       * @param rate Spin rate
       */
      void spin(int rate);
};

}  // namespace door_detector

#endif // DOOR_DETECTOR_ROS_CONNECTOR_H