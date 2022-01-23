
/**
 * @file cloud_processor.h
 * @author Teshan Liyanage (teshanuka@gmail.com)
 * @brief Cloud processing library for door detection from a pointcloud
 * @version 0.1
 * @date 2022-01-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef DOOR_DETECTOR_CLOUD_PROCESSOR_H
#define DOOR_DETECTOR_CLOUD_PROCESSOR_H

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#define VISUALIZE

#ifdef VISUALIZE
#include <pcl/visualization/cloud_viewer.h>
#endif

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)      std::cout << x
#define DEBUG_PRINTLN(x)    DEBUG_PRINT(x) << std::endl
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

namespace door_detector {

typedef pcl::PointXYZ Point;


/**
 * @brief Stores a line segment defined by two points on the line 
 */
typedef struct LineSegment
{
    LineSegment(){};
    LineSegment(const Point& pt1, const Point& pt2) : start_pt(pt1), end_pt(pt2) {};
    Point start_pt, end_pt;
} LineSegment;


/**
 * @brief Stores information of multiple lines from a pointcloud 
 */
typedef struct FittedLineInfo
{
    FittedLineInfo() : num_lines(0) {};
    std::vector<pcl::PointCloud<Point>::Ptr> lines;
    std::vector<LineSegment> lines_min_max_points;
    std::vector<pcl::ModelCoefficients::Ptr> lines_coefficients;
    int num_lines;
} FittedLineInfo;


/**
 * @brief Stores information of a set of pointcloud clusters 
 */
typedef struct ClusterInfo
{
    ClusterInfo() : num_clusters(0) {};
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    int num_clusters;
} ClusterInfo;


/**
 * @brief Calculates the length of a line segment (On xy plane)
 * 
 * @param line 
 * @return float 
 */
inline float lineLengthXY(const LineSegment& line) {
    return sqrt(pow(line.end_pt.x - line.start_pt.x, 2) + pow(line.end_pt.y - line.start_pt.y, 2));
}


/**
 * @brief Find three points from a line segment such that the points form one line perpendicular to the given line segment
 *  and exacly divides the line segment in two. i.e. Second point returned is the center point of the line segment.
 * 
 * @param line      Input line segment
 * @param perp_l    Two points' length from the cenerpoint of the segment.
 * @return std::vector<Point> 
 */
inline std::vector<Point> getPerpendicularPoints(const LineSegment& line, float perp_l) {
    // Source: https://www.geeksforgeeks.org/find-points-at-a-given-distance-on-a-line-of-given-slope/
    
    Point mid_pt((line.start_pt.x+line.end_pt.x)/2, (line.start_pt.y+line.end_pt.y)/2, (line.start_pt.z+line.end_pt.z)/2);
    Point pt1(0, 0, mid_pt.z), pt2(0, 0, mid_pt.z);
    
    // Get the perpendicular gradient
    float m = -(line.end_pt.x-line.start_pt.x)/(line.end_pt.y-line.start_pt.y);

    if (m == 0) {
        pt1.x = mid_pt.x + perp_l;
        pt1.y = mid_pt.y;
 
        pt2.x = mid_pt.x - perp_l;
        pt2.y = mid_pt.y;
    }

    // if slope is infinite
    else if (m == std::numeric_limits<float>::max()) {
        pt1.x = mid_pt.x;
        pt1.y = mid_pt.y + perp_l;
 
        pt2.x = mid_pt.x;
        pt2.y = mid_pt.y - perp_l;
    }
    else {
        float dx = (perp_l / sqrt(1 + (m * m)));
        float dy = m * dx;
        pt1.x = mid_pt.x + dx;
        pt1.y = mid_pt.y + dy;
        pt2.x = mid_pt.x - dx;
        pt2.y = mid_pt.y - dy;
    }

    return std::vector<Point>({pt1, mid_pt, pt2});
}


/**
 * @brief A support class for door detection 
 */
class CloudProcessor {

private:
    #ifdef VISUALIZE
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    #endif

    float min_door_width_;
    float cluster_min_size_, cluster_max_size_, cluster_tolerance_;
    float line_min_points_, line_exact_min_points_;

    /**
     * @brief Find lines in a given pointcloud. This uses RANSAC from PCL.
     *  The function iterates over the given pointcloud searching for new lines.
     * 
     * @param input_cloud        Input point cloud
     * @param max_iterations     Max iterations for RANSAC
     * @param distance_threshold Distance threshold for RANSAC
     * @param cutoff_percentage  Stop searching for lines when pointcloud gets this small after 
     *                          removing previously detected lines
     * @param dense             Output lines may contain overlapping points from other lines that fit to the same model
     * @return FittedLineInfo*  
     */
    FittedLineInfo *  RANSAC_PointCloudLines(pcl::PointCloud<Point>::ConstPtr input_cloud, 
                        long int max_iterations,
                        float distance_threshold,
                        float cutoff_percentage, bool dense);

    /**
     * @brief Cluster given pointcloud. This uses Euclidean clustering from PCL
     * 
     * @param input_cloud        Input point cloud
     * @param min_cluster_size    Min cluster size
     * @param max_cluster_size   Max cluster size
     * @param cluster_tolerence  Cluster tolerance for Euclidean clustering
     * @return ClusterInfo* 
     */
    ClusterInfo *
    EuclideanClustering(pcl::PointCloud<Point>::ConstPtr input_cloud, long int min_cluster_size,
                        long int max_cluster_size,
                        float cluster_tolerence);
                        
    /**
     * @brief Find the two farthest points of a given pointcloud of a line.
     * 
     * @param line          Input line pointcloud
     * @param coefficients  Line coefficients
     * @return LineSegment  A line segment defined by the two farthest points
     */
    LineSegment getLineXYMinMaxPoints(pcl::PointCloud<Point>::ConstPtr line, 
                        pcl::ModelCoefficients::ConstPtr coefficients);

    /**
     * @brief Detect a door at a corner. i.e. When a door is formed by two perpendicular walls
     *  with a gap between the two
     * 
     * @param line1     Wall 1 segment
     * @param line2     Wall 2 segment
     * @param door      Output door line segment
     * @return true     When a door is detected
     * @return false    When a door can't be detected
     */
    bool detectCornerDoor(const LineSegment& line1, const LineSegment& line2, LineSegment& door);

    /**
     * @brief Detect a door on a wall. i.e. When a door is formed such that it split one wall into two
     * 
     * @param clusters      Clusters of the split wall
     * @param coefficients  Line coefficients of the wall line
     * @param door          Output door line segment
     * @return true         When a door is detected
     * @return false        When a door can't be detected
     */
    bool detectWallDoor(const ClusterInfo* const clusters, pcl::ModelCoefficients::Ptr& coefficients, LineSegment& door);

public:
    CloudProcessor(float min_door_width, float cluster_min_size, float cluster_max_size, float cluster_tolerance,
                   float line_min_points, float line_exact_min_points);

    /**
     * @brief Detect walls from a fiven pointcloud. The pointcloud is assumed to be from a 2D lidar, i.e. a flat pointcloud parallel 
     *  to the z plane
     * 
     * @param cloud                     Input point cloud
     * @return std::vector<LineSegment> Detected door line segments
     */
    std::vector<LineSegment> detect(const pcl::PointCloud<Point>::ConstPtr& cloud);

    ~CloudProcessor();
};

}  // namespace door_detector

#endif //DOOR_DETECTOR_CLOUD_PROCESSOR_H
