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

typedef struct LineSegment
{
    Point startPt, endPt;
} LineSegment;


typedef struct FittedLineInfo
{
    FittedLineInfo() : num_lines(0) {};
    std::vector<pcl::PointCloud<Point>::Ptr> lines;
    std::vector<LineSegment> lines_min_max_points;
    std::vector<pcl::ModelCoefficients::Ptr> lines_coefficients;
    int num_lines;
} FittedLineInfo;

typedef struct ClusterInfo
{
    ClusterInfo() : num_clusters(0) {};
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    int num_clusters;
} ClusterInfo;

class CloudProcessor {

private:
    #ifdef VISUALIZE
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    #endif

    float min_door_width_;
    float cluster_min_size_, cluster_max_size_, cluster_tolerance_;
    float line_min_points_, line_exact_min_points_;

    FittedLineInfo *  RANSAC_PointCloudLines(pcl::PointCloud<Point>::ConstPtr inputCloud, 
                        long int maxIterations,
                        float distanceThreshold,
                        float cutoffPercentage, bool dense);

    ClusterInfo *
    EuclideanClustering(pcl::PointCloud<Point>::ConstPtr inputCloud, long int minClusterSize,
                        long int maxCluseterSize,
                        float clusterTolerance);
                        
    LineSegment getLineXYMinMaxPoints(pcl::PointCloud<Point>::ConstPtr line, 
                        pcl::ModelCoefficients::ConstPtr coefficients);

    bool detectCornerDoor(const LineSegment& line1, const LineSegment& line2, LineSegment& door);

public:
    CloudProcessor(float min_door_width, float cluster_min_size, float cluster_max_size, float cluster_tolerance,
                   float line_min_points, float line_exact_min_points);

    void detect(const pcl::PointCloud<Point>::ConstPtr& cloud);

    void spinOnce();

    ~CloudProcessor();
};

}  // namespace door_detector

#endif //DOOR_DETECTOR_CLOUD_PROCESSOR_H
