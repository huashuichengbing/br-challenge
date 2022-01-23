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
    FittedLineInfo() : numberOfLines(0) {};
    std::vector<pcl::PointCloud<Point>::Ptr> lines;
    std::vector<LineSegment> lines_min_max_points;
    std::vector<pcl::ModelCoefficients::Ptr> lines_coefficients;
    int numberOfLines;
} FittedLineInfo;

class CloudProcessor {
#ifdef VISUALIZE
    // int text_id_;
    pcl::visualization::PCLVisualizer::Ptr viewer_;
#endif

    float min_door_width_;

    FittedLineInfo *  RANSAC_PointCloudLines(pcl::PointCloud<Point>::ConstPtr inputCloud, 
                        long int maxIterations,
                        float distanceThreshold,
                        float cutoffPercentage, bool dense);

    LineSegment getLineXYMinMaxPoints(pcl::PointCloud<Point>::ConstPtr line, 
                        pcl::ModelCoefficients::ConstPtr coefficients);

    bool detectCornerDoor(const LineSegment& line1, const LineSegment& line2, LineSegment& door);

public:
    CloudProcessor(float min_door_width);

    void detect(const pcl::PointCloud<Point>::ConstPtr& cloud);

    void spinOnce();

    ~CloudProcessor();
};

}  // namespace door_detector

#endif //DOOR_DETECTOR_CLOUD_PROCESSOR_H
