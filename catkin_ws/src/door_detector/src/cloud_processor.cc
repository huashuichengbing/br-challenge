#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "door_detector/cloud_processor.h"

#define LINE_MIN_PTS 30
#define LINE_MIN_FIT_PTS (LINE_MIN_PTS/2)
#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)      std::cout << x
#define DEBUG_PRINTLN(x)    DEBUG_PRINT(x) << std::endl
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

namespace cloud_processor {

using std::string;

CloudProcessor::CloudProcessor()
#ifdef VISUALIZE
: viewer_(new pcl::visualization::PCLVisualizer("PCL Viewer"))
#endif
{
    DEBUG_PRINTLN("PCL Version: " << PCL_MAJOR_VERSION << "." << PCL_MINOR_VERSION << "." << PCL_REVISION_VERSION);
}

FittedLineInfo *
CloudProcessor::RANSAC_PointCloudLines(pcl::PointCloud<Point>::ConstPtr inputCloud, long int maxIterations,
                        float distanceThreshold,
                        float cutoffPercentage, bool dense) {
    FittedLineInfo *retLines = new FittedLineInfo;
    retLines->numberOfLines = 0;

    if (retLines == NULL) {
        return NULL;
    }

    pcl::SampleConsensusModelLine<Point>::Ptr obj(
            new pcl::SampleConsensusModelLine<Point>(inputCloud));

    pcl::PointCloud<Point>::Ptr localInputCloud;
    localInputCloud = inputCloud->makeShared();

    pcl::PointCloud<Point>::Ptr cloud_p(new pcl::PointCloud<Point>), cloud_f(new pcl::PointCloud<Point>);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<Point> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices), exact_inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(static_cast<int>(maxIterations));
    seg.setDistanceThreshold(distanceThreshold);

    // Create the filtering object
    pcl::ExtractIndices<Point> extract;
    int nr_points = (int) localInputCloud->points.size();
    bool skip = false;

    while (localInputCloud->points.size() > cutoffPercentage * nr_points) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // Segment the largest model fit from the remaining cloud
        seg.setInputCloud(localInputCloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty()) {
            std::cerr << "Could not estimate a line model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers from the actual inputLine, not the one that's shrinking.
        // this way we get all points in the plane fitted.  We'll handle segmenatation
        // and deletion of small patches in other fitted lines later.
        // We just use the coefficients and find new inliers/indices
        float *ptr = &(coefficients->values[0]);
        Eigen::Map<Eigen::VectorXf> coeff(ptr, coefficients->values.size());

        obj->selectWithinDistance(coeff, distanceThreshold, exact_inliers->indices);
        extract.setInputCloud(inputCloud);
        extract.setIndices(exact_inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);


        if (exact_inliers->indices.size() < LINE_MIN_PTS || inliers->indices.size() < LINE_MIN_FIT_PTS)
            skip = true;

        if (!skip) {
            if (dense) {
                retLines->lines.push_back(cloud_p->makeShared());
                LineMinMaxPoints pts;
                pcl::getMinMax3D (*cloud_p, pts.minPt, pts.maxPt);
                retLines->lines_min_max_points.push_back(pts);
            }

            retLines->lines_coefficients.push_back(coefficients);
            retLines->numberOfLines++;
        }

        //This one gets the points cut out of it.
        extract.setInputCloud(localInputCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        if (!skip) {
            if (!dense) {
                retLines->lines.push_back(cloud_p->makeShared());
                LineMinMaxPoints pts;
                pcl::getMinMax3D (*cloud_p, pts.minPt, pts.maxPt);
                retLines->lines_min_max_points.push_back(pts);
            }
        }

        extract.setNegative(true);
        extract.filter(*cloud_f);
        localInputCloud.swap(cloud_f);
        skip = false;
    }

    return retLines;
}

void CloudProcessor::run(const pcl::PointCloud<Point>::ConstPtr& cloud) {

#ifdef  VISUALIZE
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();

    pcl::visualization::PointCloudColorHandlerCustom<Point> color(cloud, 255, 0, 0);
    viewer_->addPointCloud<Point>(cloud, color, "cloud");
#endif

    auto lines = RANSAC_PointCloudLines(cloud, 1000, 0.01, 0.1, true);
    for (int i = 0; i < lines->numberOfLines - 1; i++) {
        // auto line_w = lines->lines_coefficients[i];
        auto line_w = lines->lines_min_max_points[i];

#ifdef  VISUALIZE
        int r = rand() % 255, g = rand() % 255, b = rand() % 255;
        viewer_->addLine(line_w.maxPt, line_w.minPt, r/ 255.0, g/ 255.0, b/ 255.0, "L"+std::to_string(i));

        // pcl::visualization::PointCloudColorHandlerCustom<Point> color(lines->lines[i], r, g, b);
        // viewer_->addPointCloud<Point>(lines->lines[i], color, "C"+std::to_string(i));
#endif
        // DEBUG_PRINTLN("#lines: " << lines->numberOfLines);
        // DEBUG_PRINTLN("line " << i << ": " << line_w.maxPt << line_w.minPt);
    }
}

void CloudProcessor::spinOnce() {
#ifdef  VISUALIZE
        viewer_->spinOnce();
#endif
}

CloudProcessor::~CloudProcessor() {
#ifdef VISUALIZE
    viewer_->close();
#endif
}

}

