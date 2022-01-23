/**
 * @file cloud_processor.cc
 * @author Teshan Liyanage (teshanuka@gmail.com)
 * @brief Cloud processing library for door detection from a pointcloud
 * @version 0.1
 * @date 2022-01-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

#include "door_detector/cloud_processor.h"

namespace door_detector {


inline float pointToLineDistanceXY(const Point& point, const LineSegment& line) {
    // reference: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    float a = line.end_pt.y - line.start_pt.y;
    float b = line.start_pt.x - line.end_pt.x;
    float c = line.end_pt.x * line.start_pt.y - line.start_pt.x * line.end_pt.y;
    return fabs(a * point.x + b * point.y + c) / sqrt(a * a + b * b);
}


inline Point lineSegmentIntersection(const LineSegment& line1, const LineSegment& line2, float& t, float& u) {
    /* reference: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    There will be an intersection if 0.0 ≤ t ≤ 1.0 and 0.0 ≤ u ≤ 1.0. The intersection point falls within the 
    first line segment if 0.0 ≤ t ≤ 1.0, and it falls within the second line segment if 0.0 ≤ u ≤ 1.0.*/
    float x1 = line1.start_pt.x;
    float y1 = line1.start_pt.y;
    float x2 = line1.end_pt.x;
    float y2 = line1.end_pt.y;
    float x3 = line2.start_pt.x;
    float y3 = line2.start_pt.y;
    float x4 = line2.end_pt.x;
    float y4 = line2.end_pt.y;

    float denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    if (denominator == 0)
        return Point(0, 0, 0);

    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4))/denominator;
    u = ((x1-x3)*(y1-y2) - (y1-y3)*(x1-x2))/denominator;

    float xi = x1 + t * (x2 - x1);
    float yi = y1 + t * (y2 - y1);
    return Point(xi, yi, 0);
}


inline float angleBetweenLinesXY(const LineSegment& line1, const LineSegment& line2) {
    // reference: https://www.math-only-math.com/angle-between-two-straight-lines.html
    float m1 = (line1.end_pt.y - line1.start_pt.y) / (line1.end_pt.x - line1.start_pt.x);
    float m2 = (line2.end_pt.y - line2.start_pt.y) / (line2.end_pt.x - line2.start_pt.x);

    float angle = atan2(m1-m2, 1+m1*m2);
    return angle;
}


#ifdef  VISUALIZE
inline void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<Point>::ConstPtr cloud, const FittedLineInfo* const lines, const std::vector<LineSegment>& doors){
    static float text_id_ = 0;

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    pcl::visualization::PointCloudColorHandlerCustom<Point> color(cloud, 255, 255, 255);
    viewer->addPointCloud<Point>(cloud, color, "cloud");

    for (int i = 0; i < lines->num_lines; i++) {
        // auto line_w = lines->lines_coefficients[i];
        auto line_w = lines->lines_min_max_points[i];

        int r = rand() % 255, g = rand() % 255, b = rand() % 255;
        viewer->addLine(line_w.end_pt, line_w.start_pt, r/ 255.0, g/ 255.0, b/ 255.0, "L"+std::to_string(i));
        viewer->addSphere(line_w.start_pt, 0.02, r/ 255.0, g/ 255.0, b/ 255.0, "Ss"+std::to_string(i));
        viewer->addSphere(line_w.end_pt, 0.02, r/ 255.0, g/ 255.0, b/ 255.0, "Se"+std::to_string(i));
    }
    viewer->addText("# lines: "+std::to_string(lines->num_lines), 10, 30, "Tl"+std::to_string(text_id_++));

    int i(0);
    for (const auto& door: doors) {
        viewer->addLine(door.end_pt, door.start_pt, 1, 1, 1, "Ld"+std::to_string(i));
        viewer->addSphere(door.start_pt, 0.05, 1, 1, 1, "Ds"+std::to_string(i));
        viewer->addSphere(door.end_pt, 0.05, 1, 1, 1, "De"+std::to_string(i++));
    }
    viewer->addText("# doors: "+std::to_string(doors.size()), 10, 60, "Td"+std::to_string(text_id_++));

    // Show detected paths
    i = 0;
    for (const auto& door: doors) {
        auto points = getPerpendicularPoints(door, 0.5);
        std::sort(points.begin(), points.end(), [&points](const Point& p1, const Point& p2){
            return lineLengthXY(LineSegment(p1, Point(0, 0, 0))) < lineLengthXY(LineSegment(p2, Point(0, 0, 0)));
        });
        viewer->addLine(points[0], points[1], 0, 1, 0, "Pt"+std::to_string(i++));
        viewer->addLine(points[1], points[2], 0, 1, 0, "Pt"+std::to_string(i++));
    }

    viewer->spinOnce();
}
#endif


CloudProcessor::CloudProcessor(float min_door_width, float cluster_min_size, float cluster_max_size, float cluster_tolerance,
                               float line_min_points, float line_exact_min_points) : 
    min_door_width_(min_door_width),
    cluster_min_size_(cluster_min_size),
    cluster_max_size_(cluster_max_size),
    cluster_tolerance_(cluster_tolerance),
    line_min_points_(line_min_points),
    line_exact_min_points_(line_exact_min_points),
    #ifdef VISUALIZE
    viewer_(new pcl::visualization::PCLVisualizer("PCL Viewer"))
    #endif
{
    #ifdef VISUALIZE
    viewer_->addCoordinateSystem(0.3);
    viewer_->setBackgroundColor(0, 0, 0);
    #endif

    DEBUG_PRINTLN("PCL Version: " << PCL_MAJOR_VERSION << "." << PCL_MINOR_VERSION << "." << PCL_REVISION_VERSION);
}


LineSegment CloudProcessor::getLineXYMinMaxPoints(pcl::PointCloud<Point>::ConstPtr line, pcl::ModelCoefficients::ConstPtr coefficients)
{
    float gradient = (coefficients->values[4] - coefficients->values[1]) / (coefficients->values[3] - coefficients->values[0]);
    // To avoid errors caused by underfit points with too horizontal or vertical lines, we check the gradient
    // either x or y coordinate can be used to deterimine the min/max points depending on how the line is oriented
    bool more_vertical = (gradient > 1.0) || (gradient < -1.0);

    LineSegment edge_points;
    edge_points.start_pt = line->points[0];
    edge_points.end_pt = line->points[0];
    for (int i = 1; i < line->points.size(); i++)
    {
        if (more_vertical) {
            if (line->points[i].y < edge_points.start_pt.y)
                edge_points.start_pt = line->points[i];
            if (line->points[i].y > edge_points.end_pt.y)
                edge_points.end_pt = line->points[i];
        } else {
            if (line->points[i].x < edge_points.start_pt.x)
                edge_points.start_pt = line->points[i];
            if (line->points[i].x > edge_points.end_pt.x)
                edge_points.end_pt = line->points[i];
        }
    }
    return edge_points;
}


bool CloudProcessor::detectCornerDoor(const LineSegment& line1, const LineSegment& line2, LineSegment& door) 
{
    // If lines are not approximately perpendicular, it is not considered a corner door
    if (fabs(fabs(angleBetweenLinesXY(line1, line2)) - M_PI / 2) > M_PI/12)
        return false;

    float t, u;
    auto intersection_pt = lineSegmentIntersection(line1, line2, t, u);

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
        // Segments intersect. If lines intersect, there is no door
        return false;
    }

    if (t >= 0 && t <= 1 ) {
        // Intersection on line 1
        auto l2_start_dist = pointToLineDistanceXY(line2.start_pt, line1);
        auto l2_end_dist = pointToLineDistanceXY(line2.end_pt, line1);

        door.start_pt = (l2_start_dist < l2_end_dist) ? line2.start_pt : line2.end_pt;
        door.end_pt = intersection_pt;

        return true;
    }
    if (u >= 0 && u <= 1) {
        // Intersection on line 2
        auto l1_start_dist = pointToLineDistanceXY(line1.start_pt, line2);
        auto l1_end_dist = pointToLineDistanceXY(line1.end_pt, line2);

        door.start_pt = (l1_start_dist < l1_end_dist) ? line1.start_pt : line1.end_pt;
        door.end_pt = intersection_pt;

        return true;
    }
    
    return false;
}


bool CloudProcessor::detectWallDoor(const ClusterInfo* const clusters, pcl::ModelCoefficients::Ptr& coefficients, LineSegment& door)
{
    // We need more than two clusters to find a door between them
    if (clusters->num_clusters < 2)
        return false;

    FittedLineInfo lines;
    lines.num_lines = clusters->num_clusters;
    for(const auto& cluster: clusters->clusters){
        lines.lines_coefficients.push_back(coefficients);
        lines.lines_min_max_points.push_back(getLineXYMinMaxPoints(cluster, coefficients));
    }

    float min_len = 10000.0f;
    for (int i=0; i<2; i++){
        for (int j=0; j<2; j++){
            LineSegment ls;
            if (i==0)
                ls.start_pt = lines.lines_min_max_points[0].start_pt;
            else
                ls.start_pt = lines.lines_min_max_points[0].end_pt;
            if (j==0)
                ls.end_pt = lines.lines_min_max_points[1].start_pt;
            else
                ls.end_pt = lines.lines_min_max_points[1].end_pt;

            auto this_len = lineLengthXY(ls);
            if (this_len < min_len) {
                door = ls;
                min_len = this_len;
            }
        }
    }
    return true;
}


FittedLineInfo *
CloudProcessor::RANSAC_PointCloudLines(pcl::PointCloud<Point>::ConstPtr input_cloud, long int max_iterations,
                        float distance_threshold,
                        float cutoff_percentage, bool dense) {
    FittedLineInfo *ret_lines = new FittedLineInfo;

    pcl::SampleConsensusModelLine<Point>::Ptr obj(
            new pcl::SampleConsensusModelLine<Point>(input_cloud));

    pcl::PointCloud<Point>::Ptr local_input_cloud;
    local_input_cloud = input_cloud->makeShared();

    pcl::PointCloud<Point>::Ptr cloud_p(new pcl::PointCloud<Point>), cloud_f(new pcl::PointCloud<Point>);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<Point> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices), actual_inliers(new pcl::PointIndices);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(static_cast<int>(max_iterations));
    seg.setDistanceThreshold(distance_threshold);

    // Create the filtering object
    pcl::ExtractIndices<Point> extract;
    int nr_points = (int) local_input_cloud->points.size();
    bool skip = false;

    while (local_input_cloud->points.size() > cutoff_percentage * nr_points) {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        // Segment the largest model fit from the remaining cloud
        seg.setInputCloud(local_input_cloud);
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

        obj->selectWithinDistance(coeff, distance_threshold, actual_inliers->indices);
        extract.setInputCloud(input_cloud);
        extract.setIndices(actual_inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        if (actual_inliers->indices.size() < line_min_points_|| inliers->indices.size() < line_exact_min_points_)
            skip = true;

        if (!skip) {
            if (dense) {
                ret_lines->lines.push_back(cloud_p->makeShared());
                LineSegment pts = getLineXYMinMaxPoints(cloud_p, coefficients);
                ret_lines->lines_min_max_points.push_back(pts);
            }

            ret_lines->lines_coefficients.push_back(coefficients);
            ret_lines->num_lines++;
        }

        //This one gets the points cut out of it.
        extract.setInputCloud(local_input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);

        if (!skip) {
            if (!dense) {
                ret_lines->lines.push_back(cloud_p->makeShared());
                LineSegment pts = getLineXYMinMaxPoints(cloud_p, coefficients);
                ret_lines->lines_min_max_points.push_back(pts);
            }
        }

        extract.setNegative(true);
        extract.filter(*cloud_f);
        local_input_cloud.swap(cloud_f);
        skip = false;
    }

    return ret_lines;
}


ClusterInfo *
CloudProcessor::EuclideanClustering(pcl::PointCloud<Point>::ConstPtr input_cloud, long int min_cluster_size,
                    long int max_cluster_size,
                    float cluster_tolerance) 
{
    ClusterInfo *ret_clusters = new ClusterInfo;

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(static_cast<int>(min_cluster_size));
    ec.setMaxClusterSize(static_cast<int>(max_cluster_size));
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud);
    ec.extract(cluster_indices);

    ret_clusters->num_clusters = static_cast<int>(cluster_indices.size());

    for (const auto &it: cluster_indices) {
        pcl::PointCloud<Point>::Ptr cloud_cluster(new pcl::PointCloud<Point>);

        for (const auto &pit: it.indices) {
            cloud_cluster->points.push_back(input_cloud->points[pit]); //*
        }

        cloud_cluster->width = static_cast<uint32_t>(cloud_cluster->points.size());
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        ret_clusters->clusters.push_back(cloud_cluster);
    }

    return ret_clusters;
}


std::vector<LineSegment> CloudProcessor::detect(const pcl::PointCloud<Point>::ConstPtr& cloud) 
{
    auto lines = RANSAC_PointCloudLines(cloud, 1000, 0.01, 0.05, false);

    std::vector<LineSegment> doors;
    LineSegment tmp_door;
    for (int i = 0; i < lines->num_lines - 1; i++) {
        for (int j = i+1; j < lines->num_lines; j++) {
            auto line_i = lines->lines_min_max_points[i];
            auto line_j = lines->lines_min_max_points[j];

            if (detectCornerDoor(line_i, line_j, tmp_door) && lineLengthXY(tmp_door) >= min_door_width_) {
                doors.push_back(tmp_door);
            }
        }
    }

    for (int i = 0; i < lines->num_lines; i++) {
        auto line_cloud = lines->lines[i];
        auto clusters = EuclideanClustering(line_cloud, cluster_min_size_, cluster_max_size_, cluster_tolerance_);

        if (clusters->num_clusters < 2)
            continue;

        if (detectWallDoor(clusters, lines->lines_coefficients[i], tmp_door) && lineLengthXY(tmp_door) >= min_door_width_) {
            doors.push_back(tmp_door);
        }

        // Free `clusters` memory
        clusters->clusters.clear();
        clusters->clusters.shrink_to_fit();
        free(clusters);
    }

    #ifdef  VISUALIZE
    visualize(viewer_, cloud, lines, doors);
    #endif

    // Free `lines` memory
    lines->lines.clear();
    lines->lines.shrink_to_fit();
    lines->lines_coefficients.clear();
    lines->lines_coefficients.shrink_to_fit();
    free(lines);

    return doors;
}


CloudProcessor::~CloudProcessor() {
    #ifdef VISUALIZE
    viewer_->close();
    #endif
}

}  // namespace door_detector
