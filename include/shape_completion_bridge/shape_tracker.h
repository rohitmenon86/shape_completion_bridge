#ifndef SHAPE_COMPLETION_BRIDGE_SHAPE_TRACKER_H
#define SHAPE_COMPLETION_BRIDGE_SHAPE_TRACKER_H
#include<ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/io.h>
#include <pcl/common/distances.h>

namespace shape_completion_bridge{

struct ObservedShape
{
    size_t instance_id;
    unsigned int num_points;
    sensor_msgs::PointCloud2 pointcloud;
    geometry_msgs::Pose pose;
};

struct ShapeTrackerParams
{
    double param_position_diff_threshold = 0.1;
    unsigned param_size_diff_threshold = 10;
    double param_nearness_threshold = 0.3;
    bool use_nearness = true;
};

struct compare
{
    int key;
    compare(int const &i): key(i) {}
 
    bool operator()(int const &i) {
        return (i == key);
    }
};

class ShapeTracker{

public:

ShapeTracker();

std::vector<ObservedShape> getNearestObservedShapes(double dist_threshold = 0, size_t num_shapes= 10);
std::vector<ObservedShape> getChangedObservedShapes(size_t num_shapes= 10);
std::vector<ObservedShape> getActiveObservedShapes(size_t num_shapes= 0);

inline void setCurrentObservedShapes(const std::vector<ObservedShape>& all_observed_shapes)
{
    all_observed_shapes_previous_.clear();
    all_observed_shapes_previous_.assign(all_observed_shapes_current_.begin(),all_observed_shapes_current_.end());
    all_observed_shapes_current_.clear();
    all_observed_shapes_current_.assign(all_observed_shapes.begin(),all_observed_shapes.end());
}

void compareAllObservedShapesByShapeParams();

bool isShapeEqual(const ObservedShape& current, const ObservedShape& previous);

inline double calcPositionDiff(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    geometry_msgs::Point diff;
    diff.x = a.x - b.x;
    diff.y = a.y - b.y;
    diff.z = a.z - b.z;

    return (fabs(diff.x) + fabs(diff.y) + fabs(diff.z));
}

private:

std::vector<ObservedShape> all_observed_shapes_current_;
std::vector<ObservedShape> all_observed_shapes_previous_;

std::vector<size_t> active_observed_shapes_current_;
std::vector<size_t> active_observed_shapes_previous_;

ShapeTrackerParams shape_tracker_params_;


};
}
#endif