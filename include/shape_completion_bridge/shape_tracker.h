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
#include <vpp_msgs/InstancePointcloudwithCentroid.h>
#include <vpp_msgs/GetListInstancePointclouds.h>
#include <capsicum_superellipsoid_detector/se_detector.h>


namespace shape_completion_bridge{

typedef vpp_msgs::InstancePointcloudwithCentroid ObservedShapeInstance;
// struct ObservedShape
// {
//     size_t instance_id;
//     unsigned int num_points;
//     sensor_msgs::PointCloud2 pointcloud;
//     geometry_msgs::Pose pose;
// };

struct ShapeTrackerParams
{
    double param_position_diff_threshold = 0.1;
    unsigned param_size_diff_threshold = 20;
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

std::vector<ObservedShapeInstance> getNearestObservedShapes(double dist_threshold = 0, size_t num_shapes= 10);
std::vector<ObservedShapeInstance> getChangedObservedShapes();
std::vector<ObservedShapeInstance> getActiveObservedShapes(size_t num_shapes= 0);

void timerGetInstancePointclouds(const ros::TimerEvent& event);


void setCurrentObservedShapes(const std::vector<ObservedShapeInstance>& all_observed_shapes);

std::vector<size_t> getChangedObservedShapesIndices();

bool isShapeEqual(const ObservedShapeInstance& current, const ObservedShapeInstance& previous);

inline double calcPositionDiff(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
{
    geometry_msgs::Point diff;
    diff.x = a.x - b.x;
    diff.y = a.y - b.y;
    diff.z = a.z - b.z;

    return (fabs(diff.x) + fabs(diff.y) + fabs(diff.z));
}

inline bool stopTimerGetInstancePointclouds()
{
    timer_get_instance_pointclouds_.stop();
    return true;
}

inline bool startTimerGetInstancePointclouds()
{
    timer_get_instance_pointclouds_.start();
    return true;
}

private:

ros::NodeHandle nh_;
ros::NodeHandle nhp_;

std::vector<ObservedShapeInstance> all_observed_shapes_current_;
std::vector<ObservedShapeInstance> all_observed_shapes_previous_;

std::vector<size_t> changed_observed_shapes_indices_current_;

std::vector<size_t> active_observed_shapes_current_;
std::vector<size_t> active_observed_shapes_previous_;

ShapeTrackerParams shape_tracker_params_;

std::vector<ObservedShapeInstance> getObservedShapes();

bool observed_shapes_available_ = false; 

ros::ServiceClient client_get_instance_pointclouds_;
ros::Publisher pub_integrated_cloud_;

ros::Timer timer_get_instance_pointclouds_;


};
}
#endif