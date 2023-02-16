#ifndef SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H
#define SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H

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
#include <shape_completion_bridge_msgs/CompleteShapes.h>
#include <shape_completion_bridge_msgs/GetActiveMissingShapes.h>


#include "shape_completion_bridge/clustering.h"
#include "shape_completion_bridge/shape_completor.h"
#include "shape_completion_bridge/superellipsoid_volume.h"
#include <shape_completion_bridge/shape_tracker.h>


namespace shape_completion_bridge
{

class ShapeCompletionService
{
public:
    ShapeCompletionService();
    //ShapeCompletionService(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    bool processCompleteShapesServiceCallback(shape_completion_bridge_msgs::CompleteShapes::Request& req, shape_completion_bridge_msgs::CompleteShapes::Response& res);
    bool processGetActiveMissingShapesServiceCallback(shape_completion_bridge_msgs::GetActiveMissingShapes::Request& req, shape_completion_bridge_msgs::GetActiveMissingShapes::Response& res);

    // bool processEvaluateShapesServiceCallback(shape_completion_bridge_msgs::EvaluateShapes::Request& req, shape_completion_bridge_msgs::EvaluateShapes::Response& res);

    void timerCallback(const ros::TimerEvent& event);
    void timerSuperellipsoidFitterCallback(const ros::TimerEvent& event);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Timer timer_publisher_, timer_sefitter_, timer_get_instance_pointclouds_;
    
    ros::Publisher pub_surface_, pub_volume_, pub_missing_surface_, pub_missing_surface_cluster_centres_;
    sensor_msgs::PointCloud2Ptr pc_ros_surf_, pc_ros_vol_, pc_ros_missing_surf_;
    bool publish_pointclouds_;

    ros::ServiceServer service_shape_completion_, service_get_active_predicted_shapes_;

    std::string p_world_frame_;
    std::unique_ptr<tf::TransformListener> listener_;
    std_msgs::Header pc_obs_pcl_tf_ros_header_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl_;

    std::string shape_completor_type_;
    std::unique_ptr<ShapeCompletor> p_shape_completor_;
    uint8_t last_shape_completion_method_;

    std::vector<ShapeCompletorResult> shape_completor_result_;
    bool shape_completor_result_available_;

    bool readPointCloudFromTopic();

    void selectShapeCompletionMethod(const uint8_t& shape_completion_method);
    
    void storeShapeCompletorResult(std::vector<ShapeCompletorResult>& src);
    std::vector<ShapeCompletorResult> getShapeCompletorResult();

    int p_min_cluster_size, p_max_cluster_size, p_max_num_iterations, p_cost_type, p_missing_surfaces_num_samples;
    double p_cluster_tolerance, p_estimate_normals_search_radius, p_estimate_cluster_center_regularization, p_pointcloud_volume_resolution, p_octree_volume_resolution, p_prior_scaling, p_prior_center, p_missing_surfaces_threshold;
    bool p_print_ceres_summary, p_use_fibonacci_sphere_projection_sampling;
    std::string p_world_frame;
    ros::Publisher pub_superellipsoids_;

    ShapeTracker shape_tracker_; 
    superellipsoid::SuperellipsoidDetector se_detector_;

};

}
#endif //SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H