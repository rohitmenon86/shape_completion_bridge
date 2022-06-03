#ifndef SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETOR_H
#define SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETOR_H

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
#include <shape_completion_bridge_msgs/ClusteredShape.h>
#include <shape_completion_bridge_msgs/SuperellipsoidResult.h>

#include <shape_completion_bridge_msgs/FitSuperellipsoids.h>
#include <shape_completion_bridge_msgs/RegisterShape.h>




#include "shape_completion_bridge/clustering.h"

namespace shape_completion_bridge
{

struct ShapeCompletorResult
{
    sensor_msgs::PointCloud2 observed_cloud;
    sensor_msgs::PointCloud2 predicted_volume_cloud;
    sensor_msgs::PointCloud2 predicted_surface_cloud;
    sensor_msgs::PointCloud2 predicted_cloud_normals;
    geometry_msgs::Transform rigid_local_transform;
};    

class ShapeCompletor
{
    public:
    ShapeCompletor(ros::NodeHandle nh, ros::NodeHandle nhp);

    virtual bool completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result) = 0;
    

    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::ServiceClient client_shape_completor_;


    std::unique_ptr<tf::TransformListener> listener_;
    std_msgs::Header pc_obs_pcl_tf_ros_header_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl_;
    

    int p_min_cluster_size_, p_max_cluster_size_;
    double p_cluster_tolerance_, p_estimate_normals_search_radius_, p_estimate_cluster_center_regularization_;

    std::vector<shape_completion_bridge_msgs::ClusteredShape> clustered_shapes_;

    bool createClusteredShapes();
}; 

class SuperellipsoidFitter: public ShapeCompletor
{
    public:
    SuperellipsoidFitter(ros::NodeHandle nh, ros::NodeHandle nhp);
    bool completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result);

    protected:
    void fromSuperelipsoidResult2ShapeCompletorResult(std::vector<shape_completion_bridge_msgs::SuperellipsoidResult> & src, std::vector<ShapeCompletorResult> & dest);
};

class ShapeRegister: public ShapeCompletor
{
    public:
    ShapeRegister(ros::NodeHandle nh, ros::NodeHandle nhp);
    bool completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result);
};



}

#endif //SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETOR_H
