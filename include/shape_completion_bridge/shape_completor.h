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
#include <pcl/common/distances.h>

#include <shape_completion_bridge_msgs/CompleteShapes.h>
#include <shape_completion_bridge_msgs/ClusteredShape.h>
#include <shape_completion_bridge_msgs/SuperellipsoidResult.h>

#include <shape_completion_bridge_msgs/FitSuperellipsoids.h>
#include <shape_completion_bridge_msgs/RegisterShape.h>
#include <shape_completion_bridge_msgs/RoiData.h>

enum MissingSurfaceClusterCentreType
{
    INDIVIDUAL_SIMPLE_CLUSTER_MEAN = 0,
    INDIVIDUAL_ROIPROB_WEIGHTED_CLUSTER_MEAN = 1,
    RECLUSTERED_SIMPLE_CLUSTER_MEAN = 2,
    RECLUSTERED_ROIPROB_WEIGHTED_CLUSTER_MEAN = 3
};



#include "shape_completion_bridge/clustering.h"

namespace shape_completion_bridge
{
static bool abs_compare(shape_completion_bridge_msgs::RoiData a, shape_completion_bridge_msgs::RoiData b)
{
    return (fabs(a.roi_probability) < fabs(b.roi_probability));
}
static bool abs_compare_min(shape_completion_bridge_msgs::RoiData a, shape_completion_bridge_msgs::RoiData b)
{
    return (std::abs(a.roi_probability) > std::abs(b.roi_probability));
}
struct ShapeCompletorResult
{
    bool valid_prediction;
    pcl::PointCloud<pcl::PointXYZ>::Ptr observed_pointcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_volume_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_surface_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_missing_surface_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_cloud_normals;
    geometry_msgs::Transform rigid_local_transform;
    geometry_msgs::Point cluster_centre;
    Eigen::Vector4f missing_surface_cluster_centre;


    ShapeCompletorResult():valid_prediction(false), observed_pointcloud(NULL), predicted_volume_cloud(NULL),predicted_surface_cloud(NULL),predicted_missing_surface_cloud(NULL),predicted_cloud_normals(NULL)
    {}   
};  

struct PointCloudPCLwithRoiData
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::vector<shape_completion_bridge_msgs::RoiData> roi_data;
};

class ShapeCompletor
{
    public:
    ShapeCompletor(ros::NodeHandle nh, ros::NodeHandle nhp);

    virtual bool completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result) = 0;
    PointCloudPCLwithRoiData createCompleteShape(std::vector<ShapeCompletorResult>& shape_completor_result);


    protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    int shape_completion_type_; 

    ros::ServiceClient client_shape_completor_;


    std::unique_ptr<tf::TransformListener> listener_;
    std_msgs::Header pc_obs_pcl_tf_ros_header_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl_;
    

    int p_min_cluster_size_, p_max_cluster_size_;
    double p_cluster_tolerance_, p_estimate_normals_search_radius_, p_estimate_cluster_center_regularization_;

    double mean_, stddev_, variance_;

    double roi_prob_min_, roi_prob_max_;

    bool shift_origin_ = false; 
    bool activate_downsampling_ = false;

    std::vector<shape_completion_bridge_msgs::ClusteredShape> clustered_shapes_;

    bool createClusteredShapes(bool shift_origin = false);

    PointCloudPCLwithRoiData processShapeCompletorResult(const ShapeCompletorResult& shape_completor_result);
    PointCloudPCLwithRoiData processFullPredictionCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr volume_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr observed_cloud);
    PointCloudPCLwithRoiData processMissingPredictionCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_missing_surface_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr observed_pointcloud);

    std::vector<shape_completion_bridge_msgs::RoiData>  calcRoiData(pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pred_cloud);
    double calcMinimumDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cloud, pcl::PointXYZ& point);
    double calcRoiProbability(double dist);

    std::vector<geometry_msgs::Point> calcMissingSurfaceClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi, MissingSurfaceClusterCentreType clustering_type);
    geometry_msgs::Point calcIndividualSimpleClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi);
    geometry_msgs::Point calcIndividualRoiProbWeightedClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi);
    std::vector<geometry_msgs::Point> calcReclusteredSimpleClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi);
    std::vector<geometry_msgs::Point> calcReclusteredRoiProbWeightedClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi);


    inline int getShapeCompletionType()
    {
        return shape_completion_type_;
    }

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
    void fromShapeRegistrationResult2ShapeCompletorResult(const shape_completion_bridge_msgs::ShapeRegistrationResult& src, ShapeCompletorResult& dest);

};



}

#endif //SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETOR_H
