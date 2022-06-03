#include "shape_completion_bridge/shape_completor.h"
namespace shape_completion_bridge
{

ShapeCompletor::ShapeCompletor(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh), nhp_(nhp)
{
    ROS_WARN("ShapeCompletor Constructor");

    nhp_.param("p_min_cluster_size", p_min_cluster_size_, 100);
    nhp_.param("p_max_cluster_size", p_max_cluster_size_, 10000);
    nhp_.param("p_cluster_tolerance", p_cluster_tolerance_, 0.01);
    nhp_.param("p_estimate_normals_search_radius", p_estimate_normals_search_radius_, 0.015);
    nhp_.param("p_estimate_cluster_center_regularization", p_estimate_cluster_center_regularization_, 2.5);

}

bool ShapeCompletor::createClusteredShapes()
{
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_obs_pcl_, cluster_indices, p_cluster_tolerance_, p_min_cluster_size_, p_max_cluster_size_);
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::experimentalClustering(pc_pcl, cluster_indices);

    // TODO: CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

    // Initialize superellipsoids
    clustered_shapes_.clear();
    for (const auto& current_cluster_pc : clusters)
    {
        auto new_clustered_shape = std::make_shared<clustering::ClusteredShape<pcl::PointXYZRGB>>(current_cluster_pc);
        new_clustered_shape->estimateNormals(p_estimate_normals_search_radius_); // search_radius
        new_clustered_shape->estimateClusterCenter(p_estimate_cluster_center_regularization_); // regularization
        shape_completion_bridge_msgs::ClusteredShape clustered_shape_msg;
        pcl::toROSMsg(*current_cluster_pc, clustered_shape_msg.cluster_pointcloud);
        new_clustered_shape->getEstimatedCenter(clustered_shape_msg.estimated_centre.x, clustered_shape_msg.estimated_centre.y, clustered_shape_msg.estimated_centre.z); 
        clustered_shapes_.push_back(clustered_shape_msg);
    }
}

SuperellipsoidFitter::SuperellipsoidFitter(ros::NodeHandle nh, ros::NodeHandle nhp):ShapeCompletor(nh, nhp)
{
    ROS_WARN("Superellipsoid Fitter Constructor");
    client_shape_completor_ = nh_.serviceClient<shape_completion_bridge_msgs::FitSuperellipsoids>("superellipsoid_detector");
}

bool SuperellipsoidFitter::completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result)
{
    pc_obs_pcl_ = pc_obs_pcl;
    createClusteredShapes();
    shape_completion_bridge_msgs::FitSuperellipsoids srv;
    srv.request.clustered_shapes = clustered_shapes_;

    if (client_shape_completor_.call(srv))
    {
        fromSuperelipsoidResult2ShapeCompletorResult(srv.response.superellipsoids, result);
        return true;
    }
    else
    {
        return false;
    }


}

void SuperellipsoidFitter::fromSuperelipsoidResult2ShapeCompletorResult(std::vector<shape_completion_bridge_msgs::SuperellipsoidResult> & src, std::vector<ShapeCompletorResult> & dest)
{
    dest.clear();
    for(const auto element:src)
    {
        ShapeCompletorResult res_element;
        res_element.observed_cloud          = element.observed_pointcloud;     
        res_element.predicted_volume_cloud  = element.volume_pointcloud;
        res_element.predicted_surface_cloud = element.surface_pointcloud;
        res_element.predicted_cloud_normals = element.normals;
        dest.push_back(res_element);
    }        
}


ShapeRegister::ShapeRegister(ros::NodeHandle nh, ros::NodeHandle nhp):ShapeCompletor(nh, nhp)
{
    ROS_WARN("ShapeRegister Constructor");
    client_shape_completor_ = nh_.serviceClient<shape_completion_bridge_msgs::RegisterShape>("shape_registration");
}

bool ShapeRegister::completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result)
{
    pc_obs_pcl_ = pc_obs_pcl;
    createClusteredShapes();


}

}