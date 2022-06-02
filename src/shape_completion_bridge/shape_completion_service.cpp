#include "shape_completion_bridge/shape_completion_service.h"
#include "shape_completion_bridge_msgs/ClusteredShape.h"

namespace shape_completion
{

ShapeCompletionService::ShapeCompletionService():nhp_("~")
{
    nhp_.param("p_min_cluster_size", p_min_cluster_size_, 100);
    nhp_.param("p_max_cluster_size", p_max_cluster_size_, 10000);
    nhp_.param("p_cluster_tolerance", p_cluster_tolerance_, 0.01);
    nhp_.param("p_estimate_normals_search_radius", p_estimate_normals_search_radius_, 0.015);
    nhp_.param("p_estimate_cluster_center_regularization", p_estimate_cluster_center_regularization_, 2.5);

    pub_completed_shapes_ = nhp_.advertise<sensor_msgs::PointCloud2>("completed_shapes", 2, true);
}

// ShapeCompletionService(ros::NodeHandle& nh, ros::NodeHandle& nhp):ShapeCompletionService(), nh_(nh), nhp_(nhp)
// {

// }

bool ShapeCompletionService::processCompleteShapesServiceCallback(const shape_completion_bridge_msgs::CompleteShapes::Request& req, shape_completion_bridge_msgs::CompleteShapes::Response& res)
{
    res.result_code = -1;
    if(readPointCloudFromTopic() == false)
    {
        ROS_WARN("Returning from service call");
        return false;
    }
    callShapeCompletionMethod(req);
    

    return true;
}

bool ShapeCompletionService::readPointCloudFromTopic()
{
    sensor_msgs::PointCloud2ConstPtr pc_obs_ros  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("pc_in", ros::Duration(5));
    if (pc_obs_ros == NULL)
    {
        ROS_ERROR("No point cloud message received after 5 secs");
        return false;
    }

    pc_obs_pcl_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*pc_obs_ros, *pc_obs_pcl_);

    // Filter NaN Values
    std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*pc_obs_pcl_, *pc_obs_pcl_, *indices);

    if (pc_obs_pcl_->size() == 0)
    {
        ROS_WARN("pc_obs_pcl_->size() == 0");
        return false;
    }
    // Transform pointcloud to the world frame
    pcl_ros::transformPointCloud("world", *pc_obs_pcl_, *pc_obs_pcl_, *listener_); // todo: parametric world_frame
    pc_obs_pcl_tf_ros_header_ = pcl_conversions::fromPCL(pc_obs_pcl_->header);
    
    return true;
}

bool ShapeCompletionService::createClusteredShapes()
{
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_obs_pcl_, cluster_indices, p_cluster_tolerance_, p_min_cluster_size_, p_max_cluster_size_);
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::experimentalClustering(pc_pcl, cluster_indices);

    // TODO: CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

    // Initialize superellipsoids
    std::vector<shape_completion_bridge_msgs::ClusteredShape> clustered_shapes;
    for (const auto& current_cluster_pc : clusters)
    {
        auto new_clustered_shape = std::make_shared<clustering::ClusteredShape<pcl::PointXYZRGB>>(current_cluster_pc);
        new_clustered_shape->estimateNormals(p_estimate_normals_search_radius_); // search_radius
        new_clustered_shape->estimateClusterCenter(p_estimate_cluster_center_regularization_); // regularization
        shape_completion_bridge_msgs::ClusteredShape clustered_shape_msg;
        pcl::toROSMsg(*current_cluster_pc, clustered_shape_msg.cluster_pointcloud);
        new_clustered_shape->getEstimatedCenter(clustered_shape_msg.estimated_centre.x, clustered_shape_msg.estimated_centre.y, clustered_shape_msg.estimated_centre.z); 
        clustered_shapes.push_back(clustered_shape_msg);
    }
}

bool ShapeCompletionService::callShapeCompletionMethod(const shape_completion_bridge_msgs::CompleteShapes::Request& req)
{
    switch(req.shape_completion_method)
    {
        case shape_completion_bridge_msgs::CompleteShapesRequest::SUPERELLIPSOID:
            completeShapeUsingSuperellipsoids();
            break;
        case shape_completion_bridge_msgs::CompleteShapesRequest::SHAPE_REGISTRATION:
            completeShapeUsingShapeRegistration();
            break;
        case shape_completion_bridge_msgs::CompleteShapesRequest::SHAPE_RECONSTRUCTION:
            completeShapeUsingShapeReconstruction();
            break;
        default:
            ROS_ERROR("Invalid shape completion method");
            return false;
    }
    return true;
}
bool ShapeCompletionService::completeShapeUsingSuperellipsoids()
{
    return true;
}

bool ShapeCompletionService::completeShapeUsingShapeRegistration()
{
    return true;
}

bool ShapeCompletionService::completeShapeUsingShapeReconstruction()
{
    return true;
}

}