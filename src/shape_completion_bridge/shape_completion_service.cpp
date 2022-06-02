#include "shape_completion_bridge/shape_completion_service.h"
#include "shape_completion_bridge/clustering.h"
namespace shape_completion
{

ShapeCompletionService::ShapeCompletionService():nhp_("~")
{
    nhp_.param("p_min_cluster_size", p_min_cluster_size_, 100);
    nhp_.param("p_max_cluster_size", p_max_cluster_size_, 10000);
    nhp_.param("p_cluster_tolerance", p_cluster_tolerance_, 0.01);
    nhp_.param("p_estimate_normals_search_radius", p_estimate_normals_search_radius_, 0.015);
    nhp_.param("p_estimate_cluster_center_regularization", p_estimate_cluster_center_regularization_, 2.5);
}

// ShapeCompletionService(ros::NodeHandle& nh, ros::NodeHandle& nhp):ShapeCompletionService(), nh_(nh), nhp_(nhp)
// {

// }

bool ShapeCompletionService::processCompleteShapesServiceCallback(const shape_completion_bridge_msgs::CompleteShapes::Request& req, shape_completion_bridge_msgs::CompleteShapes::Response& res)
{
    res.result_code = -1;
    sensor_msgs::PointCloud2 pc_ros;
    if(getPointCloud(pc_ros) == false)
    {
        ROS_WARN("Returning from service call");
        return false;
    }
    callShapeCompletionMethod(req);
    

    return true;
}

bool ShapeCompletionService::getPointCloud(sensor_msgs::PointCloud2& pc_ros)
{
    boost::shared_ptr<sensor_msgs::PointCloud2 const> temp  = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("pc_in", ros::Duration(5));
    if (temp == NULL)
    {
        ROS_ERROR("No point cloud message received after 5 secs");
        return false;
    }
    pc_ros = *temp;
    return true;
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