#include "shape_completion_bridge/shape_completion_service.h"
#include "shape_completion_bridge_msgs/ClusteredShape.h"

namespace shape_completion_bridge
{

ShapeCompletionService::ShapeCompletionService():nhp_("~")
{
    nhp_.param("shape_completor_type", shape_completor_type_, std::string("superellipsoids"));
    pub_completed_shapes_ = nhp_.advertise<sensor_msgs::PointCloud2>("completed_shapes", 2, true);

    p_shape_completor_ = std::make_unique<SuperellipsoidFitter>(nh_, nhp_);
}


bool ShapeCompletionService::processCompleteShapesServiceCallback(const shape_completion_bridge_msgs::CompleteShapes::Request& req, shape_completion_bridge_msgs::CompleteShapes::Response& res)
{
    res.result_code = -1;
    if(readPointCloudFromTopic() == false)
    {
        ROS_WARN("Returning from service call");
        return false;
    }
    std::vector<ShapeCompletorResult> shape_completor_result;
    p_shape_completor_ ->completeShapes(pc_obs_pcl_, "", shape_completor_result);
    auto merged_pred_pc_with_roi = p_shape_completor_->createCompleteShape(shape_completor_result);

    pcl::toROSMsg(*(merged_pred_pc_with_roi.cloud), res.full_predicted_point_cloud.point_cloud);
    res.full_predicted_point_cloud.roi_data = merged_pred_pc_with_roi.roi_data;

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



}