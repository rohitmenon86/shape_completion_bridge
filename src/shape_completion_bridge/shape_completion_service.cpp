#include "shape_completion_bridge/shape_completion_service.h"
#include "shape_completion_bridge_msgs/ClusteredShape.h"

namespace shape_completion_bridge
{

ShapeCompletionService::ShapeCompletionService():nhp_("~")
{
    nhp_.param("shape_completor_type", shape_completor_type_, std::string("superellipsoids"));
    nhp_.param("publish_pointclouds", publish_pointclouds_, true);
    pub_surface_ = nhp_.advertise<sensor_msgs::PointCloud2>("surface_cloud", 2, true);
    pub_volume_ = nhp_.advertise<sensor_msgs::PointCloud2>("volume_cloud", 2, true);
    pub_missing_surface_ = nhp_.advertise<sensor_msgs::PointCloud2>("missing_surface_cloud", 2, true);

    timer_publisher_ = nhp_.createTimer(ros::Duration(1.0), &ShapeCompletionService::timerCallback, this);
    service_shape_completion_ = nhp_.advertiseService("shape_completion_service", &ShapeCompletionService::processCompleteShapesServiceCallback, this);
    last_shape_completion_method_ = 0;
    p_shape_completor_ = std::make_unique<SuperellipsoidFitter>(nh_, nhp_);
}

void ShapeCompletionService::selectShapeCompletionMethod(const uint8_t& shape_completion_method)
{
    if(shape_completion_method == last_shape_completion_method_)
        return;
    ROS_INFO("Shape Completion Method changed");
    switch(shape_completion_method)
    {
        case shape_completion_bridge_msgs::CompleteShapes::Request::SUPERELLIPSOID:
            p_shape_completor_ = std::make_unique<SuperellipsoidFitter>(nh_, nhp_);
            break;
        case shape_completion_bridge_msgs::CompleteShapes::Request::SHAPE_REGISTRATION:
            p_shape_completor_ = std::make_unique<ShapeRegister>(nh_, nhp_);
            break;
        default:
            ROS_WARN("Shape Completion method selected currently not implemented or invalid. Defaulting to SuperellipsoidFitter");
            p_shape_completor_ = std::make_unique<SuperellipsoidFitter>(nh_, nhp_);
    }
    last_shape_completion_method_ = shape_completion_method;

}

bool ShapeCompletionService::processCompleteShapesServiceCallback(shape_completion_bridge_msgs::CompleteShapes::Request& req, shape_completion_bridge_msgs::CompleteShapes::Response& res)
{
    res.result_code = -1;
    if(readPointCloudFromTopic() == false)
    {
        ROS_WARN("Returning from service call");
        return false;
    }
    selectShapeCompletionMethod(req.shape_completion_method);
    std::vector<ShapeCompletorResult> shape_completor_result;
    pc_ros_missing_surf_ = NULL;
    if(p_shape_completor_ ->completeShapes(pc_obs_pcl_, "", shape_completor_result))
    {
        ROS_WARN("Before createCompleteShape");
        auto merged_pred_pc_with_roi = p_shape_completor_->createCompleteShape(shape_completor_result);
        ROS_WARN("After createCompleteShape");

        pcl::toROSMsg(*(merged_pred_pc_with_roi.cloud), res.full_predicted_point_cloud.point_cloud);
        res.full_predicted_point_cloud.point_cloud.header = pc_obs_pcl_tf_ros_header_;
        ROS_WARN("After toROSMsg");
        res.full_predicted_point_cloud.roi_data = merged_pred_pc_with_roi.roi_data;

        ROS_WARN_STREAM("Merged Predicted pointcloud size: "<<res.full_predicted_point_cloud.point_cloud.data.size());
        ROS_WARN_STREAM("Merged Predicted ROI Data size: "<<res.full_predicted_point_cloud.roi_data.size());
        std::vector<shape_completion_bridge_msgs::RoiData>::iterator result;
        result = std::max_element(res.full_predicted_point_cloud.roi_data.begin(), res.full_predicted_point_cloud.roi_data.end(), abs_compare);
        // auto max_roi = std::max(res.full_predicted_point_cloud.roi_data.begin(), res.full_predicted_point_cloud.roi_data.end(), abs_compare);
         ROS_WARN_STREAM("Max ROI Probability value: "<<(*result).roi_probability);

        if(publish_pointclouds_)
        {
            pc_ros_missing_surf_.reset(new sensor_msgs::PointCloud2());
            *pc_ros_missing_surf_ = res.full_predicted_point_cloud.point_cloud;
            pc_ros_missing_surf_->header = pc_obs_pcl_tf_ros_header_;
        }
        ROS_WARN("After publish_pointclouds_");
        res.result_code = 0;

    }
    else
    {
        ROS_WARN("Shape Completion Failure");
    }
    return true;

}

void ShapeCompletionService::timerCallback(const ros::TimerEvent& event)
{
    if(pc_ros_missing_surf_ != NULL)
        pub_missing_surface_.publish(pc_ros_missing_surf_);
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