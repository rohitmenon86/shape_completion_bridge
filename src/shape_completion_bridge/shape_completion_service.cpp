#include "shape_completion_bridge/shape_completion_service.h"
#include "shape_completion_bridge_msgs/ClusteredShape.h"
#include <mutex>          // std::mutex
#include <superellipsoid_msgs/SuperellipsoidArray.h>

std::mutex mtx;           // mutex for critical section

namespace shape_completion_bridge
{

ShapeCompletionService::ShapeCompletionService():nhp_("~")
{
    nhp_.param("shape_completor_type", shape_completor_type_, std::string("superellipsoids"));
    nhp_.param("publish_pointclouds", publish_pointclouds_, true);
    pub_surface_ = nhp_.advertise<sensor_msgs::PointCloud2>("surface_cloud", 2, true);
    pub_volume_ = nhp_.advertise<sensor_msgs::PointCloud2>("volume_cloud", 2, true);
    pub_missing_surface_ = nhp_.advertise<sensor_msgs::PointCloud2>("missing_surface_cloud", 2, true);
    pub_superellipsoids_ = nhp_.advertise<superellipsoid_msgs::SuperellipsoidArray>("superellipsoids", 2, true);

    timer_publisher_ = nhp_.createTimer(ros::Duration(1.0), &ShapeCompletionService::timerCallback, this);
    timer_sefitter_ = nhp_.createTimer(ros::Duration(5.0), &ShapeCompletionService::timerSuperellipsoidFitterCallback, this);

    //service_shape_completion_ = nhp_.advertiseService("shape_completion_service", &ShapeCompletionService::processCompleteShapesServiceCallback, this);
    //service_shape_evaluation_ = nhp_.advertiseService("shape_evaluation_service", &ShapeCompletionService::processEvaluateShapesServiceCallback, this);
    service_get_active_predicted_shapes_ = nhp_.advertiseService("get_active_predicted_shapes", &ShapeCompletionService::processGetActiveMissingShapesServiceCallback, this);

    last_shape_completion_method_ = 0;
    p_shape_completor_ = std::make_unique<SuperellipsoidFitter>(nh_, nhp_);
    shape_completor_result_available_ = false;

    nhp_.param("p_cost_type", p_cost_type, (int)superellipsoid_volume::CostFunctionType::RADIAL_EUCLIDIAN_DISTANCE);
    nhp_.param("p_min_cluster_size", p_min_cluster_size, 100);
    nhp_.param("p_max_cluster_size", p_max_cluster_size, 10000);
    nhp_.param("p_max_num_iterations", p_max_num_iterations, 100);
    nhp_.param("p_missing_surfaces_num_samples", p_missing_surfaces_num_samples, 500);
    nhp_.param("p_missing_surfaces_threshold", p_missing_surfaces_threshold, 0.015);
    nhp_.param("p_cluster_tolerance", p_cluster_tolerance, 0.01);
    nhp_.param("p_estimate_normals_search_radius", p_estimate_normals_search_radius, 0.015);
    nhp_.param("p_estimate_cluster_center_regularization", p_estimate_cluster_center_regularization, 2.5);
    nhp_.param("p_pointcloud_volume_resolution", p_pointcloud_volume_resolution, 0.001);
    nhp_.param("p_octree_volume_resolution", p_octree_volume_resolution, 0.001);
    nhp_.param("p_prior_scaling", p_prior_scaling, 0.1);
    nhp_.param("p_prior_center", p_prior_center, 0.1);
    nhp_.param("p_print_ceres_summary", p_print_ceres_summary, false);
    nhp_.param("p_use_fibonacci_sphere_projection_sampling", p_use_fibonacci_sphere_projection_sampling, false);
    nhp_.param<std::string>("p_world_frame", p_world_frame, "world");
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

// bool ShapeCompletionService::processCompleteShapesServiceCallback(shape_completion_bridge_msgs::CompleteShapes::Request& req, shape_completion_bridge_msgs::CompleteShapes::Response& res)
// {
//     ROS_WARN("processCompleteShapesServiceCallback");
//     res.result_code = -1;
//     if(readPointCloudFromTopic() == false)
//     {
//         ROS_WARN("Returning from service call");
//         return false;
//     }
//     selectShapeCompletionMethod(req.shape_completion_method);
//     std::vector<ShapeCompletorResult> shape_completor_result;
//     pc_ros_missing_surf_ = NULL;
//     shape_completor_result_available_ = false;
//     if(p_shape_completor_ ->completeShapes(pc_obs_pcl_, "", shape_completor_result))
//     {
//         storeShapeCompletorResult(shape_completor_result);
//         ROS_WARN("Before createCompleteShape");
//         auto merged_pred_pc_with_roi = p_shape_completor_->createCompleteShape(shape_completor_result);
//         ROS_WARN("After createCompleteShape");
//         if(merged_pred_pc_with_roi.cloud->points.size() < 1)
//         {
//             ROS_WARN("Merged Predicted Missing PC has no points");
//             return true; 
//         }
//         pcl::toROSMsg(*(merged_pred_pc_with_roi.cloud), res.full_predicted_point_cloud.point_cloud);
//         res.full_predicted_point_cloud.point_cloud.header = pc_obs_pcl_tf_ros_header_;
//         res.missing_surface_cluster_centres = merged_pred_pc_with_roi.cluster_centres;
        
//         ROS_WARN("After toROSMsg");
//         res.full_predicted_point_cloud.roi_data = merged_pred_pc_with_roi.roi_data;

//         ROS_WARN_STREAM("Merged Predicted pointcloud size: "<<res.full_predicted_point_cloud.point_cloud.data.size());
//         ROS_WARN_STREAM("Merged Predicted ROI Data size: "<<res.full_predicted_point_cloud.roi_data.size());
//         std::vector<shape_completion_bridge_msgs::RoiData>::iterator result;
//         result = std::max_element(res.full_predicted_point_cloud.roi_data.begin(), res.full_predicted_point_cloud.roi_data.end(), abs_compare);
//         // auto max_roi = std::max(res.full_predicted_point_cloud.roi_data.begin(), res.full_predicted_point_cloud.roi_data.end(), abs_compare);
//          ROS_WARN_STREAM("Max ROI Probability value: "<<(*result).roi_probability);

//         if(publish_pointclouds_)
//         {
//             pc_ros_missing_surf_.reset(new sensor_msgs::PointCloud2());
//             *pc_ros_missing_surf_ = res.full_predicted_point_cloud.point_cloud;
//             pc_ros_missing_surf_->header = pc_obs_pcl_tf_ros_header_;
//         }
//         ROS_WARN("After publish_pointclouds_");
//         res.result_code = 0;

//     }
//     else
//     {
//         ROS_WARN("Shape Completion Failure");
//     }
//     return true;

// }

void ShapeCompletionService::storeShapeCompletorResult(std::vector<ShapeCompletorResult>& src)
{
    mtx.lock();
    shape_completor_result_.clear();
    shape_completor_result_ = src;
    shape_completor_result_available_ = true;
    mtx.unlock();
}

std::vector<ShapeCompletorResult> ShapeCompletionService::getShapeCompletorResult()
{
    mtx.lock();
    std::vector<ShapeCompletorResult> res = shape_completor_result_;
    //shape_completor_result_.clear();
    mtx.unlock();
    return res;
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


void ShapeCompletionService::timerSuperellipsoidFitterCallback(const ros::TimerEvent& event)
{
    if(shape_completor_result_available_ == false)
    {
        ROS_ERROR("Shape Completor Result not available yet. Hence cannot evaluate shapes");
        return; 
    }
    auto sc_result_vec = getShapeCompletorResult();
    std::vector<std::shared_ptr<superellipsoid_volume::Superellipsoid<pcl::PointXYZ>>> superellipsoids;
    superellipsoid_msgs::SuperellipsoidArray sea;
    for(const auto& sc_result: sc_result_vec)
    {
        ROS_WARN_STREAM("Before make SE, point size: "<<sc_result.predicted_surface_cloud->points.size());
        auto new_superellipsoid = std::make_shared<superellipsoid_volume::Superellipsoid<pcl::PointXYZ>>(sc_result.predicted_surface_cloud);
        ROS_WARN("Before estimateNormals");
        new_superellipsoid->estimateNormals(p_estimate_normals_search_radius); // search_radius
        ROS_WARN("Before estimateClusterCenter");
        new_superellipsoid->estimateClusterCenter(p_estimate_cluster_center_regularization); // regularization
        ROS_WARN("Before flipNormalsTowardsClusterCenter");
        new_superellipsoid->flipNormalsTowardsClusterCenter();
        superellipsoids.push_back(new_superellipsoid);
        ROS_WARN("\t***********Before fit***********************");
        bool success = new_superellipsoid->fit(p_print_ceres_summary, p_max_num_iterations, (superellipsoid_volume::CostFunctionType)(p_cost_type));

        if (success && pub_superellipsoids_.getNumSubscribers() > 0)
        {
            sea.header = pc_obs_pcl_tf_ros_header_;
            superellipsoid_msgs::Superellipsoid se_msg = new_superellipsoid->generateRosMessage();
            se_msg.header = pc_obs_pcl_tf_ros_header_;
            sea.superellipsoids.push_back(se_msg);
        }
    }
    ROS_INFO_STREAM("\t***********Cluster Fitting Publishing ***********************");
    pub_superellipsoids_.publish(sea);
}

bool ShapeCompletionService::processGetActiveMissingShapesServiceCallback(shape_completion_bridge_msgs::GetActiveMissingShapes::Request& req, shape_completion_bridge_msgs::GetActiveMissingShapes::Response& res)
{
    shape_tracker_.stopTimerGetInstancePointclouds();
    auto changed_observed_shapes = shape_tracker_.getChangedObservedShapes(); 
    for(size_t i = 0; i < changed_observed_shapes.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr observed_shape;
        pcl::fromROSMsg(changed_observed_shapes[i].pointcloud, *observed_shape);
        std::shared_ptr<superellipsoid::Superellipsoid<pcl::PointXYZRGB>> superellipsoid_ptr;
        std::vector<superellipsoid::CompletedShapeData> completed_shape_data_list;
        if(se_detector_.fitSuperellipsoid(observed_shape, superellipsoid_ptr))
        {
            superellipsoid::SuperellipsoidResult<pcl::PointNormal> result = se_detector_.computeSuperellipsoidResult<pcl::PointNormal>(superellipsoid_ptr, observed_shape);
            superellipsoid::CompletedShapeData completed_shape_data;
            //superellipsoid::fromSuperellipsoidResult2CompletedShapeData<pcl::PointNormal>(result, completed_shape_data);
            //res.active_missing_shapes.push_back(completed_shape_data.missing_shape);
        }
    }
    return true;
    
}




}