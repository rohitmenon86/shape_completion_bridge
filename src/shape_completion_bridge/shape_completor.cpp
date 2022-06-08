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
    nhp_.param("p_mean", mean_, 0.02);
    nhp_.param("p_stddev", stddev_, 0.03);
    variance_ = stddev_ * stddev_;
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
    return true;
}

PointCloudPCLwithRoiData ShapeCompletor::createCompleteShape(std::vector<ShapeCompletorResult>& shape_completor_result)
{
    PointCloudPCLwithRoiData merged_pred_pc_with_roi;
    int counter = 0; 
    for(const auto& completed_shape:shape_completor_result)
    {
        if(completed_shape.valid_prediction)
        {
            if(counter == 0)
            {
                merged_pred_pc_with_roi = processShapeCompletorResult(completed_shape);
            }
            else
            {
                auto pred_pc_with_roi = processShapeCompletorResult(completed_shape);

                *(merged_pred_pc_with_roi.cloud) +=  *(pred_pc_with_roi.cloud);

                merged_pred_pc_with_roi.roi_data.insert(merged_pred_pc_with_roi.roi_data.end(), pred_pc_with_roi.roi_data.begin(), pred_pc_with_roi.roi_data.end());
            }
        }
    }
    return merged_pred_pc_with_roi;

}

PointCloudPCLwithRoiData ShapeCompletor::processShapeCompletorResult(const ShapeCompletorResult& shape_completor_result)
{
    PointCloudPCLwithRoiData result;
    if(shape_completor_result.predicted_missing_surface_cloud != NULL)
    {
        result.cloud = shape_completor_result.predicted_missing_surface_cloud;   
    }
    else 
    {
    }
    result.roi_data = calcRoiData(shape_completor_result.observed_pointcloud, result.cloud);

    return result;
}

std::vector<shape_completion_bridge_msgs::RoiData> ShapeCompletor::calcRoiData(pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pred_cloud)
{
    std::vector<shape_completion_bridge_msgs::RoiData> roi_data_vec; 
    roi_data_vec.reserve(pred_cloud->points.size());
    for(auto &pred_point : pred_cloud->points)
    {
        shape_completion_bridge_msgs::RoiData roi_data_pt;
        roi_data_pt.roi_probability = calcRoiProbability(calcMinimumDistance(actual_cloud, pred_point));
        roi_data_vec.push_back(roi_data_pt);
    }
    return roi_data_vec;
}

double ShapeCompletor::calcMinimumDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cloud, pcl::PointXYZ& point)
{
    double min_dist = mean_;
    double dist = 5*stddev_ + mean_;
    for (const auto &actual_point : actual_cloud->points)
    {
        dist = pcl::euclideanDistance (actual_point, point);
        if(dist < min_dist)
            return min_dist;
    }
    return dist;
}
double ShapeCompletor::calcRoiProbability(double dist)
{
    return exp(-0.5* (dist- mean_)*(dist- mean_)/variance_);
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
    for(const auto src_element:src)
    {
        ShapeCompletorResult res_element;
        pcl::fromROSMsg(src_element.observed_pointcloud, *(res_element.observed_pointcloud));
        pcl::fromROSMsg(src_element.volume_pointcloud, *(res_element.predicted_volume_cloud));
        pcl::fromROSMsg(src_element.surface_pointcloud, *(res_element.predicted_surface_cloud));
        pcl::fromROSMsg(src_element.missing_surface_pointcloud, *(res_element.predicted_missing_surface_cloud));
        pcl::fromROSMsg(src_element.normals, *(res_element.predicted_cloud_normals));
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

    return true;
}

}