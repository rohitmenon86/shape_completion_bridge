#include "shape_completion_bridge/shape_completor.h"
namespace shape_completion_bridge
{

ShapeCompletor::ShapeCompletor(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh), nhp_(nhp)
{
    ROS_INFO("ShapeCompletor Constructor");

    nhp_.param("p_min_cluster_size", p_min_cluster_size_, 100);
    nhp_.param("p_max_cluster_size", p_max_cluster_size_, 10000);
    nhp_.param("p_cluster_tolerance", p_cluster_tolerance_, 0.01);
    nhp_.param("p_estimate_normals_search_radius", p_estimate_normals_search_radius_, 0.015);
    nhp_.param("p_estimate_cluster_center_regularization", p_estimate_cluster_center_regularization_, 2.5);
    nhp_.param("p_mean", mean_, 0.03);
    nhp_.param("p_stddev", stddev_, 0.005);
    variance_ = stddev_ * stddev_;
}

bool ShapeCompletor::createClusteredShapes()
{
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::euclideanClusterExtraction(pc_obs_pcl_, cluster_indices, p_cluster_tolerance_, p_min_cluster_size_, p_max_cluster_size_);
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters = clustering::experimentalClustering(pc_pcl, cluster_indices);
    
    if(clusters.size() < 1)
        return false;
    // TODO: CHECK IF FRUIT SIZE MAKES SENSE. OTHERWISE SPLIT OR DISCARD

    std_msgs::Header pc_obs_pcl_tf_ros_header = pcl_conversions::fromPCL(pc_obs_pcl_->header);

    // Initialize superellipsoids
    clustered_shapes_.clear();
    for (const auto& current_cluster_pc : clusters)
    {
        auto new_clustered_shape = std::make_shared<clustering::ClusteredShape<pcl::PointXYZRGB>>(current_cluster_pc);
        new_clustered_shape->estimateNormals(p_estimate_normals_search_radius_); // search_radius
        new_clustered_shape->estimateClusterCenter(p_estimate_cluster_center_regularization_); // regularization
        shape_completion_bridge_msgs::ClusteredShape clustered_shape_msg;
        pcl::toROSMsg(*current_cluster_pc, clustered_shape_msg.cluster_pointcloud);
        clustered_shape_msg.cluster_pointcloud.header = pc_obs_pcl_tf_ros_header;
        new_clustered_shape->getEstimatedCenter(clustered_shape_msg.estimated_centre.x, clustered_shape_msg.estimated_centre.y, clustered_shape_msg.estimated_centre.z); 
        clustered_shapes_.push_back(clustered_shape_msg);
    }
    ROS_INFO_STREAM("Created "<<clustered_shapes_.size()<< " shapes");
    return true;
}

PointCloudPCLwithRoiData ShapeCompletor::createCompleteShape(std::vector<ShapeCompletorResult>& shape_completor_result)
{
    PointCloudPCLwithRoiData merged_pred_pc_with_roi;
    bool merged_pc_initialised = false; 
    for(const auto& completed_shape:shape_completor_result)
    {
        if(completed_shape.valid_prediction)
        {
            ROS_DEBUG("Valid prediction");
            if(merged_pc_initialised == false)
            {
                merged_pred_pc_with_roi = processShapeCompletorResult(completed_shape);
            }
            else
            {
                ROS_DEBUG("processShapeCompletorResult Before ");
                auto pred_pc_with_roi = processShapeCompletorResult(completed_shape);
                ROS_DEBUG("processShapeCompletorResult After");
                if(pred_pc_with_roi.cloud != NULL &&  pred_pc_with_roi.cloud->points.size() > 0)
                {
                    ROS_DEBUG("merge pc");
                    *(merged_pred_pc_with_roi.cloud) +=  *(pred_pc_with_roi.cloud);
                    ROS_DEBUG("merge roi");
                    merged_pred_pc_with_roi.roi_data.insert(merged_pred_pc_with_roi.roi_data.end(), pred_pc_with_roi.roi_data.begin(), pred_pc_with_roi.roi_data.end());
                    ROS_DEBUG("done merging");
                }
            }
            if(merged_pred_pc_with_roi.cloud != NULL)
            {
                merged_pc_initialised = true;
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
    double ratio = double(result.cloud->points.size())/double(shape_completor_result.observed_pointcloud->points.size());
    ROS_INFO_STREAM("obs_cloud  size: "<<shape_completor_result.observed_pointcloud->points.size()<< "\tpred_cloud size: "<<result.cloud->points.size()<< "\t\t ratio of pred to actual: "<<ratio);
    if(result.cloud->points.size() < 50 || ratio < 0.1)
    {
        ROS_ERROR("Discarding this cloud due to few points");
        result.cloud = NULL;
        return result;
    }
    result.roi_data = calcRoiData(shape_completor_result.observed_pointcloud, result.cloud);

    return result;
}

std::vector<shape_completion_bridge_msgs::RoiData> ShapeCompletor::calcRoiData(pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pred_cloud)
{
    std::vector<shape_completion_bridge_msgs::RoiData> roi_data_vec; 
    if(pred_cloud->points.size() < 1)
    {
        ROS_WARN_STREAM(" Point Cloud Empty. Hence not calculating ROI value");
        return roi_data_vec;
    }
    roi_data_vec.reserve(pred_cloud->points.size());
    for(auto &pred_point : pred_cloud->points)
    {
        shape_completion_bridge_msgs::RoiData roi_data_pt;
        roi_data_pt.roi_probability = calcRoiProbability(calcMinimumDistance(actual_cloud, pred_point));
        roi_data_vec.push_back(roi_data_pt);
    }
    ROS_DEBUG_STREAM("roi_data_vec size: "<<roi_data_vec.size()<< "\tpred_cloud->points.size: "<<pred_cloud->points.size());
    std::vector<shape_completion_bridge_msgs::RoiData>::iterator result;
    result = std::max_element(roi_data_vec.begin(), roi_data_vec.end(), abs_compare);
    auto max_roi = *result;
    ROS_DEBUG_STREAM("Max ROI Probability value for each shape: "<<max_roi.roi_probability);

    return roi_data_vec;
}

double ShapeCompletor::calcMinimumDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr actual_cloud, pcl::PointXYZ& point)
{
    
    double min_dist = 10*stddev_ + mean_;
    for (const auto &actual_point : actual_cloud->points)
    {
        double dist = pcl::euclideanDistance (actual_point, point);
        // if(dist < mean_)
        //     return mean_;
        if(dist < min_dist)
            min_dist = dist;    
    }
    return min_dist;
}
double ShapeCompletor::calcRoiProbability(double dist)
{
    double prob = fmin(0.5+ 0.5*exp(-0.5* fabs((dist- mean_)*(dist- mean_)/variance_)), 0.75);
    //ROS_INFO_STREAM("dist: "<<dist<<"\t prob: "<<prob);
    return prob;

}

SuperellipsoidFitter::SuperellipsoidFitter(ros::NodeHandle nh, ros::NodeHandle nhp):ShapeCompletor(nh, nhp)
{
    ROS_INFO("Superellipsoid Fitter Constructor");
    client_shape_completor_ = nh_.serviceClient<shape_completion_bridge_msgs::FitSuperellipsoids>("capsicum_superellipsoid_fitter_server/fit_superellipsoids");
}

bool SuperellipsoidFitter::completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result)
{
    pc_obs_pcl_ = pc_obs_pcl;
    bool success = createClusteredShapes();
    if(!success)
    {
        ROS_WARN("Could not create clusters"); 
        return false;
    }
    shape_completion_bridge_msgs::FitSuperellipsoids srv;
    pcl::toROSMsg(*pc_obs_pcl, srv.request.observed_cloud);
    srv.request.observed_cloud.header = pc_obs_pcl_tf_ros_header_;
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
    ROS_DEBUG("fromSuperelipsoidResult2ShapeCompletorResult, Before");
    for(const auto src_element:src)
    {
        if(src_element.valid_completion)
        {
            ShapeCompletorResult res_element;
            res_element.valid_prediction = src_element.valid_completion;
            res_element.observed_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            res_element.predicted_volume_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            res_element.predicted_surface_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            res_element.predicted_missing_surface_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            res_element.predicted_cloud_normals.reset(new pcl::PointCloud<pcl::PointXYZ>);

            pcl::fromROSMsg(src_element.observed_pointcloud, *(res_element.observed_pointcloud));
            pcl::fromROSMsg(src_element.volume_pointcloud, *(res_element.predicted_volume_cloud));
            pcl::fromROSMsg(src_element.surface_pointcloud, *(res_element.predicted_surface_cloud));
            pcl::fromROSMsg(src_element.missing_surface_pointcloud, *(res_element.predicted_missing_surface_cloud));
            pcl::fromROSMsg(src_element.normals, *(res_element.predicted_cloud_normals));
            dest.push_back(res_element);
        }
    }    
    ROS_DEBUG("fromSuperelipsoidResult2ShapeCompletorResult, After");    
}


ShapeRegister::ShapeRegister(ros::NodeHandle nh, ros::NodeHandle nhp):ShapeCompletor(nh, nhp)
{
    ROS_INFO("ShapeRegister Constructor");
    client_shape_completor_ = nh_.serviceClient<shape_completion_bridge_msgs::RegisterShape>("shape_registration");
}

bool ShapeRegister::completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result)
{
    pc_obs_pcl_ = pc_obs_pcl;
    if(!createClusteredShapes())
    {
        ROS_WARN("Could not create clusters"); 
        return false;
    }
    for(size_t i = 0; i < clustered_shapes_.size(); ++i)
    {
        shape_completion_bridge_msgs::RegisterShape srv;
        srv.request.observed_shape = clustered_shapes_[i];
        if (client_shape_completor_.call(srv))
        {
            //fromShapeRegistrationResult2ShapeCompletorResult(srv.response.registered_shape, result);
            return true;
        }
        else
        {
            return false;
        }

    }


    return true;
}

}