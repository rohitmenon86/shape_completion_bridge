#include "shape_completion_bridge/shape_completor.h"
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
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
    nhp_.param("p_roi_prob_min", roi_prob_min_, 0.5);
    nhp_.param("p_roi_prob_max", roi_prob_max_, 0.75);
    variance_ = stddev_ * stddev_;
}

bool ShapeCompletor::createClusteredShapes(bool shift_origin)
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
        if(current_cluster_pc->points.size() > 800)
        {
            ROS_WARN("Fruit cluster almost complete. Hence discarding");
            continue;
        }
        auto new_clustered_shape = std::make_shared<clustering::ClusteredShape<pcl::PointXYZRGB>>(current_cluster_pc);
        new_clustered_shape->estimateNormals(p_estimate_normals_search_radius_); // search_radius
        new_clustered_shape->estimateClusterCenter(p_estimate_cluster_center_regularization_); // regularization
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_final = current_cluster_pc;
        
        if(shift_origin_)
            cloud_final = new_clustered_shape->shiftCloudOriginToEstimatedCentre(); // for shape_registration

        if(activate_downsampling_)
        {
            size_t cloud_size = cloud_final->width * cloud_final->height; 
            if(cloud_size > 200)
            {
                float voxel_size = 0.005f;
                if(cloud_size > 400)
                    voxel_size = 0.01f;
                // Create the filtering object
                ROS_INFO_STREAM("Cloud Size before downsampling: "<<cloud_final->width * cloud_final->height );
                pcl::VoxelGrid<pcl::PointXYZRGB> vg;
                vg.setInputCloud (cloud_final);
                vg.setLeafSize (voxel_size, voxel_size, voxel_size);
                vg.filter (*cloud_final);
                ROS_INFO_STREAM("Cloud Size after downsampling: "<<cloud_final->width * cloud_final->height );
            }
        }    

        shape_completion_bridge_msgs::ClusteredShape clustered_shape_msg;
        pcl::toROSMsg(*cloud_final, clustered_shape_msg.cluster_pointcloud);
        clustered_shape_msg.cluster_pointcloud.header = pc_obs_pcl_tf_ros_header;
        new_clustered_shape->getEstimatedCenter(clustered_shape_msg.cluster_centre.x, clustered_shape_msg.cluster_centre.y, clustered_shape_msg.cluster_centre.z); 
        clustered_shapes_.push_back(clustered_shape_msg);
    }
    ROS_INFO_STREAM("Created "<<clustered_shapes_.size()<< " shapes");
    return true;
}

PointCloudPCLwithRoiData ShapeCompletor::createCompleteShape(std::vector<ShapeCompletorResult>& shape_completor_result)
{
    ROS_INFO_STREAM("xxxxxxxxxxxxxxxx Shape Completion Type: "<<shape_completion_type_);
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
    auto cluster_centres = calcReclusteredSimpleClusterMean(merged_pred_pc_with_roi);
    merged_pred_pc_with_roi.cluster_centres.clear();
    merged_pred_pc_with_roi.cluster_centres.insert(merged_pred_pc_with_roi.cluster_centres.begin(), cluster_centres.begin(), cluster_centres.end());
    return merged_pred_pc_with_roi;

}

PointCloudPCLwithRoiData ShapeCompletor::processShapeCompletorResult(const ShapeCompletorResult& shape_completor_result)
{
    if(shape_completor_result.predicted_missing_surface_cloud != NULL)
    {
        ROS_DEBUG("1");
        return processMissingPredictionCloud(shape_completor_result.predicted_missing_surface_cloud, shape_completor_result.observed_pointcloud);
        ROS_DEBUG("2");
    }
    else if(shape_completor_result.predicted_surface_cloud != NULL)
    {
        ROS_DEBUG("3");
        return processFullPredictionCloud(shape_completor_result.predicted_surface_cloud, shape_completor_result.observed_pointcloud);
        ROS_DEBUG("4");
    }
    else if(shape_completor_result.predicted_volume_cloud != NULL)
    {
        ROS_DEBUG("5");
        if(shape_completor_result.observed_pointcloud != NULL)
            ROS_DEBUG("5.1");
        return processFullPredictionCloud(shape_completor_result.predicted_volume_cloud, shape_completor_result.observed_pointcloud);
        ROS_DEBUG("6");
    }
    else if(shape_completor_result.predicted_cloud_normals != NULL)
    {
    }
}

PointCloudPCLwithRoiData ShapeCompletor::processMissingPredictionCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_missing_surface_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr observed_pointcloud)
{
    PointCloudPCLwithRoiData result;
    result.cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    result.cloud = predicted_missing_surface_cloud;   
    double ratio = double(result.cloud->points.size())/double(observed_pointcloud->points.size());
    ROS_DEBUG_STREAM("obs_cloud  size: "<<observed_pointcloud->points.size()<< "\tpred_cloud size: "<<result.cloud->points.size()<< "\t\t ratio of pred to actual: "<<ratio);
    if((getShapeCompletionType() == 0 && (result.cloud->points.size() < 10 || ratio < 0.1)) || (getShapeCompletionType() == 1 && result.cloud->points.size() < 10))
    {
        ROS_WARN("Discarding this cloud due to few points");
        result.cloud = NULL;
        return result;
    }
    result.roi_data = calcRoiData(observed_pointcloud, result.cloud);
    return result;
}

PointCloudPCLwithRoiData ShapeCompletor::processFullPredictionCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr volume_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr observed_cloud)
{
    auto missing_cloud = clustering::removeActualPointsfromPrediction<pcl::PointXYZ>(volume_cloud, observed_cloud);
    return processMissingPredictionCloud(missing_cloud, observed_cloud);
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
    double prob = fmin(roi_prob_min_ + 0.5*exp(-0.5* fabs((dist- mean_)*(dist- mean_)/variance_)), roi_prob_max_);
    //ROS_INFO_STREAM("dist: "<<dist<<"\t prob: "<<prob);
    return prob;

}

geometry_msgs::Point ShapeCompletor::calcIndividualSimpleClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_with_roi.cloud, centroid);

    geometry_msgs::Point cluster_centre; 
    cluster_centre.x = centroid(0);
    cluster_centre.y = centroid(1);
    cluster_centre.z = centroid(2);

    return cluster_centre;
}

geometry_msgs::Point ShapeCompletor::calcIndividualRoiProbWeightedClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi)
{
    geometry_msgs::Point cluster_centre; 
    return cluster_centre;
}

std::vector<geometry_msgs::Point> ShapeCompletor::calcReclusteredSimpleClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi)
{
    std::vector<pcl::PointIndices> cluster_indices_out;
    float cluster_tolerance = 0.02;
    int min_cluster_size = 11; 
    int max_cluster_size = 800;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  missing_surface_reclusters = clustering::euclideanClusterExtraction(cloud_with_roi.cloud, cluster_indices_out, cluster_tolerance, min_cluster_size, max_cluster_size);

    std::vector<geometry_msgs::Point> cluster_centres;
    for(const auto& cluster: missing_surface_reclusters)
    {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        geometry_msgs::Point cluster_centre; 
        cluster_centre.x = centroid(0);
        cluster_centre.y = centroid(1);
        cluster_centre.z = centroid(2);
        cluster_centres.push_back(cluster_centre);
    }
    return cluster_centres;

}

std::vector<geometry_msgs::Point> ShapeCompletor::calcReclusteredRoiProbWeightedClusterMean(const PointCloudPCLwithRoiData& cloud_with_roi)
{
    std::vector<pcl::PointIndices> cluster_indices_out;
    float cluster_tolerance = 0.02;
    int min_cluster_size = 10; 
    int max_cluster_size = 800;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>  missing_surface_reclusters = clustering::euclideanClusterExtraction(cloud_with_roi.cloud, cluster_indices_out, cluster_tolerance, min_cluster_size, max_cluster_size);


    std::vector<geometry_msgs::Point> cluster_centres;
    return cluster_centres;
}


SuperellipsoidFitter::SuperellipsoidFitter(ros::NodeHandle nh, ros::NodeHandle nhp):ShapeCompletor(nh, nhp)
{
    ROS_INFO("Superellipsoid Fitter Constructor");
    shape_completion_type_ = 0; 
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

            res_element.cluster_centre = src_element.optimised_centre;

            dest.push_back(res_element);
        }
    }    
    ROS_DEBUG("fromSuperelipsoidResult2ShapeCompletorResult, After");    
}


ShapeRegister::ShapeRegister(ros::NodeHandle nh, ros::NodeHandle nhp):ShapeCompletor(nh, nhp)
{
    ROS_INFO("ShapeRegister Constructor");
    shape_completion_type_ = 1; 
    shift_origin_ = true;
    activate_downsampling_ = true; 
    client_shape_completor_ = nh_.serviceClient<shape_completion_bridge_msgs::RegisterShape>("/register_shape");
}

bool ShapeRegister::completeShapes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl, std::string roi_name, std::vector<ShapeCompletorResult> & result)
{
    bool success = false;
    pc_obs_pcl_ = pc_obs_pcl;
    if(!createClusteredShapes(true))
    {
        ROS_WARN("Could not create clusters"); 
        return false;
    }
    for(size_t i = 0; i < clustered_shapes_.size(); ++i)
    {
        try
        {
            shape_completion_bridge_msgs::RegisterShape srv;
            srv.request.observed_shape = clustered_shapes_[i];
            if (client_shape_completor_.call(srv))
            {
                ROS_INFO("Shape Registration Service Call succeeded ");
                if(srv.response.result_code > -1)
                {
                    ShapeCompletorResult res_element;
                    fromShapeRegistrationResult2ShapeCompletorResult(srv.response.registered_shape, res_element);
                    result.push_back(res_element);
                    success = true;
                }
                else
                {
                    ROS_ERROR_STREAM("Shape Registration Failed: "<<srv.response.result_text);
                }
            }
            else
            {
                ROS_ERROR("Shape Registration Service Call failed ");
            }
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
    }
    return success;
}

void ShapeRegister::fromShapeRegistrationResult2ShapeCompletorResult(const shape_completion_bridge_msgs::ShapeRegistrationResult& src, ShapeCompletorResult& dest)
{
    ROS_DEBUG("fromShapeRegistrationResult2ShapeCompletorResult, Before");
    dest.valid_prediction = true;
    dest.predicted_volume_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    dest.observed_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(src.predicted_pointcloud, *(dest.predicted_volume_cloud));
    pcl::fromROSMsg(src.observed_pointcloud, *(dest.observed_pointcloud));
    auto x = src.cluster_centre.x;
    auto y = src.cluster_centre.y;
    auto z = src.cluster_centre.z;
    clustering::shiftCloudCentroidToDesiredOrigin<pcl::PointXYZ>(dest.observed_pointcloud, x, y, z);
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.transform = src.rigid_local_transform;
    Eigen::Affine3f eigen_transform, local_eigen_transform = tf2::transformToEigen(tf_stamped).cast<float>();
    Eigen::Vector3f orignal_centroid;
    orignal_centroid(0) = src.cluster_centre.x;
    orignal_centroid(1) = src.cluster_centre.y;
    orignal_centroid(2) = src.cluster_centre.z;
    eigen_transform = Eigen::Affine3f::Identity();
    eigen_transform.pretranslate(orignal_centroid);
    eigen_transform.pretranslate(local_eigen_transform.translation());
    //eigen_transform.rotate(local_eigen_transform.rotation());
    ROS_DEBUG_STREAM("Original cluster centre: "<<orignal_centroid.transpose()<<"\t Desired local transform: "<<src.rigid_local_transform<<"\t New transform: "<<eigen_transform.translation());

    dest.cluster_centre.x = eigen_transform.translation().x();
    dest.cluster_centre.y = eigen_transform.translation().y();
    dest.cluster_centre.z = eigen_transform.translation().z();


    pcl::transformPointCloud (*(dest.predicted_volume_cloud), *(dest.predicted_volume_cloud), local_eigen_transform.inverse());
    pcl::transformPointCloud (*(dest.predicted_volume_cloud), *(dest.predicted_volume_cloud), eigen_transform);
    //clustering::shiftCloudCentroidToDesiredOrigin<pcl::PointXYZ>(dest.predicted_volume_cloud, x, y, z);
    ROS_DEBUG("fromShapeRegistrationResult2ShapeCompletorResult, After");
}


} //end of namespace