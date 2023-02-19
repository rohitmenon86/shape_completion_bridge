#include "shape_completion_bridge/shape_tracker.h"
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

std::mutex mtx_vpp;           // mutex for critical section

namespace shape_completion_bridge
{

ShapeTracker::ShapeTracker():nhp_("~")
{
    all_observed_shapes_current_.clear();
    all_observed_shapes_previous_.clear();
    active_observed_shapes_current_.clear();
    active_observed_shapes_previous_.clear();

    //timer_get_instance_pointclouds_ = nhp_.createTimer(ros::Duration(2.0), &ShapeTracker::timerGetInstancePointclouds, this);
    client_get_instance_pointclouds_ = nh_.serviceClient<vpp_msgs::GetListInstancePointclouds>("/gsm_node/get_list_instance_pointclouds");
    pub_integrated_cloud_ = nhp_.advertise<sensor_msgs::PointCloud2>("integrated_cloud", 2, true);
    sub_list_instance_clouds_ = nh_.subscribe("/gsm_node/list_instance_pointclouds", 1, &ShapeTracker::callbackInstancePointclouds, this);

}

bool ShapeTracker::isShapeEqual(const ObservedShapeInstance& current, const ObservedShapeInstance& previous)
{

    if ((calcPositionDiff(current.centroid, previous.centroid) > shape_tracker_params_.param_position_diff_threshold))
        return false;

    if(fabs(current.num_points - previous.num_points) > shape_tracker_params_.param_size_diff_threshold)
        return false;
    
    return true;
}

std::vector<size_t> ShapeTracker::getChangedObservedShapesIndices()
{
    changed_observed_shapes_indices_current_.clear();
    std::vector<size_t> equal_indices_in_previous;
    for(size_t i = 0; i < all_observed_shapes_current_.size(); ++i)
    {
        bool match_not_found = true;
        for(size_t j = 0; j < all_observed_shapes_previous_.size(); ++j)
        {
            if(std::any_of(equal_indices_in_previous.cbegin(), equal_indices_in_previous.cend(), compare(j)))
                continue;
            if(isShapeEqual(all_observed_shapes_current_[i], all_observed_shapes_previous_[j]))
            {
                match_not_found = false;
                equal_indices_in_previous.push_back(j);
            }
        }
        if(match_not_found)
        {
            changed_observed_shapes_indices_current_.push_back(i);
        }
    }
    return changed_observed_shapes_indices_current_;
}

std::vector<ObservedShapeInstance> ShapeTracker::getChangedObservedShapes()
{
    std::vector<ObservedShapeInstance> changed_observed_shapes;
    for(size_t i = 0; i < changed_observed_shapes_indices_current_.size(); ++i)
    {
        changed_observed_shapes.push_back(all_observed_shapes_current_[changed_observed_shapes_indices_current_[i]]);
    }
    return changed_observed_shapes;
}

void ShapeTracker::timerGetInstancePointclouds(const ros::TimerEvent& event)
{
    ROS_INFO("timerGetInstancePointclouds");
    vpp_msgs::GetListInstancePointclouds srv;
    srv.request.get_specific_instances = false;
    if(client_get_instance_pointclouds_.call(srv))
    {
        ROS_INFO("Get Instance PC call successful");
        setCurrentObservedShapes(srv.response.instance_clouds_with_centroid);
        pub_integrated_cloud_.publish(srv.response.integrated_cloud);
        return;
    }
    ROS_INFO("Get Instance PC call failure");
}

void ShapeTracker::callbackInstancePointclouds(const vpp_msgs::InstancePointcloudwithCentroidArray& list_instance_clouds)
{
    setCurrentObservedShapes(list_instance_clouds.instance_clouds_with_centroid);
}

void ShapeTracker::setCurrentObservedShapes(const std::vector<ObservedShapeInstance>& all_observed_shapes)
{
    mtx_vpp.lock();
    all_observed_shapes_previous_.clear();
    all_observed_shapes_previous_.assign(all_observed_shapes_current_.begin(),all_observed_shapes_current_.end());
    all_observed_shapes_current_.clear();
    all_observed_shapes_current_.assign(all_observed_shapes.begin(),all_observed_shapes.end());
    observed_shapes_available_ = true;
    mtx_vpp.unlock();
}

std::vector<ObservedShapeInstance> ShapeTracker::getObservedShapes()
{
    mtx_vpp.lock();
    std::vector<ObservedShapeInstance> res = all_observed_shapes_current_;
    //shape_completor_result_.clear();
    mtx_vpp.unlock();
    return res;
}

}//end of namespace shape_completion_bridge