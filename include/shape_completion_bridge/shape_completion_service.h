#ifndef SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H
#define SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H

#include<ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/kdtree/io.h>
#include <shape_completion_bridge_msgs/CompleteShapes.h>

#include "shape_completion_bridge/clustering.h"
#include "shape_completion_bridge/shape_completor.h"

namespace shape_completion_bridge
{

class ShapeCompletionService
{
public:
    ShapeCompletionService();
    //ShapeCompletionService(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    bool processCompleteShapesServiceCallback(shape_completion_bridge_msgs::CompleteShapes::Request& req, shape_completion_bridge_msgs::CompleteShapes::Response& res);
    void timerCallback(const ros::TimerEvent& event);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    ros::Timer timer_publisher_;
    
    ros::Publisher pub_surface_, pub_volume_, pub_missing_surface_;
    sensor_msgs::PointCloud2Ptr pc_ros_surf_, pc_ros_vol_, pc_ros_missing_surf_;
    bool publish_pointclouds_;

    ros::ServiceServer service_shape_completion_;

    std::string p_world_frame_;
    std::unique_ptr<tf::TransformListener> listener_;
    std_msgs::Header pc_obs_pcl_tf_ros_header_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_obs_pcl_;

    std::string shape_completor_type_;
    std::unique_ptr<ShapeCompletor> p_shape_completor_;
    uint8_t last_shape_completion_method_;

    bool readPointCloudFromTopic();

    void selectShapeCompletionMethod(const uint8_t& shape_completion_method);


    template <typename PointT>
    inline typename pcl::PointCloud<PointT>::ConstPtr removeActualPointsfromPrediction(typename pcl::PointCloud<PointT>::Ptr pc_surf_pred, typename pcl::PointCloud<PointT>::Ptr pc_surf_real)
    {
        std::vector<int> indices_to_remove;
        // pcl::getApproximateIndices<pcl::PointXYZ, pcl::PointXYZRGB>(pc_surf_pred, pc_surf_real, indices_to_remove);
        
        try 
        {
            pcl::search::KdTree<PointT> tree_pred; 
            tree_pred.setInputCloud (pc_surf_pred);
            float radius = 0.015f;
            for (const auto &point : pc_surf_real->points)
            {
                PointT temp;
                temp.x = point.x;
                temp.y = point.y;
                temp.z = point.z;
                std::vector<int> point_indices;
                std::vector<float> point_distances;
                if(tree_pred.radiusSearch (temp, radius, point_indices, point_distances))
                {
                    indices_to_remove.insert(indices_to_remove.end(), point_indices.begin(), point_indices.end());
                }
            }
            std::sort( indices_to_remove.begin(), indices_to_remove.end() );
            indices_to_remove.erase( std::unique( indices_to_remove.begin(), indices_to_remove.end() ), indices_to_remove.end() );
            ROS_WARN_STREAM("No of indices: "<<indices_to_remove.size());
            pcl::IndicesConstPtr indices_ptr(new pcl::Indices(indices_to_remove));
            const auto [inlier_cloud, outlier_cloud] = clustering::separateCloudByIndices<pcl::PointXYZ>(pc_surf_pred, indices_ptr); 
            return outlier_cloud;
        }
        catch(const std::exception &e)
        {
            ROS_WARN_STREAM("removeActualPointsfromPrediction"<<e.what());
        }
    }

    
};

}
#endif //SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H