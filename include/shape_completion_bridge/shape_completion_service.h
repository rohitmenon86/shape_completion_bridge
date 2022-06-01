#ifndef SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H
#define SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H

#include<ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <pcl/filters/extract_indices.h>

namespace shape_completion
{
class ShapeCompletionService
{

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Subscriber sub_pointcloud_;
    ros::ServiceServer service_se_detector_;
    std::unique_ptr<tf::TransformListener> listener_;
    std_msgs::Header pc_pcl_tf_ros_header_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_pcl_;

};

}
#endif //SHAPE_COMPLETION_BRIDGE_SHAPE_COMPLETION_SERVICE_H