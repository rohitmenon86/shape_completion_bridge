#include "shape_completion_bridge/shape_completion_service.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "shape_completion_service");
  ros::NodeHandle n;
  shape_completion_bridge::ShapeCompletionService shape_completor;
  ROS_INFO("Ready to Complete Shapes");
  ros::spin();
  return 0;
}