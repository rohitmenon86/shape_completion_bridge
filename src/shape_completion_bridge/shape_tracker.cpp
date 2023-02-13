#include "shape_completion_bridge/shape_tracker.h"
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace shape_completion_bridge
{

ShapeTracker::ShapeTracker()
{
    all_observed_shapes_current_.clear();
    all_observed_shapes_previous_.clear();
    active_observed_shapes_current_.clear();
    active_observed_shapes_previous_.clear();
}

bool ShapeTracker::isShapeEqual(const ObservedShape& current, const ObservedShape& previous)
{

    if ((calcPositionDiff(current.pose.position, previous.pose.position) > shape_tracker_params_.param_position_diff_threshold))
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

std::vector<ObservedShape> ShapeTracker::getChangedObservedShapes()
{
    std::vector<ObservedShape> changed_observed_shapes;
    for(size_t i = 0; i < changed_observed_shapes_indices_current_.size(); ++i)
    {
        changed_observed_shapes.push_back(all_observed_shapes_current_[changed_observed_shapes_indices_current_[i]]);
    }
}

}//end of namespace shape_completion_bridge