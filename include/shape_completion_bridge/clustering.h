#ifndef SHAPE_COMPLETION_BRIDGE_CLUSTERING_H
#define SHAPE_COMPLETION_BRIDGE_CLUSTERING_H

#include <vector>
#include <memory>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/extract_indices.h>
#include<iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <boost/math/special_functions/beta.hpp>

#ifndef SIGNUM
#define SIGNUM(x) ((x)>0?+1.:-1.)
#endif

namespace clustering
{

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> 
euclideanClusterExtraction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out, float cluster_tolerance, int min_cluster_size, int max_cluster_size);

/*
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>
experimentalClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_out);

bool
experimentalClusteringCondition(const pcl::PointXYZRGBNormal& point_a, const pcl::PointXYZRGBNormal& point_b, float squared_distance);
*/

pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
getColoredCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, std::vector<pcl::PointIndices> &cluster_indices_in);

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> separateCloudByIndices(const typename pcl::PointCloud<PointT>::ConstPtr &input_cloud, const pcl::IndicesConstPtr &indices)
{
typename pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>), outlier_cloud(new pcl::PointCloud<PointT>);
inlier_cloud->header = input_cloud->header;
outlier_cloud->header = input_cloud->header;
pcl::ExtractIndices<PointT> extract;
extract.setInputCloud(input_cloud);
extract.setIndices(indices);
extract.setNegative(false);
extract.filter(*inlier_cloud);
extract.setNegative(true);
extract.filter(*outlier_cloud);
return std::make_pair(inlier_cloud, outlier_cloud);
}

template <typename PointT>
class ClusteredShape
{
public:
  ClusteredShape(typename pcl::PointCloud<PointT>::Ptr cloud_in);
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(float search_radius);
  void flipNormalsTowardsClusterCenter();
  pcl::PointXYZ estimateClusterCenter(float regularization);
  pcl::PointXYZ getEstimatedCenter();
  void getEstimatedCenter(double& x, double& y, double& z);
  typename pcl::PointCloud<PointT>::Ptr shiftCloudOriginToEstimatedCentre();

  private:
  typename pcl::PointCloud<PointT>::Ptr cloud_in_;
  typename pcl::PointCloud<PointT>::Ptr cloud_shifted_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_in_;
  pcl::PointXYZ estimated_center_;
  std::shared_ptr<std::vector<double>> parameters_ptr_;
};

template <typename PointT>
ClusteredShape<PointT>::ClusteredShape(typename pcl::PointCloud<PointT>::Ptr cloud_in)
{
  cloud_in_ = cloud_in;
  normals_in_ = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
  parameters_ptr_ = std::make_shared<std::vector<double>>();
}

template <typename PointT>
pcl::PointCloud<pcl::Normal>::Ptr ClusteredShape<PointT>::estimateNormals(float search_radius)
{
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_in_);
  ne.setRadiusSearch (search_radius); // Use all neighbors in a sphere of radius 3cm (0.03f) might be a good value
  normals_in_->points.reserve(cloud_in_->points.size());
  ne.compute (*normals_in_);
  return normals_in_;
}


template <typename PointT>
void ClusteredShape<PointT>::flipNormalsTowardsClusterCenter()
{
  for (size_t i = 0; i < normals_in_->size(); i++)
  {
    pcl::flipNormalTowardsViewpoint(cloud_in_->points.at(i),
                                    estimated_center_.x,
                                    estimated_center_.y,
                                    estimated_center_.z,
                                    normals_in_->at(i).normal_x,
                                    normals_in_->at(i).normal_y,
                                    normals_in_->at(i).normal_z);
  }
}


// Source: https://silo.tips/download/least-squares-intersection-of-lines
template <typename PointT>
pcl::PointXYZ ClusteredShape<PointT>::estimateClusterCenter(float regularization)
{
  // TODO: check cloud_in and normals_in have the same size

  Eigen::MatrixXf R_mat(3,3);
  R_mat.setZero();
  Eigen::MatrixXf Q_mat(3,1);
  Q_mat.setZero();
  auto eye3 = Eigen::MatrixXf::Identity(3, 3);
  Eigen::MatrixXf cluster_mean(3,1);
  cluster_mean.setZero();

  for (size_t i=0; i<normals_in_->size(); i++)
  {
    auto normal = normals_in_->at(i);
    auto point = cloud_in_->at(i);

    Eigen::MatrixXf v(3,1);
    v << normal.getNormalVector3fMap().x(), normal.getNormalVector3fMap().y(), normal.getNormalVector3fMap().z();

    Eigen::MatrixXf a(3,1);
    a << point.getVector3fMap().x(), point.getVector3fMap().y(), point.getVector3fMap().z();

    R_mat += (eye3 - v * v.transpose());
    Q_mat += ((eye3 - (v * v.transpose() ) ) * a);

    cluster_mean += a;
  }

  cluster_mean = cluster_mean / normals_in_->size();

  // regularization: bias towards a point (a value of 2.5 might be a good)
  R_mat += regularization * eye3;
  Q_mat += regularization * cluster_mean;

  auto R_dec = R_mat.completeOrthogonalDecomposition();
  auto R_inv = R_dec.pseudoInverse();
  auto cp_ = (R_inv * Q_mat).transpose();

  estimated_center_ = pcl::PointXYZ(cp_(0,0), cp_(0,1), cp_(0,2));

  if (std::isnan(estimated_center_.x) || std::isnan(estimated_center_.y) || std::isnan(estimated_center_.z))
  {
    estimated_center_ = pcl::PointXYZ(cluster_mean(0,0), cluster_mean(1,0), cluster_mean(2,0));
    fprintf(stderr, "Center prediction with normals failed. Using cluster mean instead...\n");
  }
  return estimated_center_;
}

template <typename PointT>
pcl::PointXYZ ClusteredShape<PointT>::getEstimatedCenter()
{
  return estimated_center_;
}

template <typename PointT>
void ClusteredShape<PointT>::getEstimatedCenter(double& x, double& y, double& z)
{
  x = estimated_center_.x;
  y = estimated_center_.y;
  z = estimated_center_.z;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ClusteredShape<PointT>::shiftCloudOriginToEstimatedCentre()
{
  Eigen::Vector4f cluster_centroid, centroid;
  cluster_centroid(0) = estimated_center_.x;
  cluster_centroid(1) = estimated_center_.y;
  cluster_centroid(2) = estimated_center_.z;
  cluster_centroid(3) = 1.0;
  pcl::compute3DCentroid(*cloud_in_, centroid);
  //std::cout << "Centroid of Original PointCloud Using compute3DCentroid: "<<centroid.transpose()<<std::endl;
  //std::cout << "+++++++++Centroid of Original PointCloud Using Our Method: "<<cluster_centroid.transpose()<<std::endl;
  Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();
  transform_mat.col(3) = -1.0*centroid;
  cloud_shifted_.reset(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud (*cloud_in_, *cloud_shifted_, transform_mat);  
  pcl::compute3DCentroid(*cloud_shifted_, centroid);
  //std::cout << "Centroid of Shifted PointCloud Using compute3DCentroid: "<<centroid.transpose()<<std::endl;
  return cloud_shifted_;
}

template <typename PointT>
inline void shiftCloudCentroidToDesiredOrigin(typename pcl::PointCloud<PointT>::Ptr cloud, double& x, double& y, double& z)
{
  Eigen::Vector4f centroid, desired_origin;
  pcl::compute3DCentroid(*cloud, centroid);
  //std::cerr << "Centroid of Original PointCloud Using compute3DCentroid: "<<centroid.transpose()<<std::endl;
  desired_origin(0) = x;
  desired_origin(1) = y;
  desired_origin(2) = z;
  desired_origin(3) = 1.0;
  //std::cerr << "Desired origin: "<<desired_origin.transpose()<<std::endl;
  Eigen::Matrix4f transform_mat = Eigen::Matrix4f::Identity();
  transform_mat.col(3) = desired_origin;
  pcl::transformPointCloud (*cloud, *cloud, transform_mat);  
  pcl::compute3DCentroid(*cloud, centroid);
  //std::cerr << "Centroid of Shifted PointCloud Using compute3DCentroid: "<<centroid.transpose()<<std::endl;
}

template <typename PointT>
inline typename pcl::PointCloud<PointT>::Ptr removeActualPointsfromPrediction(typename pcl::PointCloud<PointT>::Ptr pc_surf_pred, typename pcl::PointCloud<PointT>::Ptr pc_surf_real)
{
    std::vector<int> indices_to_remove;
    // pcl::getApproximateIndices<pcl::PointXYZ, pcl::PointXYZRGB>(pc_surf_pred, pc_surf_real, indices_to_remove);
    std::cout<<"Before remove points";
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
        std::cout<<"No of indices: "<<indices_to_remove.size();
        pcl::IndicesConstPtr indices_ptr(new pcl::Indices(indices_to_remove));
        const auto [inlier_cloud, outlier_cloud] = clustering::separateCloudByIndices<pcl::PointXYZ>(pc_surf_pred, indices_ptr); 
        return outlier_cloud;
    }
    catch(const std::exception &e)
    {
        std::cerr<<"removeActualPointsfromPrediction"<<e.what();
        return NULL;
    }
}

} // namespace

#endif // SHAPE_COMPLETION_BRIDGE_CLUSTERING_H