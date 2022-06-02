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

  private:
  typename pcl::PointCloud<PointT>::Ptr cloud_in_;
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

} // namespace

#endif // SHAPE_COMPLETION_BRIDGE_CLUSTERING_H