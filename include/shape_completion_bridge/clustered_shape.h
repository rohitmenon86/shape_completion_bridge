#ifndef SHAPE_COMPLETION_BRIDGE_CLUSTERED_SHAPE_H
#define SHAPE_COMPLETION_BRIDGE_CLUSTERED_SHAPE_H

#include <memory>
#include <vector>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <boost/math/special_functions/beta.hpp>

#ifndef SIGNUM
#define SIGNUM(x) ((x)>0?+1.:-1.)
#endif


template <typename PointT>
class ClusteredShape
{
public:
  ClusteredShape(typename pcl::PointCloud<PointT>::Ptr cloud_in);
  pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(float search_radius);
  void flipNormalsTowardsClusterCenter();
  pcl::PointXYZ estimateClusterCenter(float regularization);
  pcl::PointXYZ getEstimatedCenter();

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
pcl::PointCloud<pcl::Normal>::Ptr Superellipsoid<PointT>::estimateNormals(float search_radius)
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
void Superellipsoid<PointT>::flipNormalsTowardsClusterCenter()
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
pcl::PointXYZ Superellipsoid<PointT>::estimateClusterCenter(float regularization)
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

#endif //SHAPE_COMPLETION_BRIDGE_CLUSTERED_SHAPE_H