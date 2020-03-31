#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "principal_curvatures_can.hpp"

#define R 2
#define r 1
#define N 10000
#define THETA_SCALE (M_PI / 2)
#define THETA_OFFSET (M_PI / 2)
#define PHI_SCALE (M_PI / 4)
#define SEARCH_RADIUS 0.1

int main(void) {
  Eigen::VectorXf theta = THETA_SCALE * Eigen::VectorXf::Random(N) + Eigen::VectorXf::Constant(N, THETA_OFFSET);
  Eigen::VectorXf phi = PHI_SCALE * Eigen::VectorXf::Random(N);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
  for(int i=0;i<N;i++) {
    pcl::PointXYZ point;
    point.x = (R + r * cos(theta(i))) * cos(phi(i));
    point.y = (R + r * cos(theta(i))) * sin(phi(i));
    point.z = r * sin(theta(i));
    cloud->points.push_back(point);
  }
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  normal_estimation.setInputCloud(cloud);
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setRadiusSearch(SEARCH_RADIUS);
  normal_estimation.setViewPoint(0, 0, std::numeric_limits<float>::infinity());
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());
  normal_estimation.compute(*normals);
  PrincipalCurvaturesEstimationCAN curvature_estimation;
  curvature_estimation.setInputCloud(cloud);
  curvature_estimation.setInputNormals(normals);
  curvature_estimation.setSearchMethod(tree);
  curvature_estimation.setRadiusSearch(SEARCH_RADIUS);
  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures>());
  curvature_estimation.compute(*curvatures);
  for(int i=0;i<N;i++) {
    pcl::PrincipalCurvatures curve_point = curvatures.get()->points[i];
    std::cout
      << theta(i) << " "
      << phi(i) << " "
      << curve_point.pc1 * curve_point.pc2 << " "
      << cos(theta(i)) / (r * (R + r * cos(theta(i))))
      << std::endl;
  }
}
