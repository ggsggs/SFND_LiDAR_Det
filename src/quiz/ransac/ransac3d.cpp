/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
#include <vector>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <Eigen/Dense>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function

  // For max iterations
  for (int i = 0; i < maxIterations; i++) {
    std::unordered_set<int> inliers;
    // Randomly sample subset and fit line
    int idx1 = rand() % cloud->points.size();
    int idx2 = rand() % cloud->points.size();
    int idx3 = rand() % cloud->points.size();
    while (idx2 == idx1) {
      idx2 = rand() % cloud->points.size();
    }
    while (idx3 == idx1 || idx3 == idx2)
    {
      idx3 = rand() % cloud->points.size();
    }
    pcl::PointXYZ p1 = cloud->points[idx1];
    pcl::PointXYZ p2 = cloud->points[idx2];
    pcl::PointXYZ p3 = cloud->points[idx3];

    Eigen::Vector3d v1{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    Eigen::Vector3d v2{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};

    auto normalV = v1.cross(v2);
    float D = -(v1.dot(normalV));

    float denominator = normalV.norm();

    for (int idx = 0; idx < cloud->points.size(); idx++) {
      auto p = cloud->points[idx];
      Eigen::Vector3d v{p.x, p.y, p.z};
      // Measure distance between every point and fitted line
      float d = fabs(v.dot(normalV) + D) / denominator;
      // If distance is smaller than threshold count it as inlier
      // std::cout << d << std::endl;
      if (d <= distanceTol) {
        inliers.insert(idx);
      }
    }

    if (inliers.size() > inliersResult.size()) {
      inliersResult = inliers;
    }
  }
  // Return indicies of inliers from fitted line with most inliers
  return inliersResult;
}

int main(int argc, const char* argv[]){

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // TODO: Change the max iteration and distance tolerance arguments for Ransac
  // function
  std::unordered_set<int> inliers = Ransac(cloud, std::atoi(argv[1]), std::atof(argv[2]));

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
