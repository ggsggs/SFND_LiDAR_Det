/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
#include <vector>
#include <random>
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

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZI> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/data_1/0000000000.pcd");
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  using PointT = pcl::PointXYZI;
  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliersResult;

  // random config
  const size_t min = 1;
  const size_t max = cloud->points.size();
  std::random_device rd;  
  std::mt19937 gen(rd()); 

  // vector of indices {0, 1, ..., size -1}
  std::vector<int> indices(cloud->points.size() - 1);
  std::iota(indices.begin(), indices.end(), 0);

  // variables to store current best results
  Eigen::Vector3d best_normalV;
  float best_D;
  int best_count = 0;
  // For max iterations
  for (int i = 0; i < maxIterations; i++) {
    std::shuffle(indices.begin(), indices.end(), gen);

    // use first 3 points in shuffled vector to create the plane.
    std::vector<Eigen::Vector3d> planePoints;
    for (size_t i = 0; i < 3; i++){
      auto p = cloud->points[indices[i]];
      planePoints.push_back({p.x, p.y, p.z});
    } 
    // direction vectors that define plane, normal vector to it, distnce to origin.
    Eigen::Vector3d v1 = planePoints.at(1) - planePoints.at(0);//{p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    Eigen::Vector3d v2 = planePoints.at(2) - planePoints.at(0);//{p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};
    auto normalV = v1.cross(v2);
    float D = -(planePoints.at(0).dot(normalV));
    float denominator = normalV.norm();

    uint count = 0;   // count of number of points that fit the plane.
    uint maxIters = indices.size()*30/100+1;
    for (int idx : indices) {
      auto p = cloud->points[idx];
      Eigen::Vector3d v{p.x, p.y, p.z};
      // Measure distance between every point and fitted line
      float d = fabs(v.dot(normalV) + D) / denominator;
      // If distance is smaller than threshold count it as inlier
      maxIters--;
      if (d <= distanceTol) count++;
      if (maxIters == 0 || maxIters + count < best_count) break;
    }

    if (count > best_count) { // update best values if candidate is superior.
      best_normalV = normalV;
      best_D = D;
      best_count = count;
    }
  }

  // generate output container once best plane candidate is known.
  for (int idx = 0; idx < cloud->points.size(); idx++) {
    auto p = cloud->points[idx];
    Eigen::Vector3d v{p.x, p.y, p.z};
    // Measure distance between every point and fitted line
    float d = fabs(v.dot(best_normalV) + best_D) / best_normalV.norm();
    // If distance is smaller than threshold count it as inlier
    if (d <= distanceTol) {
      inliersResult.insert(idx);
    }
  }
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;
  return inliersResult;
}

int main(int argc, const char *argv[]) {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = CreateData3D();

  const Eigen::Vector4f minPoint{-40, -7, -3, 1};
  const Eigen::Vector4f maxPoint{40, 7, 1, 1};
  const float resolution = 0.2;

  ProcessPointClouds<pcl::PointXYZI>* pointProcessor =
      new ProcessPointClouds<pcl::PointXYZI>();
  auto filterCloud =
      pointProcessor->FilterCloud(cloud, resolution, minPoint, maxPoint);

  // TODO: Change the max iteration and distance tolerance arguments for Ransac
  // function
  std::unordered_set<int> inliers =
      Ransac(filterCloud, std::atoi(argv[1]), std::atof(argv[2]));

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZI>());

  for (int index = 0; index < filterCloud->points.size(); index++) {
    pcl::PointXYZI point = filterCloud->points[index];
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
