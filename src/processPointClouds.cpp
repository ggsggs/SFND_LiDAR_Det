// PCL lib Functions for processing point clouds

#include <random>
#include <iterator>
#include <experimental/algorithm>
#include "processPointClouds.h"

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  // DONE: Fill in the function to do voxel grid point reduction and region
  // based filtering
  pcl::VoxelGrid<PointT> vg;
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered{
      new pcl::PointCloud<PointT>()};
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*cloudFiltered);

  typename pcl::PointCloud<PointT>::Ptr cloudRegion{
      new pcl::PointCloud<PointT>()};

  pcl::CropBox<PointT> region(true);
  region.setMin(minPoint);
  region.setMax(maxPoint);
  region.setInputCloud(cloudFiltered);
  region.filter(*cloudRegion);

  std::vector<int> indices;
  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f{-1.5, -1.7, -1, 1});
  roof.setMax(Eigen::Vector4f{2.6, 1.7, -0.4, 1});
  roof.setInputCloud(cloudRegion);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices)
    inliers->indices.push_back(point);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds_PCL(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // DONE: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr planeCloud{new pcl::PointCloud<PointT>};
  typename pcl::PointCloud<PointT>::Ptr obsCloud{new pcl::PointCloud<PointT>};

  for (auto idx : inliers->indices)
    planeCloud->points.push_back(cloud->points[idx]);

  pcl::ExtractIndices<PointT> extract;

  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obsCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(obsCloud, planeCloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane_PCL(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  // DONE:: Fill in this function to find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients()};
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    std::cout << "Could not estimate plane for current CP";
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds_PCL(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
    std::unordered_set<int> inliers =
      Ransac(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(
        new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(
        new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++) {
      auto& point = cloud->points[index];
      if (inliers.count(index))
        cloudInliers->points.push_back(point);
      else
        cloudOutliers->points.push_back(point);
    } 

    return {cloudOutliers, cloudInliers};
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering_PCL(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance); // 2cm
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);

  for (auto &cluster : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster{
        new pcl::PointCloud<PointT>};
    for (auto &idx : cluster.indices) {
      cloud_cluster->points.push_back(cloud->points[idx]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = 1;

    clusters.push_back(cloud_cluster);
  }

  // TODO-END
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}

// helper fun
template <typename PointT>
std::unordered_set<int>
ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud,
                                   int maxIterations, float distanceTol) {
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