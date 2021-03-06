/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0.1, 0.1, 0.1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0.1, 0.1, 0.1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0.1, 0.1, 0.1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  const Eigen::Vector4f minPoint{-40, -7, -3, 1};
  const Eigen::Vector4f maxPoint{40, 7, 1, 1};
  const float resolution = 0.2;

  auto filterCloud =
      pointProcessorI->FilterCloud(inputCloud, resolution, minPoint, maxPoint);

  auto pairClouds = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pointProcessorI->Clustering(pairClouds.first, 0.4, 3, 1000);

  // colors vector, evenly spread.
  std::vector<Color> colors;
  for (float r = 0; r <= 1; r += 0.5)
    for (float g = 0; g <= 1; g += 0.5)
      for (float b = 0; b <= 1; b += 0.5) {
        if (r == g && r == b && r == 0)
          continue;
        colors.push_back(Color(r, g, b));
      }

  // plot point cloud before and after filtering in gray colors.
  renderPointCloud(viewer, inputCloud, "inputCloud", Color(0.05, 0.05, 0.05));
  renderPointCloud(viewer, filterCloud, "filterCloud", Color(0.1, 0.1, 0.1));
  // plot each cluster in a differnt color + bbox
  int clusterId = 0;
  for (auto cluster : cloudClusters) {
    auto color = colors[clusterId % colors.size()];
    std::cout << "cluster size ";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     color);
    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId, color, 0.3);
    ++clusterId;
  }
  // plot plane cloud in whiteish color. 
  renderPointCloud(viewer, pairClouds.second, "planeCloud",
                   Color(0.8, 0.8, 0.8));
}


// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI =
      new ProcessPointClouds<pcl::PointXYZI>();
  // stream object to read pcd files.
  std::vector<boost::filesystem::path> stream = 
      pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce();
  }
}
