/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "kdtree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
	Lidar *lidar = new Lidar(cars,0);
  	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud=lidar->scan();
  	//renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");
 	
    // TODO:: Create point processor
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
  
    //std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlaneMyRansac(inputCloud, 200, 0.10);
	//renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
	//renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
  
  	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

	for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
	{
    	std::cout << "cluster size ";
      	pointProcessor.numPoints(cluster);
      	renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      
      	Box box = pointProcessor.BoundingBox(cluster);
		renderBox(viewer,box,clusterId);
      	++clusterId; 
	}
}

void clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	processed[indice]=true;
  	cluster.push_back(indice);
  
  	std::vector<int> nearest = tree->search(points[indice],distanceTol);
  
  	for(int id:nearest){
    	if(!processed[id]){
        	clusterHelper(id,points,cluster,processed,tree,distanceTol);
        }
    }
}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
  	
  	//create the vector for recording processed points
  	//initialized as false
   	std::vector<bool> processed(points.size(),false);
    
  	int i = 0;
  	while(i<points.size()){
    	if(processed[i]){
        	i++;
          	continue;
        }
      
      	std::vector<int> cluster;
        clusterHelper(i,points,cluster,processed,tree,distanceTol);
      
      	if(cluster.size()>=minSize&&cluster.size()<=maxSize){
      		clusters.push_back(cluster);
        }
      	i++;
    }
   
	return clusters;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
  //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
  //renderPointCloud(viewer,inputCloud,"inputCloud");
 
  //Experiment with the ? values and find what works best
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.3f , Eigen::Vector4f (-10, -10, -2, 1), 	Eigen::Vector4f ( 20, 10, 2, 1));
  //renderPointCloud(viewer,filterCloud,"filterCloud");

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneMyRansac(filterCloud, 40, 0.02);
  //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));
  
  KdTree* tree = new KdTree;
  
  //get the points from obstCloud
  
  std::vector<std::vector<float>> points;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstCloud = segmentCloud.first;
  
  for(int i=0; i<obstCloud->points.size(); i++)
  {	
    pcl::PointXYZI point = obstCloud->points[i];
    float x = point.x;
    float y = point.y;
    float z = point.z;
    
    std::vector<float> pointIn; 
   	pointIn.push_back(x);
    pointIn.push_back(y);
    pointIn.push_back(z);
    
  	points.push_back(pointIn);
  }
  
  for (int i=0; i<points.size(); i++) 
  {
    tree->insert(points[i],i); 
  }
  
  //Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  //
  std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, 2.0, 10, 100);
  //
  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  // Render clusters
  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  for(std::vector<int> cluster : clusters)
  {
  	pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  	for(int indice: cluster)
    {	
      	float x=points[indice][0];
      	float y=points[indice][1];
      	float z=points[indice][2];
      	pcl::PointXYZI point;
    	point.x = x;
    	point.y = y;
    	point.z = z;
  		clusterCloud->points.push_back(point);
    }
    struct Box box=pointProcessorI->BoundingBox(clusterCloud);
    renderBox(viewer, box, clusterId, Color(1.0,0,0), 1.0);
  	renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),colors[clusterId%3]);
  	++clusterId;
  }
  if(clusters.size()==0)
  {
    renderPointCloud(viewer,obstCloud,"data");
  }
	
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
	//cityBlock(viewer);
  
  	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  
    while (!viewer->wasStopped ())
    {
      	// Clear viewer
  		viewer->removeAllPointClouds();
  		viewer->removeAllShapes();

  		// Load pcd and run obstacle detection process
  		inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
  		cityBlock(viewer, pointProcessorI, inputCloudI);

  		streamIterator++;
  		if(streamIterator == stream.end())
    	{
    		streamIterator = stream.begin();
    	}
        viewer->spinOnce ();
    } 
}
