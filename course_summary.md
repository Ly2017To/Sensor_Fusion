## Introduction

This course works on integrating data from multiple sensors into a world picture. 

Lidar, Radar and Camera are the sensors introduced. Lidar stands for light detection and ranging, which uses infrared wave to detect the distance of the objects around. Radar stands for radio dection and ranging, which uses radio waves to measure the speed of the objects based on doppler effect and generate radar maps for localization. Camera collects image data or video segments of the environment. The following table compares the differences between the three sensors from various perspectives. 

|               | Camera        | Lidar         | Radar         |
| ------------- |:-------------:|:-------------:|:-------------:|
| Resolution    | Good          | OK            | Bad           |
| Noise         | Good          | Bad           | Bad           |
| Velocity      | Bad           | Bad           | Good          |
| All-Weather   | Bad           | Bad           | Good          |
| Size          | Good          | Bad           | Good          |


Let us make a sensor fusion example. Lidar has better resolution than radar, and radar can measure the speed of objects directly. By combining lidar and radar, a better spatial understanding of the objects and their movements can be achieved.

Hardware is changing all over the time. Finding the best sensor solution is an open and ongoing problem.

## Lidar Obstacle Detection

To detemine the distance of an object, lidar sends out a beam of light and measures how long it takes to come back. To get the distance of each point in the field of view, lidar scans beams of laser across the field of view. A set of reflections that lidar measured is a point cloud. Each data in a point cloud is represented by (x,y,z,I), where x,y and z are Cartesian coordinates that express its spatial location and I is intensity that tells the reflective property. The Lidar cordinate system is the same as the car's local coordinate system. The x diretion points out front of the car and the other two directions can be found by the "Right Hand Rule". The position to mount the lidar depends on the required field of view in the application. The point cloud data can be collected by recording lidar data while driving. The point cloud data can be used to track object, predict behaviour of objects, classify objects and so on. 

Segmentation is a process of associating points with objects. We introduce the process of seperating obstacles and roads from point cloud data using RANSEC (random sample consensus) algorithm. It is an iteractive method that picks subset of points randomly and fits a model for each iteration. The iteration with the most inliers to the model is the best. 

Clustering draws the boundery around points, which means to group points by how close they are to each other. KD-tree is a binary tree that organizes k-dimensional data points and its operations include insertion, deletion, search and so on. Each node is inserted by comparing between dimensions alternatively. To delete a node, we need to first locate the node and then delete this node without breaking the KD-tree property. The search we are doing here is to find all the points in a KD-tree that within a certain distance to a target point. This search process can be achieved by traversing a KD-tree and check whether each point is close enougth to the target on the path. KD-tress is also the backbone algorithm of Euclidean clustering algorithm of PCL. The process of this algorithm is shown below.

EuclideanCluster():
	list of clusters
	iterate through each point
		if point has not been processed
			create cluster
			Proximity(point, cluster)
			cluster add clusters
	return clusters

Proximity(point, cluster):
	if point has not been processed
		mark point as processed
		add point to cluster
		nearby points = kdTreeCloseEnough(point)
		iterate through each nearby point
			Proximity(nearby point, cluster)






## Radar




## Camera





## Kalman Filters