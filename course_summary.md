## Introduction

<p align='justify'>
This course works on integrating data from multiple sensors into a world picture.
</p> 

<p align='justify'>
Lidar, Radar and Camera are the sensors introduced. Lidar stands for light detection and ranging, which uses infrared wave to detect the distance of the objects around. Radar stands for radio dection and ranging, which uses radio waves to measure the speed of the objects based on doppler effect and generate radar maps for localization. Camera collects image data or video segments of the environment. The following table compares the differences between the three sensors from various perspectives. 
</p> 



|               | Camera        | Lidar         | Radar         |
| ------------- |:-------------:|:-------------:|:-------------:|
| Resolution    | Good          | OK            | Bad           |
| Noise         | Good          | Bad           | Bad           |
| Velocity      | Bad           | Bad           | Good          |
| All-Weather   | Bad           | Bad           | Good          |
| Size          | Good          | Bad           | Good          |


<p align='justify'>
Let us make a sensor fusion example. Lidar has better resolution than radar, and radar can measure the speed of objects directly. By combining lidar and radar, a better spatial understanding of the objects and their movements can be achieved.
</p> 

<p align='justify'>
Hardware is changing all over the time. Finding the best sensor solution is an open and ongoing problem.
</p> 

## Lidar Obstacle Detection

<p align='justify'>
To detemine the distance of an object, lidar sends out a beam of light and measures how long it takes to come back. To get the distance of each point in the field of view, lidar scans beams of laser across the field of view. A set of reflections that lidar measured is a point cloud. Each data in a point cloud is represented by (x,y,z,I), where x,y and z are Cartesian coordinates that express its spatial location and I is intensity that tells the reflective property. The Lidar cordinate system is the same as the car's local coordinate system. The x diretion points out front of the car and the other two directions can be found by the "Right Hand Rule". The position to mount the lidar depends on the required field of view in the application. The point cloud data can be collected by recording lidar data while driving. The point cloud data can be used to track object, predict behaviour of objects, classify objects and so on. In real world, the challenges of using data come from environment conditions.
</p> 

<p align='justify'>
Segmentation is a process of associating points with objects. We introduce the process of seperating obstacles and roads from point cloud data using RANSEC (random sample consensus) algorithm. It is an iteractive method that picks subset of points randomly and fits a model for each iteration. The iteration with the most inliers to the model is the best. 
</p>

<p align='justify'>
Clustering draws the boundery around points, which means to group points by how close they are to each other. KD-tree is a binary tree that organizes k-dimensional data points and its operations include insertion, deletion, search and so on. Each node is inserted by comparing between dimensions alternatively. To delete a node, we need to first locate the node and then delete this node without breaking the KD-tree property. The search we are doing here is to find all the points in a KD-tree that within a certain distance to a target point. This search process can be achieved by traversing a KD-tree and check whether each point is close enougth to the target on the path. KD-tress is also the backbone algorithm of Euclidean clustering algorithm of PCL. This algorithm is shown below.
</p> 

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

<p align='justify'>
Lidar data also needs to be downsampled or filtered. The pipeline to process the real world point clould data of this project includes filtering, segmentation and clutering.
</p> 


## Radar

<p align='justify'>
Radar is capable of detecting velocity and spatial information of objects at long range accurately. In addition, it can operate in poor weather conditions. FMCW (Frequency Modulated Continuous Wave) radar is the most commonly used for automotive applications. It is low cost and able to measure speed and location simultaneously. FMCW is a continuously transmitting signal in which its frequency changes linearly with time. It's shape is of saw-tooth. The antenna is for transmitting and receiving radar signal. RCS (radar cross section) is defined as the size and ability of target to reflect radar energy and it is closely related to how targets reflect radar signals.
</p>

<p align='justify'>
Except the signals from the objects of interest, radar also receives noise. Clutter is a term used for unwanted echos in the context of autonomous driving. Clutter might be relections of the road or unrelevant objects. Genarally, a noise threshold is needed for radar to filter out noise signals. CFAR is a solution to keep the false alarm rate lower without losing valid targets by varing the threshold based on target surroundings. 
</p>





## Camera


<p align='justify'>

</p>

<p align='justify'>

</p>



## Kalman Filters

<p align='justify'>
Kalman Filter estimates the continuous state of system that gives a unimodal distribution, which is a probability distribution with one peak. In Kalman Filter, this distribution is given by Gaussian distribution, which is characterized by mean and variance. Our task is to maitain a best estimation of mean and variance of the objects that we are tracking. The variance measures the uncertainty about the distribution. We iterate measurement update and prediction by applying Bayes Rule, where the measurements and predictions are all Gaussian distributions. The measurement update step is the multiplication of the distribution of prediction and measurement. In terms of statistics, the posterior is propotional to the product of prior and the likelihood. The prediction step is also called motion update, which is characterized by the addition of mean and variance of distributions.
</p>

<p align='justify'>
The variables for Kalman Filter to estimate are states, which are observable states and hidden states. From the observations of observable states, we can estimate the hidden states. For designing Kalman Filter, we need state transfer functions and measurement functions. The matrix form of Kalman Filter is shown as follows.
</p>


** Kalman Filter Variables **  

```math
X : Estimate  
P : Uncertainty Covariance  
F : State Transition Matrix  
U : Motion Vector  
Z : Measurement  
H : Measurement Function  
R : Measurement Noise  
I : Identity Matrix  
```
