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
Let us make a sensor fusion example. Lidar has better resolution than radar, and radar can measure the speed of objects directly. By combining lidar and radar, a better spatial understanding of the objects and their movements can be achieved. Sensor Fusion is also very important for self driving cars to achieve high automation level (https://www.csail.mit.edu/news/explained-levels-autonomy-self-driving-cars). Hardware is changing all over the time. Finding the best sensor solution is an open and ongoing problem. 
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
Except for the signals from the objects of interest, radar also receives noise. Clutter is a term used for unwanted echos in the context of autonomous driving. Clutter might be relections of the road or unrelevant objects. Genarally, a noise threshold is needed for radar to filter out noise signals. CFAR is a solution to keep the false alarm rate lower without losing valid targets by varing the threshold based on target surroundings. 
</p>

<p align='justify'>
The transmitting antenna transmits chirps to the moving targets. The receiving antenna receives chirps reflected from the targets. The transmitted chirps and reflected chirps are input signals to a mixer, which outputs a signal of frequency and phase that are differences of the frequency and phase of the input signals. To measure the range of target, applying FFT to the output signal to get the frequency, from which the range can be calculated along with the parameters of the chirp. The higher this frequency, the longer is the range of the target. This is called Range FFT. 
</p>

<p align='justify'>
The speed of the object is proportional to the phase change of the signal at the frequency mentioned above. To measure the speed of this object, we need to first transmit and receive multiple chirps and then apply FFT to a group of FFT values of the output signal obtained at the frequency mentioned above to extract the phase information needed. This is called Doppler FFT. Similar to the idea of exploiting phase change to estimate velocity, angle of arrial can be estimated by applying FFT across multiple chirps that are received by multiple antennas seperated in space. 
</p>

<p align='justify'>
There is a very good resource to learn about range, velocity and angle of arrival estimation, as well as systems about FMCW. [mmWave Radar Sensors] (https://www.ti.com/video/series/mmwave-training-series.html).
</p>


## Camera

<p align='justify'>
It makes sense to have multiple cameras on the self driving car because of different use cases. For lens, thinner lens are for far field view and wider lens are for near field view. There are also chips to cover different spectrums for different applications. For exmple, night time driving and high dynamic ranges. Mostly in the near fields, camera-only depth detection can be reliable. For multiple cameras, calibration with respect to a common time and place is needed to map images into a global frame.  
</p>

<p align='justify'>
To engineering a collision avoidance system, we first transform the problem from how to avoid collisions to how to calculate the time until collision occurs. For lidar measurements, it is necessary to filter out the unwanted points to calculate the time to collsion based on motion model. In real life application, unwanted points can be filtered out based on steering angle and curvature of lanes. For cameras, we can measure the time to collision without measuring distance. By using the pinhole camera model and some laws of projection to relate metric distance in the real world to the relative distance on the image plane. The above estimated distance is refined by bouding box techniques based on deep learning. Then the time to collision can be calculated based on a set of corresponding points in successive camera images.   
</p>

<p align='justify'>
The corresponding points mentioned above are called key points. Tracking key points means following unique points in one image through a sequence of images taken by the same camera. First, key points need to be located in an image. Second, key points need to be described. Third, key points can be matched based on their descriptors on the images taken by the same camera. In this case, key points could be tail light corners, license plate corners and so on. There are various key points detectors and descriptors available. Here is a link of tutorial provided by OpenCV (https://docs.opencv.org/3.4/db/d27/tutorial_py_table_of_contents_feature2d.html). Accuracy, matching quality and speed are considered when selecting detectors and descriptors in real world applications. 
</p>

<p align='justify'>
Mapping lidar measurements with objects can be achieved by the following three steps. First, map the lidar measurements on the image plane. Second, detect the objects on the image obtained by the camera. Third, based on the positions of the objects detected, cluster the lidar measurements on the image plane. YOLO is a deep learning algorithm that detects objects efficiently. Here is a very nice YOLO introduction [Introduction to the YOLO Family] (https://pyimagesearch.com/2022/04/04/introduction-to-the-yolo-family/). According to the above measurements from lidar and camera, the closest object in front of the car can be found, from whose distance the time to collision can be computed. 
</p>


## Kalman Filters

<p align='justify'>
Kalman Filter estimates the continuous state of system that gives a unimodal distribution, which is a probability distribution with one peak. In Kalman Filter, this distribution is given by Gaussian distribution, which is characterized by mean and variance. Our task is to maitain a best estimation of mean and variance of the objects that we are tracking. The variance measures the uncertainty about the distribution. We iterate measurement update and prediction by applying Bayes Rule, where the measurements and predictions are all Gaussian distributions. The measurement update step is the multiplication of the distribution of prediction and measurement. In terms of statistics, the posterior is propotional to the product of prior and the likelihood. The prediction step is also called motion update, which is characterized by the addition of mean and variance of distributions.
</p>

<p align='justify'>
The variables for Kalman Filter to estimate are states, which are observable states and hidden states. From the observations of observable states, we can estimate the hidden states. For designing Kalman Filter, we need state transfer functions and measurement functions. The matrix form of Kalman Filter is shown as follows. The Kalman Filter consists of endless loop of state prediction and measurement update steps.
</p>

**Kalman Filter Variables**  
$X$ : Estimate  
$P$ : Uncertainty Covariance  
$F$ : State Transition Matrix  
$U$ : Motion Vector  
$Q$ : Covriance Noise  
$Z$ : Measurement  
$H$ : Measurement Function  
$R$ : Measurement Noise Covariant Matrix  
$I$ : Identity Matrix 
$Y$ : Error  
$K$ : Gain  

**State Prediction**  
$X^{\prime} = F \cdot X + U$  
$P^{\prime} = F \cdot P \cdot F^T + Q$  


**Measurement Update**  
$Y = Z - H \cdot X^{\prime}$  
$S = H \cdot P^{\prime} \cdot H^T + R$  
$K = P^{\prime} \cdot H^T \cdot S^{-1}$  
$X = X^{\prime} + K \cdot Y$  
$P = (I - K \cdot H ) \cdot P^{\prime}$

<p align='justify'>
The above Kalman Filter equations are valid for linear transformations, which does not change the distributions of the states. What if we have nonlinear transformations applied to the states? *Extended Kalman Filter* can be applied by linearizing the nonlinear transformations with first order Taylor Expansion. Thus, the Jacobian Matrix of the nonlinear transformation matrix needs to be computed to use the Kalman Filter equations above.
</p>

<p align='justify'>
*Uncented Kalman Filter* is another way to process non linear transformations by approximating the distribution after a nonlinear transformation. This approximation is done in the following three steps. First, a set of sigma points with its corresponding weights is computed. Second, transform these sigma points according to the required non linear transformation. Third, compute the mean and variance of the approximation Gaussian distribution with the weights in the first step and the transformed points in the second step. The above distribution approximation corresponds to the state prediction step in Kalman Filter. In measurement update step, if the measurement function that maps the prediction state to measurement space is nonlinear, then the points calculated at state prediction step can be used as sigma points. The mean and variance to be mapped at measurement space can be calculated the same way as in the third step at the state prediction step. The mean and variance are updated based on the measurements from the sensors. The equations can be found at the paper called The Unscented Kalman Filter for Nonlinear Estimation. 
</p>