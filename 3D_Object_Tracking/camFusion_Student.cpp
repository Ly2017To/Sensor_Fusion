#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cmath>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;

//a fast way to calculate mean and variance of a vector based on a post from stackoverflow
//link: https://stackoverflow.com/questions/7616511/calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boost

double mean(std::vector<double> v)
{
	double sum = std::accumulate(std::begin(v), std::end(v), 0.0);
	return sum / v.size();
}

double stdDev(double m, std::vector<double> v)
{
	double accum = 0.0;
	std::for_each (std::begin(v), std::end(v), [&](const double d){accum += (d - m) * (d - m);});
    return std::sqrt(accum / (v.size()-1));
}


//a fast way to calcualte euclidean distance based on a post from stackoverflow
//link: https://stackoverflow.com/questions/38365900/using-opencv-norm-function-to-get-euclidean-distance-of-two-points
double euclideanDist(cv::Point2f& a, cv::Point2f& b)
{
    cv::Point2f diff = a - b;
    return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    //find all key point matches that are within the region of interest in the camera image
  	//calculate the average distance
  	
  	float aveDistance = 0; 
  
  	//if the kptMatches points is inside the bounding box ROI
  	//put it into the vector
  	for (auto it = kptMatches.begin(); it != kptMatches.end(); ++it)
    {
      	if(kptsPrev[it->queryIdx].pt.inside(boundingBox.roi)&&kptsCurr[it->trainIdx].pt.inside(boundingBox.roi))
        {
        	boundingBox.kptMatches.push_back(*it);
          	//compute the sum of euclidean distance
          	aveDistance += euclideanDist(kptsPrev[it->queryIdx].pt,kptsCurr[it->trainIdx].pt);
        }
    }
  
  	//calculate the average distance
  	if(boundingBox.kptMatches.size()>0)
    {
  		aveDistance=aveDistance/boundingBox.kptMatches.size();
    }
  	else
    {
    	return;
    }
  
  	//remove the outliers, which are defined if their distance is larger than twice of the average distance
  	for (auto it = boundingBox.kptMatches.begin(); it!= boundingBox.kptMatches.end(); )
    {
    	if(euclideanDist(kptsPrev[it->queryIdx].pt,kptsCurr[it->trainIdx].pt)>(2*aveDistance))
        {
        	boundingBox.kptMatches.erase(it);
        }
      	else
        {
        	++it;
        }
    }
  
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    //based on the solution in the previous class, to compute TTC.
  
  	// compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = euclideanDist(kpOuterCurr.pt,kpInnerCurr.pt);
            double distPrev = euclideanDist(kpOuterPrev.pt,kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }
  
  	std::sort(distRatios.begin(), distRatios.end());
  
  	size_t size = distRatios.size();
  
  	double medianDistRatio=0;

    if (size % 2 == 0)
    {
       medianDistRatio=(distRatios[size / 2 - 1] + distRatios[size / 2]) / 2;
    }
    else 
    {
       medianDistRatio=distRatios[size / 2];
    }

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medianDistRatio);
  
  	std::cout<<"TTCamera: "<<TTC<<std::endl;
}
                   
float meanDistanceLidar(std::vector<LidarPoint> &lidarPoints)
{	
    double laneWidth = 4.0; // assumed width of the ego lane
    double minDistance = 1e9;
  
    std::vector<double> distance;
  	double meanDistance, stdDistance;
  	
  	// find closest distance to Lidar points within ego lane
  	for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it)
    {
      	if(abs(it->y)<=laneWidth/2){
        	distance.push_back(it->x);
        }  
    }
  
  	//calculate the mean distance
  	meanDistance=mean(distance);
  	stdDistance=stdDev(meanDistance,distance);
  
  	//filter out the point of distance larger than twice of standard deviation with respect to mean 
  	for(auto it=distance.begin();it!=distance.end();)
    {
    	if(std::abs(*it-meanDistance)>2.0*stdDistance){
        	distance.erase(it);
        }
      	else{
      		++it;
        }
    }
  
  	//then find the mean distance value
  	for (auto it = distance.begin(); it!=distance.end(); ++it)
    {
        minDistance = minDistance > *it ? *it : minDistance;   
    }

  	return minDistance;
}

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    
    double minXPrev, minXCurr;
  	
  	minXPrev=meanDistanceLidar(lidarPointsPrev);
    minXCurr=meanDistanceLidar(lidarPointsCurr);
      
    // compute TTC from both measurements
    TTC = minXCurr /frameRate / (minXPrev-minXCurr);
  
  	std::cout << "minXPrev: "<< minXPrev << std::endl;
  	std::cout << "minXCurr: "<< minXCurr << std::endl;
  	
  	std::cout << "TTCLidar: "<< TTC << std::endl;
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    //iterate through the matches of current of previous frame keypoints
  	//flags indicates whether the matching points found or not in the bounding boxes in the previous and current frame
  	//false: not found; true: found
  	bool findbbPre, findbbCurr; 
  
  	//index variables
  	int i, j; 
  
  	//a multimap stores candidate solutions
  	std::multimap<int, int> candidateMatches; 
  
  	//std::cout << prevFrame.boundingBoxes.size() << std::endl;
  	//std::cout << currFrame.boundingBoxes.size() << std::endl;
  
  	for (auto it = matches.begin(); it != matches.end(); ++it)
    {
		findbbPre=findbbCurr=false;
      	i=j=0;
      	
      	while(i<prevFrame.boundingBoxes.size()&&j<currFrame.boundingBoxes.size())
        { 
        	if(prevFrame.keypoints[it->queryIdx].pt.inside(prevFrame.boundingBoxes[i].roi))
            {
              	findbbPre=true;
            }
          	else
            {
            	i++;
            }
          	if(currFrame.keypoints[it->trainIdx].pt.inside(currFrame.boundingBoxes[j].roi))
            {
              	findbbCurr=true;
            }
          	else
            {
            	j++;
            }
          
          	if(findbbPre&&findbbCurr)
        	{
              	//<bb index in preFrame, bb index in currFrame>
        		candidateMatches.insert(std::pair<int, int>(i,j));
              	break;
        	}
        } 	
    }  
  
  	for (j=0; j<currFrame.boundingBoxes.size(); ++j)
    {
      	int matchesArr[prevFrame.boundingBoxes.size()] = {0};
  		for(auto it = candidateMatches.begin(); it!= candidateMatches.end(); ++it)
    	{
    		//std::cout << it->first << ":" << it->second << std::endl;
          	if(j==(it->second))
            {
            	matchesArr[it->first]++;
            }
    	}
      
      	int i = std::max_element(matchesArr, matchesArr + prevFrame.boundingBoxes.size()) - matchesArr;
      
      	//<bb index in prevFrame, bb index in currFrame>
        //std::cout << i << "matches" << j << std::endl;
      	//<bb index in preFrame, bb index in currFrame>
      	bbBestMatches.insert(std::pair<int,int >(i,j));
    }
}
