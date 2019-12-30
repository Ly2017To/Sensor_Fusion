Required Experiment setup:

In this exercise, the matching descriptors step is using brute force as matcher type and KNN as selector type. 

Results Description:

detector_summary.xlsx: for each detector, the number of keypoints detected on each image and the time consumed for the detection.

detector_descriptor_summary.xlsx: for each combination of detector and descriptor, the average number of matched points detected and the average time of computing descriptor.

Analysis Based on Summary:

My selection criteria: more matched points in a shorter time interval. If there is tradeoff needs to be considered, shorter time interval is considered more important.

Based on the results I have obtained, I choose the following three detector and descriptor combinations (Detector/Descriptor):

FAST/BRISK; FAST/BRIEF; FAST/ORB;
