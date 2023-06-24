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






## Radar






## Camera





## Kalman Filters