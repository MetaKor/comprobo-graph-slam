# Project Story 1

## The graphSLAM optimization technique zoo

At its core, graphSLAM offers a fairly general worldview for thinking about approaching the SLAM problem.


## The Struggle of Efficient Object Detection and Relative Position Tracking

I am responsible for managing the detection, tracking, and relative position calculations between our neato and known objects. In our case we are planning on using monocolor balls of varying diameter for the Neato to use as landmarks. To tackle this problem, I am currently taking the approach of developing two classes. The first class, named 'objectdetect', allows the user to quickly add a new object to use as a landmark to track. This class takes an image of the object and then calculates the primary color, corners, descriptors, and other helpful factors to be stored as attributes. With these attributes, the second class called 'livetrack' will load in any objects to track, constantly detect them in the live video feed, and then use the known diameter of the object and relative size in frame to calculate the angle and distance to the Neato.
