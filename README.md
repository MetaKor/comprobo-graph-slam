

<div align="center">
  <h1> NEATO GraphSLAM </h1>
  <h2> Luke Raus, Philip Post, Florian Schwarzinger </h2>
Computational Robotics at Olin College of Engineering - Fall 2022  
  <p> </p>
</div>

<div align="center">
<img src = "assets/happyneatoslam.png" width="500"> 
</div>

## Main Idea and Goal

For his project we wanted to take a deep dive into the GraphSLAM algorithm. The algorithm essentially takes information from the robot’s sensing of itself (interoception) and measurements of the surrounding world (exteroception) and combines it to track both where the robot is and the environment around it. We broke this problem up into 2 main parts: visual object detection and the actual GraphSLAM algorithm. For the visual object detection we wanted to be able to have a moving camera be able to see and recognize landmarks. To keep things simple, we used solid pool balls each with distinct colors so the algorithm could differentiate between which pool ball was which. For the GraphSLAM algorithm, using the object detection and odometry of the robot, we can build up a representation of the data gathered while traversing its environment and then construct a maximum-likelihood map of these surroundings and the trajectory taken through that environment.

## Why GraphSlam?

Coming into this project, the members of the team all had slightly different learning goals. Collectively we wanted to work on something that involved visual landmark detection as well as research and implement an algorithm in the category of SLAM (Simultaneous Localization And Mapping) algorithms. Looking into the different SLAM algorithms, GraphSLAM was kinda the perfect fit for us.

<div align="center">
<img src = "assets/whygraphslam.png" width="500"> 
</div>

GraphSLAM is an algorithm that, like the other algorithms in the SLAM category, can allow a robot to simultaneously keep track of where it is in space as well as generate a map of the world around it. It does this by keeping track of landmarks around the robot as well as its own odometry and creates a graph with each position and landmark in it. You can then take these nodes and run them through an optimization algorithm which will update the graph to be as accurate as possible. The result is a graph of locations the robot has been (and where it currently is) as well as all of the landmarks it has seen and their locations.
