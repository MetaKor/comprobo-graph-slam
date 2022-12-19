

<div align="center">
  <h1> NEATO GraphSLAM </h1>
  <h2> Luke Raus, Philip Post, Florian Schwarzinger </h2>
Computational Robotics at Olin College of Engineering - Fall 2022
</div>



## Main Idea and Goal

For his project we wanted to take a deep dive into the GraphSLAM algorithm. The algorithm essentially takes information from the robot’s sensing of itself (interoception) and measurements of the surrounding world (exteroception) and combines it to track both where the robot is and the environment around it. We broke this problem up into 2 main parts: visual object detection and the actual GraphSLAM algorithm. For the visual object detection we wanted to be able to have a moving camera be able to see and recognize landmarks. To keep things simple, we used solid pool balls each with distinct colors so the algorithm could differentiate between which pool ball was which. For the GraphSLAM algorithm, using the object detection and odometry of the robot, we can build up a representation of the data gathered while traversing its environment and then construct a maximum-likelihood map of these surroundings and the trajectory taken through that environment.
