

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

## Why GraphSLAM?

Coming into this project, the members of the team all had slightly different learning goals. Collectively we wanted to work on something that involved visual landmark detection as well as research and implement an algorithm in the category of SLAM (Simultaneous Localization And Mapping) algorithms. Looking into the different SLAM algorithms, GraphSLAM was kinda the perfect fit for us.

<div align="center">
<img src = "assets/whygraphslam.png" width="500"> 
</div>

GraphSLAM is an algorithm that, like the other algorithms in the SLAM category, can allow a robot to simultaneously keep track of where it is in space as well as generate a map of the world around it. It does this by keeping track of landmarks around the robot as well as its own odometry and creates a graph with each position and landmark in it. You can then take these nodes and run them through an optimization algorithm which will update the graph to be as accurate as possible. The result is a graph of locations the robot has been (and where it currently is) as well as all of the landmarks it has seen and their locations.

## Why Visual Landmarks?

We wanted to use visual landmarks because, nowadays, cameras are one of the most ubiquitous sensors on Earth. Not only are they everywhere but cameras are very versatile, providing color, depth, proportion, orientation, and more. Cameras are not infallible and are susceptible to extreme lighting conditions or obstructions. For us, the benefits were far more attractive than the downsides. A visual landmark is a distinct and unique object in the image frame that can reliably be used for geometric references. It can be thought of as an anchor in the image frame that allows us to figure out where the camera is in free space. The more landmarks the better as just one does not give us much confidence of a location. All in all, we chose a method that relies on visual landmarks because it is accessible, versatile, and has a wide array of potential applications.

## How GraphSLAM works

GraphSLAM, at its core, is not so much a single detailed algorithm for solving the SLAM problem than it is a worldview: a general way to represent and think about the localization-and-mapping problem with the end goal of converting the problem into a computationally-tractable optimization problem. Under this broader formulation are many specific choices that can be made about how exactly to represent the data and how to formulate and then solve the optimization.

<div align="center">
<img src = "assets/graphslamoverview.png" width="500"> 
</div>

When we say that GraphSLAM is a graph-based representation of the SLAM problems, it means that the relevant data is encoded as a set of nodes/vertices connected by edges. Graphs are fundamental data structures in many fields, as they let one denote arbitrary relationships between arbitrary elements, so it is fairly natural for them to be useful here. One pertinent caveat: we are used to seeing graphs with edges that denote single scalar weights, such as the strength of a social connection between individuals or the distance between locations. However, in GraphSLAM, edges represent much more complex relationships than this: typically, full geometric transformations with associated uncertainties. This requires additional sophistication in our optimization so as to properly respect these transformations.

Nodes in GraphSLAM come in two types:
- **Robot poses**, traditionally denoted $\mathbf{x}_i$. These represent the pose (location and orientation) of the robot in space. Lacking a better world frame, we can represent these in the frame of the robot’s initial pose.
- **Landmarks**, denoted $\mathbf{z}_i$. These represent the pose of landmarks that we have identified using our sensors. Depending on the application and choice of sensor, these could come in many flavors themselves.

Edges then naturally denote the relationships between these nodes as sensed by the robot.
- **Odometry edges** or **“motion links”** encode the spatial relationship between subsequent poses as measured by the robot’s odometry. They thus link consecutive poses $\mathbf{x}_i$ and $\mathbf{x}_{i+1}$.
- **Observation edges** encode the spatial relationship between a landmark and the pose the robot was in when it made the measurement. They thus link a pose $\mathbf{x}_i$ to a concurrent measurement $\mathbf{z}_j$.

## Intuition for graph optimization

In a perfect world with optimal, noise-free sensors, this information would combine perfectly to  comprise a graph in which the pose transformations between every vertex exactly agree with the pose transformations encoded in the edges. Intuitively, we can imagine this like a bunch of stakes driven into the ground (vertices) and a bunch of rods connecting them (edges) in every which way, and every rod is exactly the right length to connect the rods such that everything perfectly slots together. Of course, anyone who has tried something like this in real life knows that not everything is going to slot together perfectly. The nodes are going to have to get nudged in various directions, and some of the edges are going to have to get stretched or warped so everything fits together to some degree. At the very best, we can arrive at a configuration that is sufficiently “happy,” i.e. none of the edges are too strained.

This is precisely the idea of the optimization underpinning GraphSLAM: that everything in the real world is noisy and error-prone, so obviously the various combinations of edges connecting our poses and landmarks in the graph will not perfectly agree. Optimizing the graph is the process of attempting to move or “jiggle around” the vertices around in the most efficient way possible to make the conflicting edges reach the happiest compromise they can.

## Constraining the optimization

Now that we have categorized our edges which comprise the constraints of our graph optimization, we need to think about the details of how we will represent these constraints, as this has important implications on the optimization step. There are two major paradigms for this representation in the literature, what we will label “heterogeneous” and “homogeneous.”

Some implementations of GraphSLAM make a very clear distinction between the different types of nodes and edges discussed above, and are thus “heterogeneous.” This distinction can be useful because it actually lets us eliminate all landmark nodes by representing landmark measurements as “virtual odometry measurements.” The idea here is that the two measurement links from a pair of poses ($\mathbf{x}_i$ and $\mathbf{x}_j$ to a single landmark can be combined into an additional link between those poses ($\hat{\mathbf{z}}_{ij}$), thus bypassing the landmark. This reduces both dimensionality and sparseness of the graph’s adjacency matrix. However, it also means that optimizing the graph only results in an optimal trajectory, so constructing the optimal map requires a separate step. You can read more about this approach [at our first project blog post here](URL HERE).

However, we have relatively few distinct landmarks, so the sparsity in landmark observation addressed by the “heterogeneous” approach is largely irrelevant to our implementation. Additionally, we would like to optimize the full trajectory and map in a single pass. Thus, we chose to instead optimize over the full graph.

Important to this “homogeneous” formulation is the idea that the different types of edges and nodes discussed above are completely interchangeable. All vertices (both robot pose and landmark) represent a pose, and all edges (both odometry and observation) represent a transformation between such poses. Optimizing the graph then immediately yields a representation of both trajectory and map; one must simply limit attention to robot nodes or landmark nodes.

As always, however, we want a way to encode a representation of uncertainty in the various edges. It is especially sensible that we expect a different degree of uncertainty in the robot’s odometry measurements than in its landmark measurements. In our case, since our extrapolation of the relative pose of a landmark sphere from a camera image will be quite susceptible to noise and imprecision, we would expect greater uncertainty in landmark measurements than from the Neato’s odometry. This corresponds to weighting the landmark edges less strongly in the graph optimization since the information here is weaker.

## How we implemented GraphSlam optimization in Python

## How did we implement visual object detection in Python and ROS

Live visual object detection poses an interesting problem when integrated with ROS. This is because ROS relies on code runs in a constant loop free of any induced lags; therefore, long for or while loops are generally taboo. Yet, object detection involves computational intensive techniques to acquire, categorize, and track an object in a visual frame, creating time lags that would normally break a ROS node or heavily degrade its performance. To overcome this, the main functions of our code were split between multiple threads that allow the main ROS node to constantly and quickly loop without ever having to wait for these tasks to complete. Namely, these tasks were split between keyboard control, video input, and image processing. 

A primary tenant of our object detection process was to make it as user friendly and scalable as possible. We wished for the object categorization and storage to be handled by an interaction with the screen or terminal rather than the traditional process of hardcoding an object into a script. To aid with this process, we created an object storage class that clearly integrated all of the parameters of an object to be accessed at any time. With this helper class implemented, we could move on to the process of recording an object to be recognized and tracked. The user would be presented with a live video stream from the Neato, which was handled on a separate thread to prevent ROS from breaking. From this feed, we enabled the user to click and drag across a section of the screen in order to save an image of only the object and then they would be prompted to enter in the width and name to properly categorize it. This is where the first major roadblock of our project came into force as the active interception of keyboard inputs to move the robot also prevented the user from easily inputting the diameter and name. When attempts were made to prevent keyboard input for Neato movement while the user was needed to enter important details, numerous errors were incurred due to the threading package not being able to stop and start again. 

<div align="center">
<img src = "assets/objectclass.png" width="500"> 
</div>

When an object to track was saved, the next phase of the object detection code would kick into gear. This part of the code focused on properly identifying the saved object within the live video frame, and if seen, the code would record the object it saw, what time, and the relative distance and angle to the object based on the camera’s frame. The trickiest aspect of this process was the actual object detection within the live video frame. This is because video is highly dependent on the background area for contrast and lighting conditions with even small variations potentially breaking our code. Therefore, we decided to go with a two pronged angle of attack for this problem. The first prong was identifying whether there was even an object in frame. Object detection is computationally expensive so if we can avoid doing it then that would save us a lot of trouble and make the overall program more efficient. Therefore, a Hough transformation was first attempted on the image, specifically a circle Hough transformation as we wanted to reduce the number of extraneous variables that could inhibit our code. Our training object of choice were pool balls due to their perfect circle shape and variety of colors at a constant width. The Hough circle transformation attempts to fit a continuous circle curve to the shapes in frame based on the gradient contrast in the frame, so minimal or extreme lighting conditions could worsen the performance. Each circle fitted is a potential match to an object the user has saved and wants to track, so we need to assess whether the objects found in the frame are indeed a proper match. To do this, we relied on the color gradient of the objects. Our helper object storage function already found the primary color of the user inputted image and then each Hough circle’s color would be compared. If the color was close enough, we had a match that could record into a dataframe as well as the distance and angle to said circle centroid, time the object was seen, and the object(s) seen in frame. Once this process was complete, a dataset of these parameters would be passed on to the actual graph SLAM calculation part of the code.

<div align="center">
<img src = "assets/objectidprocess.png" width="500"> 
</div>

This aspect of the code wasn’t without headaches as the lighting conditions of the room dramatically altered the Neato's camera readings. This would interfere with both the Hough transforms and color matching as lighting shifts could reduce the contrast crucial for matching circles and wash out any accurate readings of the matched circle’s color. For future work, a variable aperture camera that was able to adjust to variable lighting conditions would be far better suited to tackling these downsides. 

## How did we integrate those things using ROS

A final setup would require multiple ROS nodes to split the primary tasks of GraphSLAM. Whenever the robot moved more than a specified amount, the first node would collect the current odometry and process the current camera image to see if there are any landmarks in it. From there it would generate a new custom message and send it to the second node to add it to the graph. This second node would receive the messages from the first, add the new node(s) and then run the graph through the optimization algorithm. This structure would be ideal because then one node can focus solely on generating each node and the other can focus only on solving the graph. 

Here’s where we ran into a lot of issues. The landmark detection software was very prone to error. Any changes in lighting would completely throw off the landmark detection and render it ineffective. We also had issues with collecting the camera frame. For some reason which we were not able to resolve, we couldn’t get the first node to store the last image received from the camera. We wanted to set it up so that it would continuously store the last image and when the robot had moved enough, grab that last frame and process it. For some reason the node would always grab the next frame (i.e. it would go through the frames sequentially, 1st, 2nd, 3rd, etc) instead of grabbing the last frame. This meant we would either need to process every frame in order to get to the last frame, something that would take a lot of processing power that we didn’t think we could spare. Another issue we had was with setting up a custom message type to send messages from our first to second node. We followed several tutorials online and kept running into issues upon compiling that, when we looked them up, were unresolved by the larger ROS community. These issues became pretty prohibitive to any ROS integration. I’m sure we would have had more issues to debug but we couldn't even get past the image processing or sending messages step, both of which are crucial for the rest of the project to function.

These are all issues that, given more time, we would be able to solve or work around but given the constraints of the project and the timeframe and workload of the group, we weren't able to put in all the time to fix these issues.

## Takeaways

-It is important to map out at the beginning of the project how much of a program should be made up of separate nodes versus threads.
-Camera based processing and recognition techniques are heavily dependent on background lighting and color.
-It is important to use standardized ways to store and categorize data to ease scaling.

