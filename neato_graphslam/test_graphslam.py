import numpy as np
from graphslam.graph import Graph
from graphslam.vertex import Vertex
from graphslam.pose.se2 import PoseSE2
from graphslam.edge.edge_odometry import EdgeOdometry


def main():

    odometry_info_matrix = np.array([[2,0,0], [0,2,0], [0,0,1]])
    landmark_info_matrix = np.array([[1,0,0], [0,1,0], [0,0,0]])

    # Make PoseSE2 objects representing robot's "true" poses
    # ang_d represents angle along unit circle on which to place robot
    # Robot's heading is +90 degrees offset from this angle (at bottom, pointing right)
    truePoses = [ PoseSE2([ np.cos(np.radians(ang_d)), \
                            np.sin(np.radians(ang_d)) ], \
                          np.radians(ang_d+90)) \
                 for ang_d in range(-90, 90+1, 20) ]

    # Give poses additive Gaussian noise
    noisyPoses = [ truePose + PoseSE2(np.random.normal(scale=0.5, size=2), \
                                      np.random.normal(scale=0.03) ) \
                 for truePose in truePoses ]

    n_poses = len(noisyPoses)

    # Landmark is at origin; initialize with some Gaussian noise
    landmark = PoseSE2(np.random.normal(scale=0.1, size=2), 0)

    # Convert landmark pose and robot poses to vertices in list
    # Landmark is vertex ID = n_poses = -1 
    vertices = [ Vertex(i, noisyPoses[i]) for i in range(n_poses) ] + \
               [ Vertex(n_poses, landmark) ]

    # Creates edges between all robot poses
    odom_edges = [ EdgeOdometry( [i, i+1], \
                                 odometry_info_matrix, \
                                 truePoses[i+1] - truePoses[i] + \
                                     PoseSE2(np.random.normal(scale=0.05, size=2), np.random.normal(scale=0.02)), #estimate
                                 [vertices[i], vertices[i+1]]) \
                  for i in range( n_poses-1 ) ]
    
    # Landmark edge information matrices have a zero on 3rd diagonal element
    # to signify that orientation/angle of landmarks is irrelevant
    landmark_edges = [ EdgeOdometry( [n_poses, i],
                                     landmark_info_matrix,
                                     -truePoses[i] + PoseSE2(np.random.normal(scale=0.05, size=2), 0), \
                                     [vertices[n_poses], vertices[i]] ) \
                      for i in range( n_poses ) ]

    edges = odom_edges + landmark_edges

    # print(vertices)
    # print(edges)

    g = Graph(edges, vertices)

    # Plot original version
    print("Plotting base graph...")
    g.plot()
    # Optimize graph
    print("Optimizing graph...")
    g.optimize()
    # Plot optimized graph
    print("Plotting optimized graph...")
    g.plot()



if __name__ == '__main__':
    main()