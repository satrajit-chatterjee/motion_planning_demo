# Motion Planning Demo

## Table of Contents

1. [Problem Description](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#1-problem-description)
2. [Target Milestone](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#2-target-milestone)
3. [Submitted Repositories](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#3-submitted-repositories)
4. [Results](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#4-results)
5. [Algorithm - RRT*](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#5-algorithm---rrt)
6. [Obstacle Avoidance Constraints](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#6-obstacle-avoidance-constraints)
7. [Map Generation](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#7-map-generation)
8. [Trajectory Generation Approach](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#8-trajectory-generation-approach)
9. [Controller Implementation](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#9-controller-implementation)
10. [Setup Instructions](https://github.com/satrajit-chatterjee/motion_planning_demo/edit/master/README.md#10-setup-instructions)

### 1. Problem Description

The problem statement can be found in the Google doc provided [here](https://docs.google.com/document/d/1vWrb-9foCaU-DPw5z-dWApgHkU9c-bylmYKEbX3m5To/edit#heading=h.zur4j868i0). 

Generating a collision-free trajectory in a cluttered environment is also crucial for robot navigation. For optimization-based motion planning,
the first consideration is the trajectory representation. A traditional trajectory representation is a piecewise polynomial while some methods
such as a safe flight corridor (see the figure below), or sum-of-squares constraints could be applied to ensure its safety. By using 
B-spline or Bezier curves, you can continually encode the obstacles avoidance constraints with their convex hull properties. 
There are also different methods to deal with obstacle avoidance and encode the conditions to generate collision-free trajectories. 

### 2. Target Milestone
Writing a ROS node that subscribes to a global map you provide and generate a global optimal collision-free trajectory. Explain which 
method you apply and how you encode the obstacle avoidance constraints.

### 3. Submitted Repositories

- [Motion planning Library](https://github.com/satrajit-chatterjee/motion_planning_demo) - contains the path planner.
- [Map generation library](https://github.com/satrajit-chatterjee/map_generator) (The library is forked from:  https://github.com/yuwei-wu/map_generator
) - contains the random point cloud map generator.
- [Controller library](https://github.com/satrajit-chatterjee/kr_mav_control) (The library is forked from: https://github.com/KumarRobotics/kr_mav_control) - contains the quadrotor controller used to execute plan.

### 4. Results

Attached are visualizations of the target milestones. All visualizations are in RViz.

- [x] Write a ROS node that subscribes to a randomly generated global map.

<a href="url"><img src="https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/assets/map.png" height=50% width=50% ></a>

- [x] Generate a global optimal collision-free trajectory.

<div style="display:flex;">
  <img src="https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/assets/path-blue-1.png" width=40% height=40%>
  <img src="https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/assets/path-blue-2.png" width=40% height=40%>
</div>
<div style="display:flex;">
  <img src="https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/assets/path-green-1.png" width=40% height=40%>
  <img src="https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/assets/path-green-2.png" width=40% height=40%>
</div>

Above are screenshots of generated trajectories visualized in blue and green.

- [x] Implement controller to execute generated trajectory.

### 5. Algorithm - RRT*
The RRT* (Rapidly-exploring Random Tree Star) algorithm is a path planning algorithm that is guaranteed to converge to globally optimal solution as the 
number of iterations approaches infinity. It constructs a tree of states, starting from the initial state, and incrementally growing the tree by adding new states
to it. The algorithm repeatedly samples random states within the configuration space of the agent, and extends the tree towards that state. It continues 
growing the tree and iteratively optimizes the existing paths by rewiring the tree. 

### 6. Obstacle Avoidance Constraints

The obstacle avoidance constraint in this [implementation](https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/src/rrt_star_planner.cpp#L207) 
is that the squared Euclidean distance of a point should be greater than a user-defined threshold from the nearest point in the point cloud. 

```
ss.setStateValidityChecker([&](const ob::State* state) {
            const ob::SE3StateSpace::StateType* se3state = state->as<ob::SE3StateSpace::StateType>();
            float x = se3state->getX();
            float y = se3state->getY();
            float z = se3state->getZ();

            // Check if the point is in collision with any obstacle in the point cloud
            pcl::PointXYZ point(x, y, z);
            std::vector<int> indices;
            std::vector<float> sqr_dists;
            octree_.nearestKSearch(point, 1, indices, sqr_dists);
            if (sqr_dists[0] <= obs_constraint_) {
                return false;
            }

            return true;
        });
```

The point cloud map is represented as an [Octree representation](https://pointclouds.org/documentation/group__octree.html). Given a sampled state by RRT*,
it finds the nearest point in the point cloud to the state. If the distance is less than or equal to the predefined obstacle constraint, the state is 
considered to be in collision. If the distance is greater, the state is considered to be valid and the state validity checker returns 'true'.

### 7. Map Generation

The map generation node is started from within the motion planning launch file [here](https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/launch/rrt_planner.launch#L3).
The generated maps are subscribed to [here](https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/src/rrt_star_planner.cpp#L83), where they are converted to a PCL Octree representation to allow for efficient nearest neighbor searches within the 
configuration space. 

```
// Subscribe to the point cloud topic
pointcloud_sub_ = nh_.subscribe("/structure_map/global_cloud", 1, &RRTStarPlanner::pointcloudCallback, this);

// Wait for the first point cloud message to arrive
while (!has_map_) {
    ROS_INFO("Waiting for map generation...");
    ros::spinOnce();
    ros::Duration(_map_wait).sleep();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr map_ (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::fromROSMsg(*global_msg, *map_);

// Create an octree representation of the point cloud for efficient collision checking
pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_ (_resolution);
octree_.setInputCloud(map_);
octree_.addPointsFromInputCloud();
```

The planning node waits until a map is received before proceeding further. 

### 8. Trajectory Generation Approach

The RRT* planner was implemented in a SO(3) state space using the [OMPL library](https://ompl.kavrakilab.org). The [startPlanning()](https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/src/rrt_star_planner.cpp#L76) 
function subscribes to the point cloud topic "/structure_map/global_cloud" and waits for the first point cloud message to arrive. 
It then creates an [Octree representation](https://pointclouds.org/documentation/group__octree.html) which is used to perform collision checking later. 
The search space ["space"](https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/src/rrt_star_planner.cpp#L101) is initialized by setting
the bounds of the state space to the bounding box of the point cloud. 

Random start and goal states are created after checking their state validity using the same obstacle avoidance constraint described above. After verifying the
start and goal states are not in collision with any obstacle in the point cloud, the planner is setup. A RRT* planner is initialized and the collision checking criteria are provided to it. 

Next an optimization objective is provided. The optimization objective used here is the [OMPL PathLengthOptimizationObjective](https://ompl.kavrakilab.org/classompl_1_1base_1_1PathLengthOptimizationObjective.html) function. 
The objective function works by calculating the length of each candidate path and assigning a cost to it based on its length. This is then used to choose the optimal path that
minimizes the path length. 

Finally, the planner is provided a max planning time and then made to attempt finding the most optimal path given the provided constraints. If a path is found, 
the path is simplified by calling the [simplifySolution](https://github.com/satrajit-chatterjee/motion_planning_demo/blob/master/src/rrt_star_planner.cpp#L239)
function and then it is published as a nav_msgs::Path message. Otherwise it outputs a ROS_INFO message saying no solution was found. 

The simplifySolution function works by trying to eliminate unnecessary waypoints along the found path by iteratively attempting to remove vertices in the 
path and checking whether the path is still valid under given constraints.

### 9. Controller Implementation

The controller manager was implemented in the sub-package named ["sim-manager"](https://github.com/satrajit-chatterjee/kr_mav_control/tree/master/sim_manager) inside the forked controller repository that can be found [here](https://github.com/satrajit-chatterjee/kr_mav_control). In here the sim_manager.py script manages the trajectory execution with a quadrotor. 

Inside here, a separate thread is started that continuously [publishes the odometry](https://github.com/satrajit-chatterjee/kr_mav_control/blob/master/sim_manager/scripts/sim_manager.py#L137) of the quadrotor. In another thread a ["manager"](https://github.com/satrajit-chatterjee/kr_mav_control/blob/master/sim_manager/scripts/sim_manager.py#L152) function is called which starts the motors of the quadrotor, commands it to take off, and then commands it to follow the generated path that the sim_manager node has subscribed to. 

The [plan_callback](https://github.com/satrajit-chatterjee/kr_mav_control/blob/master/sim_manager/scripts/sim_manager.py#L44) for the plan subscriber fetches the published optimal trajectory and generates intermediate waypoints along the trajectory by discretizing the path at a user-defined resolution. 

### 10. Setup Instructions



