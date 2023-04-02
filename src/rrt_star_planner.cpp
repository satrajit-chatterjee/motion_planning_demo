#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <nav_msgs/Path.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ompl_rrtstar/ReplanTrigger.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

ros::Publisher plan_pub;
std::string frame_id = "odom";

ros::Subscriber pointcloud_sub_;
sensor_msgs::PointCloud2::ConstPtr global_msg = nullptr;
bool has_map_ = false;

bool has_odom_ = false;

bool start_replan_service = false;

std::vector<double> new_odom = {};

class RRTStarPlanner {
public:
    ros::NodeHandle nh_;
    double _resolution = 0.1;
    double _map_wait = 1.0;
    double plan_duration_ = 1.0;
    double obs_constraint_ = 0.05;
    std::string map_frame_id;    

    RRTStarPlanner(ros::NodeHandle& nh) 
    : nh_(nh){}

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Convert the ROS message to a PCL point cloud

        global_msg = msg;

        has_map_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {

        // Extract position and orientation from the message
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
    
        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        new_odom = {x, y, z, qx, qy, qz, qw};

        has_odom_ = true;
        
        ROS_INFO("Found new start state!");
    }


    void startPlanning() {
        // Reading roslaunch params
        nh_.param("planner/map_wait_duration", _map_wait, 1.0);
        nh_.param("planner/plan_duration", plan_duration_, 1.0);
        nh_.param("planner/obs_constraint", obs_constraint_, 1.0);

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

        // Create an OMPL state space for 3D planning
        ob::StateSpacePtr space(new ob::SE3StateSpace());

        // Set the bounds of the state space to the bounding box of the point cloud
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*map_, min_pt, max_pt);
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, min_pt[0]);
        bounds.setHigh(0, max_pt[0]);
        bounds.setLow(1, min_pt[1]);
        bounds.setHigh(1, max_pt[1]);
        bounds.setLow(2, min_pt[2]);
        bounds.setHigh(2, max_pt[2]);
        space->as<ob::SE3StateSpace>()->setBounds(bounds);

        // Create a simple setup with the state space
        og::SimpleSetup ss(space);

        ob::ScopedState<ob::SE3StateSpace> start(space);
        
        
        // Set a random start state
        bool start_valid = false;
        ROS_INFO("Setting random start position...");
        while (!start_valid) {
            start.random();
            float x = start->getX();
            float y = start->getY();
            float z = start->getZ();

            // Check if the point is in collision with any obstacle in the point cloud
            pcl::PointXYZ point(x, y, z);
            std::vector<int> indices;
            std::vector<float> sqr_dists;
            octree_.nearestKSearch(point, 1, indices, sqr_dists);
            if (sqr_dists[0] > obs_constraint_) {
                start_valid = true;
            }
        }

        ROS_INFO("The random start state is::");
        ROS_INFO("X::%f::Y::%f::Z::%f", start->getX(), start->getY(), start->getZ());

        // Set both start and goal states to random orientations
        double start_q1 = ompl::RNG().uniformReal(0, 2 * M_PI);
        double start_q2 = ompl::RNG().uniformReal(0, 2 * M_PI);
        double start_q3 = ompl::RNG().uniformReal(0, 2 * M_PI);
        double start_q4 = ompl::RNG().uniformReal(0, 2 * M_PI);
        start->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle(start_q1, start_q2, start_q3, start_q4);
        
        
        // Set a random goal state
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        bool goal_valid = false;
        ROS_INFO("Setting random goal position...");
        while (!goal_valid) {
            goal.random();
            float x = goal->getX();
            float y = goal->getY();
            float z = goal->getZ();

            // Check if the point is in collision with any obstacle in the point cloud
            pcl::PointXYZ point(x, y, z);
            std::vector<int> indices;
            std::vector<float> sqr_dists;
            octree_.nearestKSearch(point, 1, indices, sqr_dists);
            if (sqr_dists[0] > obs_constraint_) {
                goal_valid = true;
            }
        }

        ROS_INFO("The random goal state is::");
        ROS_INFO("X::%f::Y::%f::Z::%f", goal->getX(), goal->getY(), goal->getZ());

        double goal_q1 = ompl::RNG().uniformReal(0, 2 * M_PI);
        double goal_q2 = ompl::RNG().uniformReal(0, 2 * M_PI);
        double goal_q3 = ompl::RNG().uniformReal(0, 2 * M_PI);
        double goal_q4 = ompl::RNG().uniformReal(0, 2 * M_PI);
        goal->as<ob::SO3StateSpace::StateType>(1)->setAxisAngle(goal_q1, goal_q2, goal_q3, goal_q4);

        
        ss.setStartAndGoalStates(start, goal);

        // Set the planner
        og::RRTstar* planner = new og::RRTstar(ss.getSpaceInformation());
        ss.setPlanner(ob::PlannerPtr(planner));

        // Set the collision checking function
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

        // Create a path length optimization objective
        ob::OptimizationObjectivePtr obj = std::make_shared<ob::PathLengthOptimizationObjective>(ss.getSpaceInformation());

        // Set the optimization objective for the planner
        ss.setOptimizationObjective(obj);


        // Attempt to solve the problem within given planning time
        ob::PlannerStatus solved = ss.solve(plan_duration_);
 
        if (solved) {
            ROS_INFO("Solution found!");
            
            // Prune generated path by removing unneccessary waypoints
            ss.simplifySolution();

            og::PathGeometric & path = ss.getSolutionPath();
            path.print(std::cout);
            
            publishPlan(path);
            
            ros::Rate loop_rate(10);

            while (ros::ok()) {
                publishPlan(path);
        
                // TODO: Have a service call to reinitiate planning

                // Call any pending callbacks
                ros::spinOnce();

                // Sleep to maintain loop rate
                loop_rate.sleep();
            }
            
        }
        else
            ROS_INFO("No solution found");
    }

    void publishPlan(og::PathGeometric & path) {
        std::vector<ob::State*> path_states = path.getStates();

        // Create a new path message
        nav_msgs::Path plan_msg;

        // Set the header of the path message
        plan_msg.header.frame_id = frame_id;
        plan_msg.header.stamp = ros::Time::now();

        // Add each state to the path message
        for (const auto& state : path_states)
        {
            // Create a new pose stamped message for the state
            geometry_msgs::PoseStamped pose_stamped;

            // Set the header of the pose stamped message
            pose_stamped.header.frame_id = frame_id;
            pose_stamped.header.stamp = ros::Time::now();

            // Set the position of the pose stamped message to the state's X, Y, and Z values
            pose_stamped.pose.position.x = state->as<ob::SE3StateSpace::StateType>()->getX();
            pose_stamped.pose.position.y = state->as<ob::SE3StateSpace::StateType>()->getY();
            pose_stamped.pose.position.z = state->as<ob::SE3StateSpace::StateType>()->getZ();;

            // Set the orientation of the pose stamped message to the state's orientation
            Eigen::Quaterniond q(state->as<ob::SE3StateSpace::StateType>()->rotation().x, 
                                state->as<ob::SE3StateSpace::StateType>()->rotation().y, 
                                state->as<ob::SE3StateSpace::StateType>()->rotation().z,
                                state->as<ob::SE3StateSpace::StateType>()->rotation().w);

            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();


            // Add the pose stamped message to the path message
            plan_msg.poses.push_back(pose_stamped);
        }

        plan_pub.publish(plan_msg);

    }

};


bool replanCallback(ompl_rrtstar::ReplanTrigger::Request &req, 
                    ompl_rrtstar::ReplanTrigger::Response &res) {
    
     ros::NodeHandle nh("~");

    RRTStarPlanner planner_obj(nh);
    planner_obj.startPlanning();
    res.success = true;
    return true;
}


int main (int argc, char** argv){
    ros::init(argc, argv, "rrt_plan_node");
    ros::NodeHandle nh("~");
    // Create a ROS publisher for the plan
    plan_pub = nh.advertise<nav_msgs::Path>("planner/rrt_plan", 1);
    ros::ServiceServer service = nh.advertiseService("replan", replanCallback);

    RRTStarPlanner planner_obj(nh);
    planner_obj.startPlanning();

    return 0;
}