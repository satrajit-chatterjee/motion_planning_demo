#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree.h>
#include <ompl/base/Planner.h>
#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class RRTStarPlanner {
public:
    ros::NodeHandle nh_;
    bool has_map_;
    double _resolution;
    double _map_wait;
    double plan_duration_;
    double obs_constraint_;
    sensor_msgs::PointCloud2::ConstPtr global_msg = nullptr;
    

    RRTStarPlanner(ros::NodeHandle& nh) 
    : nh_(nh){
        this->has_map_ = false;
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Convert the ROS message to a PCL point cloud

        global_msg = msg;

        this->has_map_ = true;
    }


    void startPlanning() {
        // Reading roslaunch params
        nh_.param("map/resolution", _resolution, 0.1);
        nh_.param("planner/map_wait_duration", _map_wait, 1.0);
        nh_.param("planner/plan_duration", plan_duration_, 1.0);
        nh_.param("planner/obs_constraint", obs_constraint_, 1.0);

        // Subscribe to the point cloud topic
        ros::Subscriber pointcloud_sub_;
        pointcloud_sub_ = nh_.subscribe("/structure_map/global_cloud", 1, &RRTStarPlanner::pointcloudCallback, this);

        // Wait for the first point cloud message to arrive
        while (!this->has_map_) {
            std::cout << "Waiting for map generation..." << std::endl;
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

        // Set the start state
        ob::ScopedState<ob::SE3StateSpace> start(space);
        start->setXYZ(-10.0, -10.0, 0);
        // start.random();
        // bool start_validity = isStateValid(start, octree_);
        
        ob::ScopedState<ob::SE3StateSpace> goal(space);
        goal->setXYZ(20.0, 20.0, 5.0);

        // Setting both start and goal states to identity orientation
        // TODO: randomize both start and goal position and orientation
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
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

        // Attempt to solve the problem within 1 second of planning time
        ob::PlannerStatus solved = ss.solve(plan_duration_);

        if (solved) {
            std::cout << ("Solution found!");
            ss.simplifySolution();
            ss.getSolutionPath().print(std::cout);
        }
        else
            std::cout << "No solution found" << std::endl;
    }

};

int main (int argc, char** argv){
        std::cout << "all OK";
        return 0;
}