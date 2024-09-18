#include <ros/ros.h>
#include <potential_gap/potential_gap.h>
#include <potential_gap/gap.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

// using namespace boost::numeric::odeint;
namespace pl = std::placeholders;

#define JANKY_PID false
#define NEAR_IDENTITY true

PLUGINLIB_EXPORT_CLASS(potential_gap::PotentialGapPlanner, nav_core::BaseLocalPlanner)

namespace potential_gap 
{
    PotentialGapPlanner::PotentialGapPlanner()
    {
        ROS_INFO_STREAM("Planner constructed");
    }

    PotentialGapPlanner::~PotentialGapPlanner()
    {
        ROS_INFO_STREAM("Planner terminated");
    }

    bool PotentialGapPlanner::isGoalReached()
    {
        ROS_INFO_STREAM("[isGoalReached()]");

        return planner.isGoalReached();
    }

    bool PotentialGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
    {
        ROS_INFO_STREAM("[setPlan()]");

        bool success = planner.setGoal(plan);

        ROS_INFO_STREAM("       success: " << success); 

        return success;
    }

    void PotentialGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_INFO_STREAM("[initialize()]");

        planner_name = name;
        ros::NodeHandle pnh("~/" + planner_name);

        planner.initialize(pnh);

        std::string robot_name = "/robot" + std::to_string(planner.getCurrentAgentCount());

        laser_sub = pnh.subscribe(robot_name + "/mod_laser_0", 100, &Planner::laserScanCB, &planner);
        // inflated_laser_sub = pnh.subscribe("/inflated_point_scan", 100, &Planner::inflatedlaserScanCB, &planner);
        // feasi_laser_sub = pnh.subscribe("/inflated_point_scan", 100, &Planner::inflatedlaserScanCB, &planner);
        
        pose_sub = pnh.subscribe(robot_name + "/odom",10, &Planner::poseCB, &planner);
        initialized = true;

        // Setup dynamic reconfigure
        dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server <potential_gap::pgConfig> > (pnh);
        f = boost::bind(&potential_gap::Planner::rcfgCallback, &planner, _1, _2);
        dynamic_recfg_server->setCallback(f);
    }

    bool PotentialGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
    {
        ROS_INFO_STREAM("[computeVelocityCommands()]");

        if (!planner.initialized())
        {
            ros::NodeHandle pnh("~/" + planner_name);
            planner.initialize(pnh);
            ROS_WARN_STREAM("computerVelocity called before initializing planner");
        }

        auto final_traj = planner.getPlanTrajectory();

        cmd_vel = planner.ctrlGeneration(final_traj);

        return planner.recordAndCheckVel(cmd_vel);
    }

    void PotentialGapPlanner::reset()
    {
        planner.reset();
        return;
    }

}