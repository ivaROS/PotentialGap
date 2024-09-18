#include <ros/ros.h>
#include <potential_gap/potential_gap.h>
#include <potential_gap/gap.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>
// using namespace boost::numeric::odeint;
namespace pl = std::placeholders;

#define JANKY_PID false
#define NEAR_IDENTITY true

PLUGINLIB_EXPORT_CLASS(potential_gap::PotentialGapPlanner, nav_core::BaseLocalPlanner)

namespace potential_gap 
{
    bool PotentialGapPlanner::isGoalReached()
    {
        ROS_INFO_STREAM("[isGoalReached()]");

        return planner.isGoalReached();
    }

    bool PotentialGapPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> & plan)
    {
        ROS_INFO_STREAM("[setPlan()]");

        if (!planner.initialized())
        {
            return false;
        } else
        {
            return planner.setPlan(plan);
        }
    }

    void PotentialGapPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_INFO_STREAM("[initialize()]");

        planner_name = name;
        // ros::NodeHandle pnh("~/" + planner_name);

        planner.initialize(planner_name); // pnh);
    }

    uint32_t PotentialGapPlanner::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                        const geometry_msgs::TwistStamped& velocity,
                                                        geometry_msgs::TwistStamped &cmd_vel,
                                                        std::string &message)
    {
        ROS_INFO_STREAM("[computeVelocityCommands()]");

        if (!planner.initialized())
        {
            planner.initialize(planner_name);
            ROS_WARN_STREAM("computerVelocity called before initializing planner");
        }

        auto final_traj = planner.getPlanTrajectory();

        geometry_msgs::Twist cmdVelNoStamp = planner.ctrlGeneration(final_traj);

        cmd_vel.twist = cmdVelNoStamp;

        bool old_flag = planner.recordAndCheckVel(cmdVelNoStamp);

        /*
        *         SUCCESS           = 0
        *         1..9 are reserved as plugin specific non-error results
        *         FAILURE           = 100  # Unspecified failure, only used for old, non-mfb_core based plugins
        *         CANCELED          = 101
        *         NO_VALID_CMD      = 102
        *         PAT_EXCEEDED      = 103
        *         COLLISION         = 104
        *         OSCILLATION       = 105
        *         ROBOT_STUCK       = 106
        *         MISSED_GOAL       = 107
        *         MISSED_PATH       = 108
        *         BLOCKED_GOAL      = 109
        *         BLOCKED_PATH      = 110
        *         INVALID_PATH      = 111
        *         TF_ERROR          = 112
        *         NOT_INITIALIZED   = 113
        *         INVALID_PLUGIN    = 114
        *         INTERNAL_ERROR    = 115
        *         OUT_OF_MAP        = 116  # The start and / or the goal are outside the map
        *         MAP_ERROR         = 117  # The map is not running properly
        *         STOPPED           = 118  # The controller execution has been stopped rigorously
        */

        return mbf_msgs::ExePathResult::SUCCESS;        
    }

    bool PotentialGapPlanner::computeVelocityCommands(geometry_msgs::Twist & cmd_vel)
    {
        ROS_INFO_STREAM("[PotentialGapPlanner::computeVelocityCommands(short)]");

        std::string dummy_message;
        geometry_msgs::PoseStamped dummy_pose;
        geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;

        bool outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);

        cmd_vel = cmd_vel_stamped.twist;

        ROS_INFO_STREAM("computeVelocityCommands cmdVel: " << cmd_vel);

        // TODO: just hardcoding this now, need to revise
        bool success = 1;

        return success;
    }
}