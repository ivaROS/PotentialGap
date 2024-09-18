#ifndef POTENTIAL_GAP_H
#define POTENTIAL_GAP_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <navfn/navfn_ros.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/gap.h>
#include <potential_gap/helper.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/numeric/odeint.hpp>

#include <potential_gap/planner.h>

namespace potential_gap {

    class PotentialGapPlanner : public nav_core::BaseLocalPlanner 
    {
        public: 
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

            /**
            * THIS IS A VIRTUAL FUNCTION THAT MUST BE OVERLOADED.
            * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands
            * to send to the base.
            * @param pose the current pose of the robot.
            * @param velocity the current velocity of the robot.
            * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
            * @param message Optional more detailed outcome as a string
            * @return Result code as described on ExePath action result:
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
            *         121..149 are reserved as plugin specific errors
            */
            uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                const geometry_msgs::TwistStamped& velocity,
                                                geometry_msgs::TwistStamped &cmd_vel,
                                                std::string &message);

            /**
            * THIS IS A VIRTUAL FUNCTION THAT MUST BE OVERLOADED.
            * \brief Use local path planner to compute next command velocity
            * \param cmdVel command velocity to update
            * \return boolean for if command velocity was successfully computed
            */
            bool computeVelocityCommands(geometry_msgs::Twist & cmdVel);

            /**
            * THIS IS A VIRTUAL FUNCTION THAT MUST BE OVERLOADED.
            * @brief  Check if the goal pose has been achieved
            * The actual check is performed in computeVelocityCommands().
            * Only the status flag is checked here.
            * @return True if achieved, false otherwise
            */
            bool isGoalReached();

            /**
            * @brief Dummy version to satisfy MBF API
            */
            bool isGoalReached(double xy_tolerance, double yaw_tolerance) { return isGoalReached(); };

            /**
            * THIS IS A VIRTUAL FUNCTION THAT MUST BE OVERLOADED.
            * @brief  Set the plan that the local planner is following
            * @param globalPlanMapFrame The plan to pass to the local planner
            * @return True if the plan was updated successfully, false otherwise
            */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> & globalPlanMapFrame);

            /**
            * THIS IS A VIRTUAL FUNCTION THAT MUST BE OVERLOADED.
            * @brief Requests the planner to cancel, e.g. if it takes too much time
            * @remark New on MBF API
            * @return True if a cancel has been successfully requested, false if not implemented.
            */
            bool cancel() { return false; };

        private:
            potential_gap::Planner planner;
            std::string planner_name;
            ros::NodeHandle pnh; // nh, 


            // ros::Subscriber feasi_laser_sub;

            // bool initialized = false;

            // boost::shared_ptr<dynamic_reconfigure::Server<potential_gap::pgConfig> > dynamic_recfg_server;
            // dynamic_reconfigure::Server<potential_gap::pgConfig>::CallbackType f;
    };
}

#endif 
