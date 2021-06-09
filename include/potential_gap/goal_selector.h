#ifndef GOAL_SELECT_H
#define GOAL_SELECT_H

#include <ros/ros.h>
#include <math.h>
#include <potential_gap/gap.h>
#include <potential_gap/potentialgap_config.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace potential_gap
{
    class GoalSelector{
        public: 
            GoalSelector() {};
            ~GoalSelector() {};

            GoalSelector(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg);
            GoalSelector& operator=(GoalSelector other) {cfg_ = other.cfg_;};
            GoalSelector(const GoalSelector &t) {cfg_ = t.cfg_;};

            /**
             * @brief Set current global path
             * @param g_path
             * @return if successfully set
             */
            bool setGoal(const std::vector<geometry_msgs::PoseStamped> &);

            /**
             * @brief Update egocircle information
             * @param egocircle
             */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);

            /**
             * @brief Set waypoint in the robot frame given laserscan and global path
             * @param egocircle
             */
            void updateLocalGoal(geometry_msgs::TransformStamped map2rbt);

        
            /**
             * @brief Return stored local goal of rbt frame in odom frame
             * @param tf
             * @return local goal in odom frame
             */
            geometry_msgs::PoseStamped getCurrentLocalGoal(geometry_msgs::TransformStamped rbt2odom);

            /**
             * @brief Return stored local goal of rbt frame in rbt frame
             * @return local goal in rbt frame
             */
            geometry_msgs::PoseStamped rbtFrameLocalGoal() {return local_goal;};

            /**
             * @brief Return global plan in original odom frame
             * @return plan in odom frame
             */            
            std::vector<geometry_msgs::PoseStamped> getOdomGlobalPlan();

            /**
             * @brief Get global plan that is within egocirlce range
             * @param g_plan
             * @return subset of global plan within egocirlce range
             */
            std::vector<geometry_msgs::PoseStamped> getRelevantGlobalPlan(geometry_msgs::TransformStamped);


        private:
            const PotentialGapConfig* cfg_;
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
            std::vector<geometry_msgs::PoseStamped> global_plan;
            std::vector<geometry_msgs::PoseStamped> mod_plan;
            geometry_msgs::PoseStamped local_goal; // Robot Frame
            boost::mutex goal_select_mutex;
            boost::mutex lscan_mutex;
            boost::mutex gplan_mutex;
            double threshold = 3;

            /**
             * @brief Thresholding
             */
            bool isNotWithin(const double dist);
            
            /**
             * @brief distance to robot
             */
            double dist2rbt(geometry_msgs::PoseStamped);

            /**
             * @brief Angular index of pose projection on polar space
             */
            int PoseIndexInSensorMsg(geometry_msgs::PoseStamped pose);

            /**
             * @brief Pose Orientation
             */
            double getPoseOrientation(geometry_msgs::PoseStamped);

            /**
             * @brief If the pose is not immediately colliding
             (visibly non-colliding or uncertain)
             */
            bool VisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);

            /**
             * @brief If the pose is immediately colliding
             */
            bool NoTVisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);

    };
}

#endif