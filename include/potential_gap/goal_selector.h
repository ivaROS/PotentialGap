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
#include <potential_gap/robot_geo_parser.h>

namespace potential_gap
{
    class GoalSelector{
        public: 
            GoalSelector() {};
            ~GoalSelector() {};

            GoalSelector(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc);
            GoalSelector& operator=(GoalSelector other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            };
            GoalSelector(const GoalSelector &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            // Map Frame
            bool setGoal(const std::vector<geometry_msgs::PoseStamped> &);
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);
            void updateLocalGoal(geometry_msgs::TransformStamped map2rbt);
            geometry_msgs::PoseStamped getCurrentLocalGoal(geometry_msgs::TransformStamped rbt2odom);
            geometry_msgs::PoseStamped rbtFrameLocalGoal() {return local_goal;};
            std::vector<geometry_msgs::PoseStamped> getRawGlobalPlan();
            std::vector<geometry_msgs::PoseStamped> getRelevantGlobalPlan(geometry_msgs::TransformStamped);


        private:
            const PotentialGapConfig* cfg_;
            RobotGeoProc robot_geo_proc_;
            boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
            std::vector<geometry_msgs::PoseStamped> global_plan;
            std::vector<geometry_msgs::PoseStamped> mod_plan;
            geometry_msgs::PoseStamped local_goal; // Robot Frame
            boost::mutex goal_select_mutex;
            boost::mutex lscan_mutex;
            boost::mutex gplan_mutex;


            double threshold = 3;

            // If distance to robot is within
            bool isNotWithin(const double dist);
            // Pose to robot, when all in rbt frames
            double dist2rbt(geometry_msgs::PoseStamped);
            int PoseIndexInSensorMsg(geometry_msgs::PoseStamped pose);
            double getPoseOrientation(geometry_msgs::PoseStamped);
            bool VisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);
            bool NoTVisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose);

    };
}

#endif