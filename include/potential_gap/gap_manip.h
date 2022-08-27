#ifndef GAP_MOD_H
#define GAP_MOD_H


#include <ros/ros.h>
#include <math.h>
#include <potential_gap/gap.h>
#include <potential_gap/potentialgap_config.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/robot_geo_parser.h>

namespace potential_gap {
    class GapManipulator {
        public: 
            GapManipulator(){};
            ~GapManipulator(){};

            GapManipulator(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc) 
            {
                cfg_ = &cfg;
                robot_geo_proc_ = robot_geo_proc;
            };
            GapManipulator& operator=(GapManipulator & other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            };
            GapManipulator(const GapManipulator &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);

            void setGapWaypoint(potential_gap::Gap&, geometry_msgs::PoseStamped);
            void reduceGap(potential_gap::Gap&, geometry_msgs::PoseStamped);
            void convertAxialGap(potential_gap::Gap&);
            void radialExtendGap(potential_gap::Gap&);
        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            const PotentialGapConfig* cfg_;
            int num_of_scan;
            boost::mutex egolock;

            Eigen::Vector2f car2pol(Eigen::Vector2f);
            Eigen::Vector2f pol2car(Eigen::Vector2f);
            Eigen::Vector2f pTheta(float, float, Eigen::Vector2f, Eigen::Vector2f);
            bool checkGoalVisibility(geometry_msgs::PoseStamped);

            RobotGeoProc robot_geo_proc_;


    };
}

#endif