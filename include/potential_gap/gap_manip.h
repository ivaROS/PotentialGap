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

namespace potential_gap {
    class GapManipulator {
        public: 
            GapManipulator(){};
            ~GapManipulator(){};

            GapManipulator(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg) {cfg_ = &cfg;};
            GapManipulator& operator=(GapManipulator & other) {cfg_ = other.cfg_;};
            GapManipulator(const GapManipulator &t) {cfg_ = t.cfg_;};

            /**
             * @brief Update latest egocicle measurement and store a share locally
             * @param egocircle 
             */
            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);

            /**
             * @brief Given a Gap and a local waypoint on the relevant global plan, calculate a gap goal and assign to the gap
             * @param gap in consideration
             * @param waypoint
             */
            void setGapWaypoint(potential_gap::Gap&, geometry_msgs::PoseStamped);

            /**
             * @brief Reduce the gap to the appropriate size so that it may display desirable convex properties
             * @param Gap
             * @param waypoint
             */
            void reduceGap(potential_gap::Gap&, geometry_msgs::PoseStamped);

            /**
             * @brief Convert radial gap to swept gaps
             * @param gap
             */
            void convertRadialGap(potential_gap::Gap&);

            /**
             * @brief Radial Extend on Gaps
             * @param gap
             */
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


    };
}

#endif