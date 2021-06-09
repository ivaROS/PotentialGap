#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>

#include <traj_generator.h>
#include <turtlebot_trajectory_generator/near_identity.h>
#include <geometry_msgs/PoseArray.h>
#include <potential_gap/helper.h>
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

namespace potential_gap {

    class gap_traj_fun : public virtual turtlebot_trajectory_generator::desired_traj_func
    {
        geometry_msgs::PoseArray desired_traj;
    public:
        gap_traj_fun(geometry_msgs::PoseArray _traj)
        {
            desired_traj = _traj;
        }

        void dState(const turtlebot_trajectory_generator::ni_state &x, turtlebot_trajectory_generator::ni_state &dxdt, const double t)
        {
            int idx = t / 0.2 + 1;
            idx = std::min(idx, (int) desired_traj.poses.size());
            dxdt[6] = desired_traj.poses[idx - 1].position.x - x[0];
            dxdt[7] = desired_traj.poses[idx - 1].position.y - x[1];

        }
    };

    class TrajectoryGenerator {
        public:
            TrajectoryGenerator(){};
            ~TrajectoryGenerator(){};

            TrajectoryGenerator(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg) {cfg_ = &cfg;};
            TrajectoryGenerator& operator=(TrajectoryGenerator & other) {cfg_ = other.cfg_;};
            TrajectoryGenerator(const TrajectoryGenerator &t) {cfg_ = t.cfg_;};

            /**
             * @brief Generate Trajectory for single Gap
             * @param Gap
             * @param waypoint
             */
            virtual geometry_msgs::PoseArray generateTrajectory(potential_gap::Gap, geometry_msgs::PoseStamped) = 0;

            /**
             * @brief Generate vector of trajecctory for a vector of gaps
             * @param gaps
             */
            virtual std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<potential_gap::Gap>) = 0;
        protected:
            const PotentialGapConfig* cfg_;
    };

    class GapTrajGenerator : public TrajectoryGenerator {
        using TrajectoryGenerator::TrajectoryGenerator;
        public:
            
            /**
             * @brief Update to the latest TF infomation, planning frame to odom frame
             * @param tf
             */
            void updateTF(geometry_msgs::TransformStamped tf) {planning2odom = tf;};

            
            geometry_msgs::PoseArray generateTrajectory(potential_gap::Gap, geometry_msgs::PoseStamped);
            std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<potential_gap::Gap>);
            
            /**
             * @brief Move trajectory from egoframe to odom frame
             * @param Trajectory
             * @param tf
             */
            geometry_msgs::PoseArray transformBackTrajectory(geometry_msgs::PoseArray, geometry_msgs::TransformStamped);

            /**
             * @brief Subsample the trajectory so it is fitted for following
             * @param trajectory
             */
            geometry_msgs::PoseArray forwardPassTrajectory(geometry_msgs::PoseArray);

        private: 
            geometry_msgs::TransformStamped planning2odom;

    };
}

#endif