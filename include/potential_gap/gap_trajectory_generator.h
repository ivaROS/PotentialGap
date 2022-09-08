#ifndef TRAJ_GEN_H
#define TRAJ_GEN_H

#include <ros/ros.h>
#include <boost/numeric/odeint.hpp>

// #include <traj_generator.h>
// #include <turtlebot_trajectory_generator/near_identity.h>
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
#include <nav_msgs/Odometry.h>
#include <potential_gap/bezier.h>
#include <potential_gap/robot_geo_parser.h>

namespace potential_gap {

    class TrajectoryGenerator {
        public:
            TrajectoryGenerator(){};
            ~TrajectoryGenerator(){};

            TrajectoryGenerator(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc) 
            {
                cfg_ = &cfg;
                robot_geo_proc_ = robot_geo_proc;
            };
            TrajectoryGenerator& operator=(TrajectoryGenerator & other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            };
            TrajectoryGenerator(const TrajectoryGenerator &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            virtual geometry_msgs::PoseArray generateTrajectory(potential_gap::Gap, geometry_msgs::PoseStamped) = 0;
            virtual std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<potential_gap::Gap>) = 0;
        protected:
            const PotentialGapConfig* cfg_;
            RobotGeoProc robot_geo_proc_;
    };

    class GapTrajGenerator : public TrajectoryGenerator {
        using TrajectoryGenerator::TrajectoryGenerator;
        public:
            GapTrajGenerator(){};
            ~GapTrajGenerator(){};

            GapTrajGenerator(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc) :
                TrajectoryGenerator(nh, cfg, robot_geo_proc)
            { };

            GapTrajGenerator& operator=(GapTrajGenerator & other)
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
            };

            GapTrajGenerator(const GapTrajGenerator &t) :
                TrajectoryGenerator(t)
            { };

            void updateTF(geometry_msgs::TransformStamped tf) {planning2odom = tf;};
            geometry_msgs::PoseArray generateTrajectory(potential_gap::Gap, geometry_msgs::PoseStamped);
            bool findBezierControlPts(potential_gap::Gap, Bezier::Bezier<2>&, nav_msgs::Odometry, geometry_msgs::TransformStamped);
            geometry_msgs::PoseArray generateBezierTrajectory(potential_gap::Gap, nav_msgs::Odometry, geometry_msgs::TransformStamped);
            std::vector<geometry_msgs::PoseArray> generateTrajectory(std::vector<potential_gap::Gap>);
            geometry_msgs::PoseArray transformBackTrajectory(geometry_msgs::PoseArray, geometry_msgs::TransformStamped);
            geometry_msgs::PoseArray forwardPassTrajectory(geometry_msgs::PoseArray);

        private: 
            geometry_msgs::TransformStamped planning2odom;

            Eigen::Vector2f getRotatedVec(Eigen::Vector2f orig_vec, float chord_length, bool ccw = true);

            bool isLeftofLine(Eigen::Vector2f l1, Eigen::Vector2f l2, Eigen::Vector2f p)
            {
                return ((l2[0] - l1[0])*(p[1] - l1[1]) - (l2[1] - l1[1])*(p[0] - l1[0])) >= 0;
            }

            bool isLargerAngle(Eigen::Vector2f v1, Eigen::Vector2f v2)
            {
                // v1 angle is larger than and equal to v2 angle ccw
                double ang_1 = atan2(v1[1], v1[0]);
                double ang_2 = atan2(v2[1], v2[0]);

                return ang_1 >= ang_2;
            }

            double getBezierDist(Bezier::Bezier<2>& qudraBezier, double t_start, double t_end, int steps)
            {
                double approx_dist = 0;
                double t_diff = (t_end - t_start) / (steps - 1);
                for(size_t k = 0; k < steps - 1; k++)
                {
                    double x = qudraBezier.valueAt(t_start + k * t_diff, 0);
                    double y = qudraBezier.valueAt(t_start + k * t_diff, 1);
                    double x_next = qudraBezier.valueAt(t_start + (k + 1) * t_diff, 0);
                    double y_next = qudraBezier.valueAt(t_start + (k + 1) * t_diff, 1);

                    double dist = sqrt(pow(x - x_next, 2) + pow(y - y_next, 2));
                    approx_dist += dist;
                }
                return approx_dist;
            }
    };
}

#endif