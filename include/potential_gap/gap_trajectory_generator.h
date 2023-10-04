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
#include <potential_gap/gap_manip.h>
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
#include <potential_gap/utils.h>

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

            TrajectoryGenerator(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc) 
            {
                cfg_ = &cfg;
                robot_geo_proc_ = robot_geo_proc;
            };
            TrajectoryGenerator& operator=(TrajectoryGenerator & other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
                return *this;
            };
            TrajectoryGenerator(const TrajectoryGenerator &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            virtual geometry_msgs::PoseArray generateTrajectory(potential_gap::Gap, geometry_msgs::PoseStamped) = 0;
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
                return *this;
            };

            GapTrajGenerator(const GapTrajGenerator &t) :
                TrajectoryGenerator(t)
            { };

            void updateTF(geometry_msgs::TransformStamped tf) {planning2odom = tf;};
            geometry_msgs::PoseArray generateTrajectory(potential_gap::Gap, geometry_msgs::PoseStamped);
            bool findBezierControlPts(potential_gap::Gap, Bezier::Bezier<2>&, nav_msgs::Odometry, geometry_msgs::TransformStamped);
            geometry_msgs::PoseArray generateBezierTrajectory(potential_gap::Gap, nav_msgs::Odometry, geometry_msgs::TransformStamped);

            geometry_msgs::PoseArray transformBackTrajectory(geometry_msgs::PoseArray, geometry_msgs::TransformStamped, std::string in_frame_id, std::string out_frame_id);
            geometry_msgs::PoseArray forwardPassTrajectory(geometry_msgs::PoseArray);

            template<int N>
            void interpBezierTraj(const Bezier::Bezier<N>& bezier_path, std::vector<geometry_msgs::Pose>& interp_pose);
            geometry_msgs::Quaternion getTang2Quat(Bezier::Tangent tang);

            geometry_msgs::PoseArray genMultiBezierTrajs(potential_gap::Gap, nav_msgs::Odometry);
            bool regulateLocalGoal(const potential_gap::Gap& selectedGap, Eigen::Vector2f& reg_goal);
            Eigen::Vector2f getCircPt(const potential_gap::Gap& selectedGap, const Eigen::Vector2f& local_goal);
            bool robotInitBezierCurve(potential_gap::Gap, const Eigen::Vector2f& circ_pt, Bezier::Bezier<3>&, Eigen::Vector2f& cp2, nav_msgs::Odometry);
            bool robotGoalBezierCurve(potential_gap::Gap, const Eigen::Vector2f& circ_pt, const Eigen::Vector2f& prev_cp2, const Eigen::Vector2f& local_goal, Bezier::Bezier<2>&, nav_msgs::Odometry);

            float getDw(const Eigen::Vector2f& orient1, const Eigen::Vector2f& orient2, float dist, float v)
            {
                Eigen::Vector2f orient1_d = orient1 / orient1.norm();
                Eigen::Vector2f orient2_d = orient2 / orient2.norm();
                float ang_diff = atan2(orient1_d(0) * orient2_d(1) - orient1_d(1) * orient2_d(0), orient1_d(0) * orient2_d(0) + orient1_d(1) * orient2_d(1));
                float dw = abs(ang_diff) / ( dist / v);
                dw = ang_diff > 0 ? dw : -dw;

                return dw;
            }

            Eigen::Vector2f estAcc(float v, float w)
            {
                float delta_time = 0.1;
                float next_theta = w * delta_time;
                Eigen::Vector2f cur_v(v, 0);
                Eigen::Vector2f next_v(v * cos(next_theta), v * sin(next_theta));
                Eigen::Vector2f est_a = (next_v - cur_v) / delta_time;

                return est_a;
            }

        private: 
            geometry_msgs::TransformStamped planning2odom;

            template<int N>
            double getBezierDist(const Bezier::Bezier<N>& bezier_path, double t_start, double t_end, int steps)
            {
                double approx_dist = 0;
                double t_diff = (t_end - t_start) / (steps - 1);
                for(size_t k = 0; k < steps - 1; k++)
                {
                    double x = bezier_path.valueAt(t_start + k * t_diff, 0);
                    double y = bezier_path.valueAt(t_start + k * t_diff, 1);
                    double x_next = bezier_path.valueAt(t_start + (k + 1) * t_diff, 0);
                    double y_next = bezier_path.valueAt(t_start + (k + 1) * t_diff, 1);

                    double dist = sqrt(pow(x - x_next, 2) + pow(y - y_next, 2));
                    approx_dist += dist;
                }
                return approx_dist;
            }

    };
}

#endif
