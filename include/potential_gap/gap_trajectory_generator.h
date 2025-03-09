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
    inline geometry_msgs::Quaternion yaw2Quat(double yaw)
    {
        double roll = 0, pitch = 0;    
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        
        geometry_msgs::Quaternion quat;
        quat.x = q.x();
        quat.y = q.y();
        quat.z = q.z();
        quat.w = q.w();

        return quat;
    }

    inline double getTang2Yaw(Bezier::Tangent tang)
    {
        double x = tang[0];
        double y = tang[1];
        // double yaw;
        // if(abs(x) < 1e-5 && abs(y) > 1e-5)
        //     yaw = M_PI / 2;
        // else
        //     yaw = atan2(y, x);

        return atan2(y, x);
    }

    inline geometry_msgs::Quaternion getTang2Quat(Bezier::Tangent tang)
    {
        double yaw = getTang2Yaw(tang);

        return yaw2Quat(yaw);
    }

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

    struct BezierPoint {
        double x, y, theta, v, w;
        int idx;
        double time;
        BezierPoint() {};
        BezierPoint(int idx_in, double time_in, double x_in, double y_in, double theta_in, double v_in, double w_in)
        {
            idx = idx_in;
            time = time_in;
            x = x_in;
            y = y_in;
            theta = theta_in;
            v = v_in;
            w = w_in;
        }
    };

    class BezierPath {
    public:
        std::vector<double> x, y, theta, v, w, time;
        double delta_t = 0.02;
        BezierPath() {};

        void add(double x_in, double y_in, double theta_in, double v_in, double w_in)
        {
            x.push_back(x_in);
            y.push_back(y_in);
            theta.push_back(theta_in);
            v.push_back(v_in);
            w.push_back(w_in);
            int orig_idx = time.size();
            time.push_back(orig_idx * delta_t);
        }

        BezierPoint getPoint(int index, double start_time = 0)
        {
            if(index < x.size())
                return BezierPoint(index, start_time + time[index], x[index], y[index], theta[index], v[index], w[index]);
            else
            {
                ROS_ERROR_STREAM("Access [" << index << "] point that is larger than the size of path: [" << x.size() << "].");
                return BezierPoint();
            }
        }

        int size()
        {
            return x.size();
        }

        BezierPoint maxV(int start_idx = 0, int end_idx = -1)
        {
            end_idx = end_idx == -1 ? size() - 1 : end_idx;
            end_idx = end_idx >= size() ? size() - 1 : end_idx;

            auto max_it = std::max_element(v.begin() + start_idx, v.begin() + end_idx + 1);
            int idx = max_it - v.begin();
            return getPoint(idx);
        }

        BezierPoint maxW()
        {
            auto max_it = std::max_element(w.begin(), w.end());
            int idx = max_it - w.begin();
            return getPoint(idx);
        }

        void slowDown(double scale, int start_idx = 0, int end_idx = -1)
        {
            end_idx = end_idx == -1 ? size() - 1 : end_idx;
            end_idx = end_idx >= size() ? size() - 1 : end_idx;

            // ROS_INFO_STREAM("slowDown " << start_idx << " " << end_idx << " " << size() << " " << v.size() << " " << w.size());

            // std::transform(v.begin() + start_idx, v.end() + end_idx + 1, v.begin() + start_idx, std::bind(std::multiplies<double>(), std::placeholders::_1, scale));
            // std::transform(w.begin() + start_idx, w.end() + end_idx + 1, w.begin() + start_idx, std::bind(std::multiplies<double>(), std::placeholders::_1, scale));

            double start_time = time[start_idx];
            v[start_idx] *= scale;
            w[start_idx] *= scale;
            for(int i = start_idx + 1; i <= end_idx; i++)
            {
                v[i] *= scale;
                w[i] *= scale;
                double cur_delta_t = delta_t / scale;
                time[i] = start_time + (i - start_idx) * cur_delta_t;
            }
        }

        void timeAdjust(int start_idx = 0, int end_idx = -1)
        {
            end_idx = end_idx == -1 ? size() - 1 : end_idx;
            end_idx = end_idx >= size() ? size() - 1 : end_idx;

            double start_time = time[start_idx];
            for(int i = start_idx + 1; i <= end_idx; i++)
            {
                time[i] = start_time + (i - start_idx) * delta_t;
            }
        }

        std::vector<geometry_msgs::Pose> toPoseVector()
        {
            std::vector<geometry_msgs::Pose> res; 
            
            for(int i = 0; i < x.size(); i++)
            {
                geometry_msgs::Pose pose;
                pose.position.x = x[i];
                pose.position.y = y[i];
                pose.orientation = yaw2Quat(theta[i]);

                res.push_back(pose);
            }
            return res;
        }

        std::vector<BezierPoint> toBezierPoints(double start_time = 0)
        {
            std::vector<BezierPoint> res;
            for(int i = 0; i < x.size(); i++)
            {
                BezierPoint p = getPoint(i, start_time);
                res.push_back(p);
            }
            return res;
        }
    };

    class BezierPathProfile {
    public:
        std_msgs::Header header;
        std::vector<BezierPath> bezier_paths;

        BezierPathProfile() {};
        ~BezierPathProfile() {};

        void addPath(BezierPath& bp)
        {
            if(bp.size() != 0)
                bezier_paths.push_back(bp);
        }

        void addPaths(std::vector<BezierPath>& bps)
        {
            for(auto& a: bps)
                addPath(a);
        }

        int size()
        {
            return bezier_paths.size();
        }

        BezierPath getPath(int index)
        {
            if(index < bezier_paths.size())
                return bezier_paths[index];
            else
            {
                ROS_ERROR_STREAM("Access [" << index << "] path that is larger than the size of paths: [" << bezier_paths.size() << "].");
                return BezierPath();
            }
        }

        void timeDilation(double vd, double v_max = 1000, double w_max = 1000)
        {
            for(auto& b : bezier_paths)
            {
                BezierPoint max_v_pt = b.maxV();
                if(max_v_pt.v > vd)
                {
                    double scale = vd / max_v_pt.v;
                    b.slowDown(scale);
                }
            }
        }

        void timeDilationSeg(double vd, double v_max = 1000, double w_max = 1000)
        {
            int seg_num = 2;
            for(auto& b : bezier_paths)
            {
                int b_seg_size = (int) round(((double) b.size() / seg_num));

                for(int i = 1; i <= seg_num; i++)
                {
                    int start_idx = (i - 1) * b_seg_size;
                    int end_idx = start_idx + b_seg_size;

                    BezierPoint max_v_pt = b.maxV(start_idx, end_idx);
                    
                    if(max_v_pt.v > vd)
                    {
                        double scale = vd / max_v_pt.v;
                        b.slowDown(scale, start_idx, end_idx);
                    }
                    else
                        b.timeAdjust(start_idx, end_idx);
                }
            }
        }

        geometry_msgs::PoseArray toPoseArray()
        {
            geometry_msgs::PoseArray res;
            res.header = header;

            if(size() == 0)
                return res;

            res.poses = bezier_paths[0].toPoseVector();

            if(size() >= 1)
            {
                for(int i = 1; i < size(); i++)
                {
                    std::vector<geometry_msgs::Pose> cur_poses = bezier_paths[i].toPoseVector();
                    if(cur_poses.size() > 1)
                    {
                        res.poses.pop_back();
                        res.poses.insert(res.poses.end(), cur_poses.begin(), cur_poses.end());
                    }
                }
            }

            return res;
        }

        std::vector<BezierPoint> toBezierPoints(double cur_v, double cur_w)
        {
            std::vector<BezierPoint> res;

            if(size() == 0)
                return res;

            res = bezier_paths[0].toBezierPoints();
            double first_delta_t = res[1].time - res[0].time;
            double end_time = res[res.size() - 1].time;

            if(size() >= 1)
            {
                for(int i = 1; i < size(); i++)
                {
                    std::vector<BezierPoint> cur_bpt = bezier_paths[i].toBezierPoints(end_time);
                    if(cur_bpt.size() > 1)
                    {
                        res.pop_back();
                        res.insert(res.end(), cur_bpt.begin(), cur_bpt.end());
                    }
                    end_time = res[res.size() - 1].time;
                }
            }

            res[0].v = cur_v;
            res[0].w = cur_w;

            return res;
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
            void addToBezierPath(const Bezier::Bezier<N>& bezier_path, const Bezier::Bezier<N-1>& bezier_path_deriv, BezierPath& path, double time, double delta_t);

            template<int N>
            void interpBezierTraj(const Bezier::Bezier<N>& bezier_path, std::vector<geometry_msgs::Pose>& interp_pose);

            geometry_msgs::PoseArray genMultiBezierTrajs(potential_gap::Gap, nav_msgs::Odometry, BezierPathProfile& path_profile);
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
