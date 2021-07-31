#ifndef ODE_H
#define ODE_H

#include <ros/ros.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include "geometry_msgs/PoseArray.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Twist.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace potential_gap {
    typedef boost::array<double, 2> state_type;

    struct polar_gap_field{

        double x1, x2, y1, y2, gx, gy;
        double close_pt_x, close_pt_y, far_pt_x, far_pt_y, far_vec_x, far_vec_y, rbt_vec_x, rbt_vec_y, angle_gap;
        double dir_vec_x, dir_vec_y;
        double _sigma;
        bool _l, _r, _axial;

        polar_gap_field(double x1, double x2, double y1, double y2, double gx, double gy, bool l, bool r, bool axial, double sigma)
            : x1(x1), x2(x2), y1(y1), y2(y2), gx(gx), gy(gy), _l(l), _r(r), _axial(axial), _sigma(sigma) {}

        void operator()(const state_type &x, state_type &dxdt, const double t)
        {
            if (atan2(y1, x1) > atan2(y2, x2)) {
                std::swap(y1, y2);
                std::swap(x1, x2);
            }
            
            Eigen::Vector2d rbt(x[0], x[1]);
            Eigen::Vector2d p1(x1, y1);
            Eigen::Vector2d p2(x2, y2);

            Eigen::Vector2d vec_1 = p1 - rbt;
            Eigen::Vector2d vec_2 = p2 - rbt;

            Eigen::Matrix2d r_pi2;
            double rot_angle = M_PI / 2;
            r_pi2 << std::cos(rot_angle), -std::sin(rot_angle), std::sin(rot_angle), std::cos(rot_angle);
            Eigen::Matrix2d neg_r_pi2;
            neg_r_pi2 << std::cos(-rot_angle), -std::sin(-rot_angle), std::sin(-rot_angle), std::cos(-rot_angle);

            Eigen::Vector2d goal_pt(gx, gy);
            Eigen::Vector2d goal_vec = goal_pt - rbt;

            double r1 = sqrt(pow(x1, 2) + pow(y1, 2));
            double r2 = sqrt(pow(x2, 2) + pow(y2, 2));
            double rx = sqrt(pow(x[0], 2) + pow(x[1], 2));
            double rg = goal_vec.norm();
            double theta1 = atan2(y1, x1);
            double theta2 = atan2(y2, x2);
            double thetax = atan2(x[1], x[0]);
            double thetag = atan2(goal_vec(1), goal_vec(0));

            double new_theta = std::min(std::max(thetag, theta1), theta2);
            double theta_test = std::min(std::max(thetax, theta1), theta2);

            Eigen::Vector2d c1 = r_pi2     * (vec_1 / vec_1.norm()) * exp(-std::abs(thetax - theta1) / _sigma);
            Eigen::Vector2d c2 = neg_r_pi2 * (vec_2 / vec_2.norm()) * exp(-std::abs(theta2 - thetax) / _sigma);

            // Since local goal will definitely be within the range of the gap, this limit poses no difference
            Eigen::Vector2d sub_goal_vec(rg * cos(new_theta), rg * sin(new_theta));

            bool left = r2 > r1;

            bool pass_gap;
            if (_axial) {
                pass_gap = (rbt.norm() > std::min(p1.norm(), p2.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            } else {
                pass_gap = (rbt.norm() > std::max(p1.norm(), p2.norm()) + 0.18) && rbt.norm() > goal_pt.norm();
            }


            Eigen::Vector2d v1 = p1 - p2;
            Eigen::Vector2d v2 = p1 - rbt;

            Eigen::Vector2d polar_vec = rbt.norm() < 1e-3 || pass_gap ? Eigen::Vector2d(0, 0) : rbt / (rbt.norm());

            Eigen::Vector2d result(0, 0); 
            double coeffs = pass_gap ? 0.0 : 1.0;

            Eigen::Vector2d final_goal_vec(0,0);

            if (pass_gap)
            {
                result = Eigen::Vector2d(0, 0);
            } else {
                result = (c1 + c2) * coeffs;
                result += sub_goal_vec / sub_goal_vec.norm();
            }

            dxdt[0] = result(0);
            dxdt[1] = result(1);
            return;
        }
    };

    struct g2g {
        double gx, gy;
        g2g(double gx, double gy)
        : gx(gx), gy(gy) {}

        void operator() ( const state_type &x , state_type &dxdt , const double  t  )
        {
            double goal_norm = sqrt(pow(gx - x[0], 2) + pow(gy - x[1], 2));
            if (goal_norm < 0.1) {
                dxdt[0] = 0;
                dxdt[1] = 0;
            } else {
                dxdt[0] = (gx - x[0]);
                dxdt[1] = (gy - x[1]);
            }
        }
    };

    struct write_trajectory
    {
        geometry_msgs::PoseArray& _posearr;
        std::string _frame_id;
        double _coefs;

        write_trajectory(geometry_msgs::PoseArray& posearr, std::string frame_id, double coefs)
        : _posearr(posearr), _frame_id(frame_id), _coefs(coefs) { }

        void operator()( const state_type &x , double t )
        {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = _frame_id;
            pose.pose.position.x = x[0] / _coefs;
            pose.pose.position.y = x[1] / _coefs;
            pose.pose.position.z = 0;

            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
            pose.pose.orientation.w = 1;
            _posearr.poses.push_back(pose.pose);
        }
    };

}

#endif
