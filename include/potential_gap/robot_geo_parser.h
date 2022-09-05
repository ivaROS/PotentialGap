#ifndef ROBOT_GEO_PARSER_H
#define ROBOT_GEO_PARSER_H

#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace potential_gap{
    enum RobotShape{
        circle,
        box
    };

    struct Robot{
        RobotShape shape;
        double radius = 0, length = 0, width = 0, diagonal_length = 0;
        double avg_lin_speed, avg_rot_speed;

        Robot(){};
        Robot(RobotShape in_shape, double robot_length, double robot_width=0, double robot_avg_lin_speed=0.2, double robot_avg_rot_speed=0.5)
        {
            shape = in_shape;
            switch (shape)
            {
            case RobotShape::circle:
                if(robot_width != 0)
                    throw std::runtime_error("Circular robot doesn't have width.");

                radius = robot_length;
                avg_lin_speed = robot_avg_lin_speed;
                avg_rot_speed = robot_avg_rot_speed;
                break;
            
            case RobotShape::box:
                if(robot_length == 0 || robot_width == 0)
                    throw std::runtime_error("Box robot need length or width.");
                
                length = robot_length;
                width = robot_width;
                diagonal_length = sqrt(length * length + width * width);
                avg_lin_speed = robot_avg_lin_speed;
                avg_rot_speed = robot_avg_rot_speed;
                break;
            
            default:
                if(robot_width != 0)
                    throw std::runtime_error("Circular robot doesn't have width.");

                radius = robot_length;
                avg_lin_speed = robot_avg_lin_speed;
                avg_rot_speed = robot_avg_rot_speed;
                break;
            }
        }
    };

    class RobotGeoStorage{
        public:
            RobotGeoStorage() {};
            ~RobotGeoStorage() 
            {
                initialized_ = false;
            };
            RobotGeoStorage(std::string config_file);
            void loadConfigFile(std::string config_file);

        private:
            std::vector<double> vec_dot_product_, equivalent_radius_, equivalent_pass_len_;
            bool initialized_ = false;

        public:
            double getInterp(double vec_dot_product, std::vector<double>& target_vec);
            double getInterpEquivR(double vec_dot_product);
            double getInterpEquivPL(double vec_dot_product);

            bool initialized() {return initialized_;}
    };

    class RobotGeoProc{
        public:
            RobotGeoProc() {};
            ~RobotGeoProc() 
            {
                initialized_ = false;
            }

            RobotGeoProc(Robot robot, double decay_factor = 0)
            {
                robot_ = robot;
                if(robot_.shape == RobotShape::circle)
                    decay_factor_ = 0;
                else
                    decay_factor_ = decay_factor;
                initialized_ = true;
            }

            bool initialized() {return initialized_;}

            double getEquivalentR(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& pt_direct)
            {
                if(robot_.shape == RobotShape::circle)
                    return robot_.radius;
                
                Eigen::Vector2d o_vec = orientation_vec;
                Eigen::Vector2d p_vec = pt_direct;
                if(o_vec.norm() != 1)
                    o_vec = o_vec / o_vec.norm();
                
                if(p_vec.norm() != 1)
                    p_vec = p_vec / p_vec.norm();

                double er;
                double vec_dot_product = o_vec.dot(p_vec);

                if(vec_dot_product > (robot_.length / robot_.diagonal_length) && vec_dot_product <= 1)
                {
                    er = robot_.length / 2 / vec_dot_product;
                }
                else if(vec_dot_product <= (robot_.length / robot_.diagonal_length) && vec_dot_product > -(robot_.length / robot_.diagonal_length))
                {
                    er = robot_.width / 2 / (sqrt(1 - vec_dot_product * vec_dot_product));
                }
                else
                {
                    er = -robot_.length / 2 / vec_dot_product;
                }
                return er;
            }

            double getEquivalentPL(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& motion_vec)
            {
                if(robot_.shape == RobotShape::circle)
                    return 2 * robot_.radius;

                Eigen::Vector2d o_vec = orientation_vec;
                Eigen::Vector2d m_vec = motion_vec;
                if(o_vec.norm() != 1)
                    o_vec = o_vec / o_vec.norm();
                
                if(m_vec.norm() != 1)
                    m_vec = m_vec / m_vec.norm();

                double m_ang = atan2(m_vec[1], m_vec[0]);
                double o_ang = atan2(o_vec[1], o_vec[0]);
                // double o_new_ang = o_ang - m_ang;
                // Eigen::Vector2d o_new_vec(cos(o_new_ang), sin(o_new_ang));
                double rot_ang = 0 - m_ang;
                Eigen::Matrix2d rot;
                rot << cos(rot_ang), -sin(rot_ang), sin(rot_ang), cos(rot_ang);
                Eigen::Vector2d o_new_vec = rot * o_vec;
                double o_new_ang = atan2(o_new_vec[1], o_new_vec[0]);
                Eigen::Vector2d m_new_vec(1, 0);
                
                Eigen::Vector2d length_vec = robot_.length / 2 * o_new_vec;
                Eigen::Vector2d left_pt_vec;
                if(o_new_ang <= -M_PI / 2 || (o_new_ang > 0 && o_new_ang <= M_PI / 2)) // TODO: double check
                {
                    left_pt_vec[0] = -length_vec[1];
                    left_pt_vec[1] = length_vec[0];
                }
                else if((o_new_ang > -M_PI / 2 && o_new_ang <= 0) || o_new_ang > M_PI / 2)
                {
                    left_pt_vec[0] = length_vec[1];
                    left_pt_vec[1] = -length_vec[0];
                }
                
                left_pt_vec = left_pt_vec / left_pt_vec.norm();
                left_pt_vec = robot_.width / 2 * left_pt_vec;
                Eigen::Vector2d corner_pt = length_vec + left_pt_vec;

                return 2 * abs(corner_pt[1]);
            }

            double getEquivalentRL(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& motion_vec)
            {
                // Motion vec cannot be normalized

                if(robot_.shape == RobotShape::circle)
                    return 2 * robot_.radius;

                Eigen::Vector2d o_vec = orientation_vec;
                Eigen::Vector2d m_vec = motion_vec;
                if(o_vec.norm() != 1)
                    o_vec = o_vec / o_vec.norm();
                
                if(m_vec.norm() != 1)
                    m_vec = m_vec / m_vec.norm();

                double m_ang = atan2(m_vec[1], m_vec[0]);
                double o_ang = atan2(o_vec[1], o_vec[0]);
                // double o_new_ang = o_ang - m_ang;
                // Eigen::Vector2d o_new_vec(cos(o_new_ang), sin(o_new_ang));
                double rot_ang = 0 - m_ang;
                Eigen::Matrix2d rot;
                rot << cos(rot_ang), -sin(rot_ang), sin(rot_ang), cos(rot_ang);
                Eigen::Vector2d o_new_vec = rot * o_vec;
                double o_new_ang = atan2(o_new_vec[1], o_new_vec[0]);
                Eigen::Vector2d m_new_vec(1, 0);
                
                Eigen::Vector2d length_vec = robot_.length / 2 * o_new_vec;
                Eigen::Vector2d pt_vec;
                if(o_new_ang <= -M_PI / 2 || (o_new_ang > 0 && o_new_ang <= M_PI / 2)) // TODO: double check
                {
                    pt_vec[0] = length_vec[1];
                    pt_vec[1] = -length_vec[0];
                }
                else if((o_new_ang > -M_PI / 2 && o_new_ang <= 0) || o_new_ang > M_PI / 2)
                {
                    pt_vec[0] = -length_vec[1];
                    pt_vec[1] = length_vec[0];
                }
                
                pt_vec = pt_vec / pt_vec.norm();
                pt_vec = robot_.width / 2 * pt_vec;
                Eigen::Vector2d corner_pt = motion_vec + length_vec + pt_vec;
                Eigen::Vector2d opposite_corner_pt = motion_vec - length_vec - pt_vec;

                return abs(corner_pt.norm() - opposite_corner_pt.norm());
            }

            double getDecayEquivalentPL(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& motion_vec, double dist)
            {
                double max_epl = getEquivalentPL(orientation_vec, motion_vec);
                double min_epl = robot_.width;
                return (max_epl - min_epl) * exp(-decay_factor_ * dist) + min_epl;
            }

            double getLinearDecayEquivalentPL(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& motion_vec, double dist)
            {
                double t = dist / robot_.avg_lin_speed;
                double ang = t * robot_.avg_rot_speed;
                double ang_diff = orientation_vec.dot(motion_vec) / (orientation_vec.norm() * motion_vec.norm());
                double cur_ang = acos(ang_diff) - ang;
                cur_ang = cur_ang >= 0 ? cur_ang : 0;
                if(ang_diff < 0)
                    cur_ang = -cur_ang;

                double m_ang = atan2(motion_vec[1], motion_vec[0]);
                cur_ang = m_ang + cur_ang;
                Eigen::Vector2d cur_vec(cos(cur_ang), sin(cur_ang));

                double epl = getEquivalentPL(cur_vec, motion_vec);
                return epl;
            }

            double getLinearDecayEquivalentRL(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& motion_vec, double dist)
            {
                double t = dist / robot_.avg_lin_speed;
                double ang = t * robot_.avg_rot_speed;
                double ang_diff = orientation_vec.dot(motion_vec) / (orientation_vec.norm() * motion_vec.norm());
                double cur_ang = acos(ang_diff) - ang;
                cur_ang = cur_ang >= 0 ? cur_ang : 0;
                if(ang_diff < 0)
                    cur_ang = -cur_ang;

                double m_ang = atan2(motion_vec[1], motion_vec[0]);
                cur_ang = m_ang + cur_ang;
                Eigen::Vector2d cur_vec(cos(cur_ang), sin(cur_ang));

                Eigen::Vector2d m_new_vec = dist * motion_vec / motion_vec.norm();
                double erl = getEquivalentRL(cur_vec, m_new_vec);
                return erl;
            }

            double getNearestDistance(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& pt)
            {
                // The pt is the relative vector from robot origin.

                int sample_size = 20; // TOO SLOW
                double res = M_PI * 2 / sample_size;

                double o_ang = atan2(orientation_vec[1], orientation_vec[0]);
                double rot_ang = 0 - o_ang;
                Eigen::Matrix2d rot;
                rot << cos(rot_ang), -sin(rot_ang), sin(rot_ang), cos(rot_ang);
                Eigen::Vector2d pt_new_vec = rot * pt;
                
                std::vector<double> dists(sample_size);

                for(size_t i = 0; i < sample_size; i++)
                {
                    double ang = i * res - M_PI;
                    ang = (ang <= M_PI) ? ang : M_PI;
                    ang = (ang >= -M_PI) ? ang : -M_PI;

                    Eigen::Vector2d orien_vec(1, 0);
                    Eigen::Vector2d i_vec(cos(ang), sin(ang));
                    double dist = getEquivalentR(orien_vec, i_vec);

                    Eigen::Vector2d i_bound = dist * i_vec;

                    dists[i] = (pt_new_vec - i_bound).norm();
                }

                return *std::min_element(dists.begin(), dists.end());
            }

            double getRobotMaxRadius()
            {
                if(robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                }
                else if(robot_.shape == RobotShape::box)
                {
                    return robot_.diagonal_length / 2;
                }
            }

            double getRobotMinRadius()
            {
                if(robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                }
                else if(robot_.shape == RobotShape::box)
                {
                    return robot_.width / 2;
                }
            }

            double getRobotHalfLength()
            {
                if(robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                }
                else if(robot_.shape == RobotShape::box)
                {
                    return robot_.length / 2;
                }
            }

            double getRobotHalfWidth()
            {
                if(robot_.shape == RobotShape::circle)
                {
                    return robot_.radius;
                }
                else if(robot_.shape == RobotShape::box)
                {
                    return robot_.width / 2;
                }
            }

        public:
            Robot robot_;
            double decay_factor_ = 0;
            bool initialized_ = false;
    };
}

#endif // ROBOT_GEO_PARSER_H