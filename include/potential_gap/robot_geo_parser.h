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

        Robot(){};
        Robot(RobotShape in_shape, double robot_length, double robot_width=0)
        {
            shape = in_shape;
            switch (shape)
            {
            case RobotShape::circle:
                if(robot_width != 0)
                    throw std::runtime_error("Circular robot doesn't have width.");

                radius = robot_length;
                break;
            
            case RobotShape::box:
                if(robot_length == 0 || robot_width == 0)
                    throw std::runtime_error("Box robot need length or width.");
                
                length = robot_length;
                width = robot_width;
                diagonal_length = sqrt(length * length + width * width);
                break;
            
            default:
                if(robot_width != 0)
                    throw std::runtime_error("Circular robot doesn't have width.");

                radius = robot_length;
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
                
                double vec_dot_product = o_vec.dot(m_vec);
                Eigen::Vector2d length_vec = robot_.length / 2 * o_vec;
                Eigen::Vector2d left_pt_vec(-length_vec[1], length_vec[0]);
                if(vec_dot_product < 0)
                {
                    left_pt_vec = -left_pt_vec;
                }
                left_pt_vec = left_pt_vec / left_pt_vec.norm();
                left_pt_vec = robot_.width / 2 * left_pt_vec;
                Eigen::Vector2d corner_pt = length_vec + left_pt_vec;

                return 2 * abs(corner_pt[1]);
            }

            double getDecayEquivalentPL(Eigen::Vector2d& orientation_vec, Eigen::Vector2d& motion_vec, double dist)
            {
                double max_epl = getEquivalentPL(orientation_vec, motion_vec);
                double min_epl = robot_.width;
                return (max_epl - min_epl) * exp(-decay_factor_ * dist) + min_epl;
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

        public:
            Robot robot_;
            double decay_factor_ = 0;
            bool initialized_ = false;
    };
}

#endif // ROBOT_GEO_PARSER_H