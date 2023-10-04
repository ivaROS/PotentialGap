#include <potential_gap/gap_trajectory_generator.h>

namespace potential_gap{
    geometry_msgs::PoseArray GapTrajGenerator::generateTrajectory(potential_gap::Gap selectedGap, geometry_msgs::PoseStamped curr_pose) {
        // return geometry_msgs::PoseArray();
        geometry_msgs::PoseArray posearr;
        posearr.header.stamp = ros::Time::now();
        
        double coefs = cfg_->traj.scale;
        write_trajectory corder(posearr, cfg_->robot_frame_id, coefs);
        // posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;
        posearr.header.frame_id = cfg_->robot_frame_id;

        if (selectedGap.goal.discard) {
            return posearr;
        }

        state_type x = {curr_pose.pose.position.x + 1e-5, curr_pose.pose.position.y + 1e-6}; 

        if (selectedGap.goal.goalwithin) {
            // ROS_INFO_STREAM("Goal to Goal");
            g2g inte_g2g(
                selectedGap.goal.x * coefs,
                selectedGap.goal.y * coefs);
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            inte_g2g, x, 0.0,
            cfg_->traj.integrate_maxt,
            cfg_->traj.integrate_stept,
            corder);
            return posearr;
        }

        float x1, x2, y1, y2;
        float half_num_scan = selectedGap.half_scan;
        x1 = (selectedGap.convex.convex_ldist) * cos(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (selectedGap.convex.convex_ldist) * sin(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        x2 = (selectedGap.convex.convex_rdist) * cos(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (selectedGap.convex.convex_rdist) * sin(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);

        if (selectedGap.mode.convex) {
            x = {- selectedGap.qB(0) - 1e-6, - selectedGap.qB(1) + 1e-6};
            x1 -= selectedGap.qB(0);
            x2 -= selectedGap.qB(0);
            y1 -= selectedGap.qB(1);
            y2 -= selectedGap.qB(1);
            selectedGap.goal.x -= selectedGap.qB(0);
            selectedGap.goal.y -= selectedGap.qB(1);

        }
        
        polar_gap_field inte(x1 * coefs, x2 * coefs,
                            y1 * coefs, y2 * coefs,
                            selectedGap.goal.x * coefs,
                            selectedGap.goal.y * coefs,
                            selectedGap.getLeftObs(),
                            selectedGap.getRightObs(),
                            selectedGap.isAxial(),
                            cfg_->gap_manip.sigma);
        boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            inte, x, 0.0,
            cfg_->traj.integrate_maxt,
            cfg_->traj.integrate_stept, corder);

        if (selectedGap.mode.convex) {
            for (auto & p : posearr.poses) {
                p.position.x += selectedGap.qB(0);
                p.position.y += selectedGap.qB(1);
            }
        }

        return posearr;
    }

    bool GapTrajGenerator::findBezierControlPts(potential_gap::Gap selectedGap, Bezier::Bezier<2>& bezier_curve, nav_msgs::Odometry curr_odom, geometry_msgs::TransformStamped odom2rbt)
    {
        // Find the intersections of triangle and circle
        float x1, x2, y1, y2;
        float half_num_scan = selectedGap.half_scan;
        x1 = (selectedGap.convex.convex_ldist) * cos(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (selectedGap.convex.convex_ldist) * sin(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        x2 = (selectedGap.convex.convex_rdist) * cos(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (selectedGap.convex.convex_rdist) * sin(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);

        float goal_x = selectedGap.goal.x;
        float goal_y = selectedGap.goal.y;

        // ROS_INFO_STREAM(goal_x << " " << goal_y << " " << selectedGap.goal.goalwithin);

        // Check if goal is in the middle
        double ang_l = std::atan2(y1, x1);
        double ang_r = std::atan2(y2, x2);
        double ang_goal = std::atan2(goal_y, goal_x);

        // assert(ang_goal >= ang_l && ang_goal <= ang_r);

        Eigen::Vector2f l_vec(x1, y1);
        Eigen::Vector2f r_vec(x2, y2);
        float circ_r = selectedGap.getMinSafeDist();
        assert(circ_r <= l_vec.norm() && circ_r <= r_vec.norm());
        Eigen::Vector2f l_inter = circ_r * l_vec / l_vec.norm();
        Eigen::Vector2f r_inter = circ_r * r_vec / r_vec.norm();
        Eigen::Vector2f goal_vec(goal_x, goal_y);

        Eigen::Vector2f rbt_orient_vec(1, 0);
        float x_speed = curr_odom.twist.twist.linear.x;
        float ideal_min_cp_length = x_speed / 2; // For quadratic bezier curve, the ideally length B'(0) = 2 (P_1 - P_0) 

        // Find the closet gap side to orientation. By default, l side is closer.
        bool l_side = true;
        Eigen::Vector2f chosen_inter = l_inter;
        Eigen::Vector2f other_inter = r_inter;
        if(abs(ang_l) > abs(ang_r))
        {
            l_side = false;
            chosen_inter = r_inter;
            other_inter = l_inter;
        }

        float robot_geo_scale = cfg_->traj.robot_geo_scale;
        float robot_geo_thresh_dist = float(robot_geo_proc_.getRobotMinRadius()) * robot_geo_scale; // width / 2

        Eigen::Vector2f cp, new_goal;
        bool success = false;

        float robot_geo_diagonal_thresh = float(robot_geo_proc_.getRobotMaxRadius()) * robot_geo_scale;
        // float cp_max_length = circ_r - robot_geo_diagonal_thresh;
        float cp_max_length = circ_r - robot_geo_thresh_dist;
        
        if(cp_max_length <= 0)
        {
            ROS_WARN_STREAM("The circle is smaller than robot thresh.");
            return success;
        }

        // If the goal is inside the circle, directly go to goal
        if(goal_vec.norm() <= circ_r)
        {
            if(ideal_min_cp_length > cp_max_length)
                cp = cp_max_length * rbt_orient_vec;
            else
                cp = ideal_min_cp_length * rbt_orient_vec;
            
            new_goal = goal_vec;

            bezier_curve = Bezier::Bezier<2>({ {0, 0}, {cp[0], cp[1]}, 
                                          {new_goal[0], new_goal[1]} });

            success = true;

            ROS_DEBUG_STREAM("Goal is within circle.");
            return success;
        }

        // Conditions
        if(ang_l <= 0 && ang_r > 0)
        {
            double chosen_ang = atan2(chosen_inter[1], chosen_inter[0]);

            if(abs(chosen_ang) <= M_PI / 2)
            {
                float dist_inter_orient = abs(chosen_inter[1]);

                if(dist_inter_orient >= robot_geo_thresh_dist)
                {
                    // Second control point
                    // if(ideal_min_cp_length < circ_r)
                    // {
                    //     float cp_length = circ_r * float(cfg_->traj.bezier_cp_scale);
                    //     cp = cp_length * rbt_orient_vec;
                    // }
                    // else
                    // {
                    //     cp = ideal_min_cp_length * rbt_orient_vec;
                    // }
                    if(ideal_min_cp_length > cp_max_length)
                        cp = cp_max_length * rbt_orient_vec;
                    else
                        cp = ideal_min_cp_length * rbt_orient_vec;

                    // Goal point region
                    success = true;

                    // Eigen::Vector2f cp_l_vec = l_vec - cp;
                    // Eigen::Vector2f cp_l_vec_rot = utils::getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                    // Eigen::Vector2f cp_r_vec = r_vec - cp;
                    // Eigen::Vector2f cp_r_vec_rot = utils::getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                    // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                    // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                    

                    // ROS_INFO_STREAM("Within 1: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);
                    Eigen::Vector2f l_used_vec = chosen_inter;
                    Eigen::Vector2f r_used_vec = other_inter;
                    if(!l_side)
                    {
                        l_used_vec = other_inter;
                        r_used_vec = chosen_inter;
                    }

                    Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);
                    Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

                    Eigen::Vector2f l_new = l_used_vec + robot_geo_thresh_dist * l_normal_vec / l_normal_vec.norm();
                    Eigen::Vector2f r_new = r_used_vec + robot_geo_thresh_dist * r_normal_vec / r_normal_vec.norm();

                    ROS_DEBUG_STREAM("Within 1: " << cp[0] << " " << cp[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);


                    // if(!utils::isLeftofLine(cp, l_new_vec, r_new_vec))
                    // if(!utils::isLargerAngle(cp_r_vec_rot, cp_l_vec_rot))
                    if(!utils::isLargerAngle(r_new, l_new))
                    {
                        success = false;
                        ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 1]");
                    }
                    else
                    {

                        // bool left_side_of_l = utils::isLeftofLine(cp, l_new_vec, goal_vec);
                        // bool left_side_of_r = utils::isLeftofLine(cp, r_new_vec, goal_vec);
                        // bool larger_than_l = utils::isLargerAngle(goal_vec - cp, cp_l_vec_rot);
                        // bool larger_than_r = utils::isLargerAngle(goal_vec - cp, cp_r_vec_rot);
                        // ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);
                        bool larger_than_l = utils::isLargerAngle(goal_vec, l_new);
                        bool larger_than_r = utils::isLargerAngle(goal_vec, r_new);

                        // if(!left_side_of_l)
                        if(!larger_than_l)
                        {
                            // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                            // new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                            new_goal = goal_vec.norm() * l_new / l_new.norm();
                        }
                        // else if(left_side_of_l && left_side_of_r)
                        else if(larger_than_l && larger_than_r)
                        {
                            // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                            // new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
                            new_goal = goal_vec.norm() * r_new / r_new.norm();
                        }
                        // else if(left_side_of_l && !left_side_of_r)
                        else if(larger_than_l && !larger_than_r)
                        {
                            new_goal = goal_vec;
                        }
                        else
                        {
                            ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap 1]");
                            success = false;
                        }
                    }
                }
                else
                {
                    float dist_other_inter_orient = abs(other_inter[1]);
                    if(dist_other_inter_orient >= robot_geo_thresh_dist)
                    {
                        // Find the intersect for max length / 2
                        float length_devi = sqrt(robot_geo_diagonal_thresh * robot_geo_diagonal_thresh - dist_other_inter_orient * dist_other_inter_orient);
                        Eigen::Vector2f max_cp(chosen_inter[0] - length_devi, 0);
                        
                        bool cp_success = false;

                        if(max_cp[0] <= 0)
                        {
                            ROS_WARN_STREAM("The chosen intersection point is too close to robot. (smaller than diagonal / 2)");
                            success = false;
                        }
                        else
                        {
                            if(max_cp.norm() <= ideal_min_cp_length)
                                cp = max_cp.norm() * rbt_orient_vec;
                            else
                                cp = ideal_min_cp_length * rbt_orient_vec;

                            cp_success = true;
                        }

                        if(cp_success)
                        {
                            success = true;
                            Eigen::Vector2f origin(0, 0);
                            if(l_side)
                            {
                                // Eigen::Vector2f cp_r_vec = r_vec - cp;
                                // Eigen::Vector2f cp_r_vec_rot = utils::getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                                // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;

                                // ROS_INFO_STREAM("Within 2 l side: " << cp[0] << " " << cp[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);

                                Eigen::Vector2f r_used_vec = other_inter;
                                Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

                                Eigen::Vector2f r_new = r_used_vec + robot_geo_thresh_dist * r_normal_vec / r_normal_vec.norm();

                                ROS_DEBUG_STREAM("Within 2 l side: " << cp[0] << " " << cp[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);

                                // if(!utils::isLeftofLine(origin, cp, r_new_vec))
                                // if(!utils::isLargerAngle(cp_r_vec_rot, Eigen::Vector2f(1,0)))
                                if(!utils::isLargerAngle(r_new, Eigen::Vector2f(1,0)))
                                {
                                    success = false;
                                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 2 l side]");
                                }
                                else
                                {
                                
                                    // bool left_side_of_l = utils::isLeftofLine(origin, cp, goal_vec);
                                    // bool left_side_of_r = utils::isLeftofLine(cp, r_new_vec, goal_vec);
                                    // bool larger_than_l = utils::isLargerAngle(goal_vec - cp, Eigen::Vector2f(1,0));
                                    // bool larger_than_r = utils::isLargerAngle(goal_vec - cp, cp_r_vec_rot);
                                    // ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);

                                    bool larger_than_l = utils::isLargerAngle(goal_vec, Eigen::Vector2f(1,0));
                                    bool larger_than_r = utils::isLargerAngle(goal_vec, r_new);

                                    // if(!left_side_of_l)
                                    if(!larger_than_l)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        // new_goal = goal_cp_vec.norm() * rbt_orient_vec + cp;
                                        new_goal = goal_vec.norm() * rbt_orient_vec;
                                    }
                                    // else if(left_side_of_l && left_side_of_r)
                                    else if(larger_than_l && larger_than_r)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        // new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
                                        new_goal = goal_vec.norm() * r_new / r_new.norm();
                                    }
                                    // else if(left_side_of_l && !left_side_of_r)
                                    else if(larger_than_l && !larger_than_r)
                                    {
                                        new_goal = goal_vec;
                                    }
                                    else
                                    {
                                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap 2 l side]");
                                        success = false;
                                    }
                                }
                            }
                            else
                            {
                                // Eigen::Vector2f cp_l_vec = l_vec - cp;
                                // Eigen::Vector2f cp_l_vec_rot = utils::getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                                // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                                
                                // ROS_INFO_STREAM("Within 2 r side: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1]);
                                
                                Eigen::Vector2f l_used_vec = other_inter;
                                Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);

                                Eigen::Vector2f l_new = l_used_vec + robot_geo_thresh_dist * l_normal_vec / l_normal_vec.norm();

                                ROS_DEBUG_STREAM("Within 2 r side: " << cp[0] << " " << cp[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1]);

                                // if(utils::isLeftofLine(origin, cp, l_new_vec))
                                // if(utils::isLargerAngle(cp_l_vec_rot, Eigen::Vector2f(1,0)))
                                if(utils::isLargerAngle(l_new, Eigen::Vector2f(1,0)))
                                {
                                    success = false;
                                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 2 r side]");
                                }
                                else
                                {

                                    // bool left_side_of_l = utils::isLeftofLine(cp, l_new_vec, goal_vec);
                                    // bool left_side_of_r = utils::isLeftofLine(origin, cp, goal_vec);
                                    // bool larger_than_l = utils::isLargerAngle(goal_vec - cp, cp_l_vec_rot);
                                    // bool larger_than_r = utils::isLargerAngle(goal_vec - cp, Eigen::Vector2f(1,0));
                                    // ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);
                                    bool larger_than_l = utils::isLargerAngle(goal_vec, l_new);
                                    bool larger_than_r = utils::isLargerAngle(goal_vec, Eigen::Vector2f(1,0));

                                    // if(!left_side_of_l)
                                    if(!larger_than_l)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        // new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                                        new_goal = goal_vec.norm() * l_new / l_new.norm();
                                    }
                                    // else if(left_side_of_l && left_side_of_r)
                                    else if(larger_than_l && larger_than_r)
                                    {
                                        // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        // new_goal = goal_cp_vec.norm() * rbt_orient_vec + cp;
                                        new_goal = goal_vec.norm() * rbt_orient_vec;
                                    }
                                    // else if(left_side_of_l && !left_side_of_r)
                                    else if(larger_than_l && !larger_than_r)
                                    {
                                        new_goal = goal_vec;
                                    }
                                    else
                                    {
                                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap 2 r side]");
                                        success = false;
                                    }
                                }
                            }
                            
                        }
                    }
                    else // dist to other inter < thresh
                    {
                        ROS_WARN_STREAM("Distances to the both intersections are smaller than diagonal / 2.");
                        success = false;
                    }
                }
            }
            else // the closest inter point has angle larger than pi/2
            {
                cp = ideal_min_cp_length * rbt_orient_vec;

                success = true;

                // Goal point region
                // Eigen::Vector2f cp_l_vec = l_vec - cp;
                // Eigen::Vector2f cp_l_vec_rot = utils::getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                // Eigen::Vector2f cp_r_vec = r_vec - cp;
                // Eigen::Vector2f cp_r_vec_rot = utils::getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                
                // ROS_INFO_STREAM("Within 1 larger: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);

                Eigen::Vector2f l_used_vec = chosen_inter;
                Eigen::Vector2f r_used_vec = other_inter;
                if(!l_side)
                {
                    l_used_vec = other_inter;
                    r_used_vec = chosen_inter;
                }

                Eigen::Vector2f l_normal_vec(-l_used_vec[1], l_used_vec[0]);
                Eigen::Vector2f r_normal_vec(r_used_vec[1], -r_used_vec[0]);

                Eigen::Vector2f l_new = l_used_vec + robot_geo_thresh_dist * l_normal_vec / l_normal_vec.norm();
                Eigen::Vector2f r_new = r_used_vec + robot_geo_thresh_dist * r_normal_vec / r_normal_vec.norm();

                ROS_DEBUG_STREAM("Within 1 larger: " << cp[0] << " " << cp[1] << " " << l_used_vec[0] << " " << l_used_vec[1] << " " << l_new[0] << " " << l_new[1] << " " << r_used_vec[0] << " " << r_used_vec[1] << " " << r_new[0] << " " << r_new[1]);

                // if(!utils::isLeftofLine(cp, l_new_vec, r_new_vec))
                // if(!utils::isLargerAngle(cp_r_vec_rot, cp_l_vec_rot))
                if(!utils::isLargerAngle(r_new, l_new))
                {
                    success = false;
                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap larger pi/2]");
                }
                else
                {

                    // bool left_side_of_l = utils::isLeftofLine(cp, l_new_vec, goal_vec);
                    // bool left_side_of_r = utils::isLeftofLine(cp, r_new_vec, goal_vec);
                    // bool larger_than_l = utils::isLargerAngle(goal_vec - cp, cp_l_vec_rot);
                    // bool larger_than_r = utils::isLargerAngle(goal_vec - cp, cp_r_vec_rot);
                    bool larger_than_l = utils::isLargerAngle(goal_vec, l_new);
                    bool larger_than_r = utils::isLargerAngle(goal_vec, r_new);

                    // if(!left_side_of_l)
                    if(!larger_than_l)
                    {
                        // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        // new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                        new_goal = goal_vec.norm() * l_new / l_new.norm();
                    }
                    // else if(left_side_of_l && left_side_of_r)
                    else if(larger_than_l && larger_than_r)
                    {
                        // Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        // new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
                        new_goal = goal_vec.norm() * r_new / r_new.norm();
                    }
                    // else if(left_side_of_l && !left_side_of_r)
                    else if(larger_than_l && !larger_than_r)
                    {
                        new_goal = goal_vec;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is within gap larger pi/2");
                        success = false;
                    }
                }
            }
        }
        else // orientation is not within gap triangle
        {
            
            if(ideal_min_cp_length > cp_max_length)
                cp = cp_max_length * rbt_orient_vec;
            else
                cp = ideal_min_cp_length * rbt_orient_vec;

            // By default, we will calculate r side geo
            Eigen::Vector2f cp_inter_vec = chosen_inter - cp;
            success = true;

            if(!l_side)
            {
                Eigen::Vector2f cp_inter_normal_vec(cp_inter_vec[1], -cp_inter_vec[0]);
                Eigen::Vector2f cp_new_inter_vec = robot_geo_thresh_dist * cp_inter_normal_vec / cp_inter_normal_vec.norm() + cp_inter_vec;
                Eigen::Vector2f new_inter_vec = cp_new_inter_vec + cp;

                // Eigen::Vector2f cp_l_vec = l_vec - cp;
                // Eigen::Vector2f cp_l_vec_rot = utils::getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                // Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                Eigen::Vector2f l_used = other_inter;
                Eigen::Vector2f l_inter_normal_vec(-l_used[1], l_used[0]);
                Eigen::Vector2f l_new_inter_vec = robot_geo_thresh_dist * l_inter_normal_vec / l_inter_normal_vec.norm() + l_used;

                ROS_DEBUG_STREAM("Not within r side: " << cp[0] << " " << cp[1] << " " << chosen_inter[0] << " " << chosen_inter[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << l_used[0] << " " << l_used[1] << " " << l_new_inter_vec[0] << " " << l_new_inter_vec[1]);
                
                // if(!utils::isLeftofLine(cp, l_new_vec, cp + cp_new_inter_vec))

                // Get intersection point p1 = (0, 0), p2 = l_new_inter_vec, p3 = cp, p4 = new_inter_vec
                float denominator = (0. - l_new_inter_vec[0]) * (cp[1] - new_inter_vec[1]) - (0. - l_new_inter_vec[1]) * (cp[0] - new_inter_vec[0]);
                float nom_x = 0. - (0. - l_new_inter_vec[0]) * (cp[0] * new_inter_vec[1] - cp[1] * new_inter_vec[0]);
                float nom_y = 0. - (0. - l_new_inter_vec[1]) * (cp[0] * new_inter_vec[1] - cp[1] * new_inter_vec[0]);

                bool has_inter = true;
                float inter_x, inter_y;
                if(denominator == 0)
                    has_inter = false;
                else
                {
                    inter_x = nom_x / denominator;
                    inter_y = nom_y / denominator;
                }

                bool inter_left_to_cp = utils::isLeftofLine(Eigen::Vector2f(0, 0), cp, Eigen::Vector2f(inter_x, inter_y));
                bool left_to_cp_line = utils::isLeftofLine(Eigen::Vector2f(0, 0), cp, goal_vec);
                bool left_to_inter_line = utils::isLeftofLine(Eigen::Vector2f(inter_x, inter_y), Eigen::Vector2f(inter_x, inter_y) + cp, goal_vec);
                bool left_to_cp_inter = utils::isLeftofLine(cp, new_inter_vec, goal_vec);
                bool left_to_new_inter = utils::isLeftofLine(Eigen::Vector2f(0, 0), l_new_inter_vec, goal_vec);

                if(!has_inter || (has_inter && inter_left_to_cp) || (has_inter && !inter_left_to_cp && left_to_inter_line))
                {
                    if(left_to_cp_line || (!left_to_cp_line && left_to_cp_inter))
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + cp;
                    }
                    else if(!left_to_cp_line && !left_to_new_inter)
                    {
                        new_goal = goal_vec.norm() * l_new_inter_vec / l_new_inter_vec.norm();
                    }
                    else if(!left_to_cp_line && left_to_new_inter && !left_to_cp_inter)
                    {
                        new_goal = goal_vec;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap r side, parallel]");
                        success = false;
                    }
                }
                else if((has_inter && !inter_left_to_cp && !left_to_inter_line))
                {
                    new_goal = Eigen::Vector2f(inter_x, inter_y);
                }
                else
                {
                    ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap r side, not desired]");
                    success = false;
                }
                
            }
            else
            {
                Eigen::Vector2f cp_inter_normal_vec(-cp_inter_vec[1], cp_inter_vec[0]);
                Eigen::Vector2f cp_new_inter_vec = robot_geo_thresh_dist * cp_inter_normal_vec / cp_inter_normal_vec.norm() + cp_inter_vec;
                Eigen::Vector2f new_inter_vec = cp_new_inter_vec + cp;

                // Eigen::Vector2f cp_r_vec = r_vec - cp;
                // Eigen::Vector2f cp_r_vec_rot = utils::getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                // Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                Eigen::Vector2f r_used = other_inter;
                Eigen::Vector2f r_inter_normal_vec(r_used[1], -r_used[0]);
                Eigen::Vector2f r_new_inter_vec = robot_geo_thresh_dist * r_inter_normal_vec / r_inter_normal_vec.norm() + r_used;

                ROS_DEBUG_STREAM("Not within l side: " << cp[0] << " " << cp[1] << " " << chosen_inter[0] << " " << chosen_inter[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << r_used[0] << " " << r_used[1] << " " << r_new_inter_vec[0] << " " << r_new_inter_vec[1]);
                // if(!utils::isLeftofLine(cp, cp + cp_new_inter_vec, r_new_vec))
                // Get intersection point p1 = (0, 0), p2 = r_new_inter_vec, p3 = cp, p4 = new_inter_vec
                float denominator = (0. - r_new_inter_vec[0]) * (cp[1] - new_inter_vec[1]) - (0. - r_new_inter_vec[1]) * (cp[0] - new_inter_vec[0]);
                float nom_x = 0. - (0. - r_new_inter_vec[0]) * (cp[0] * new_inter_vec[1] - cp[1] * new_inter_vec[0]);
                float nom_y = 0. - (0. - r_new_inter_vec[1]) * (cp[0] * new_inter_vec[1] - cp[1] * new_inter_vec[0]);

                bool has_inter = true;
                float inter_x, inter_y;
                if(denominator == 0)
                    has_inter = false;
                else
                {
                    inter_x = nom_x / denominator;
                    inter_y = nom_y / denominator;
                }

                bool inter_left_to_cp = utils::isLeftofLine(Eigen::Vector2f(0, 0), cp, Eigen::Vector2f(inter_x, inter_y));
                bool left_to_cp_line = utils::isLeftofLine(Eigen::Vector2f(0, 0), cp, goal_vec);
                bool left_to_inter_line = utils::isLeftofLine(Eigen::Vector2f(inter_x, inter_y), Eigen::Vector2f(inter_x, inter_y) + cp, goal_vec);
                bool left_to_cp_inter = utils::isLeftofLine(cp, new_inter_vec, goal_vec);
                bool left_to_new_inter = utils::isLeftofLine(Eigen::Vector2f(0, 0), r_new_inter_vec, goal_vec);

                if(!has_inter || (has_inter && !inter_left_to_cp) || (has_inter && inter_left_to_cp && !left_to_inter_line))
                {
                    if(!left_to_cp_line || (left_to_cp_line && !left_to_cp_inter))
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + cp;
                    }
                    else if(left_to_cp_line && left_to_new_inter)
                    {
                        new_goal = goal_vec.norm() * r_new_inter_vec / r_new_inter_vec.norm();
                    }
                    else if(left_to_cp_line && !left_to_new_inter && left_to_cp_inter)
                    {
                        new_goal = goal_vec;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap l side, parallel]");
                        success = false;
                    }
                }
                else if((has_inter && inter_left_to_cp && left_to_inter_line))
                {
                    new_goal = Eigen::Vector2f(inter_x, inter_y);
                }
                else
                {
                    ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap l side, not desired]");
                    success = false;
                }
            }
            
        }

        if(success)
        {
            bezier_curve = Bezier::Bezier<2>({ {0, 0}, {cp[0], cp[1]}, 
                                          {new_goal[0], new_goal[1]} });
        }

        return success;
    }

    geometry_msgs::PoseArray GapTrajGenerator::generateBezierTrajectory(potential_gap::Gap selectedGap, nav_msgs::Odometry curr_odom, geometry_msgs::TransformStamped odom2rbt)
    {
        geometry_msgs::PoseArray posearr;
        posearr.header.stamp = ros::Time::now();
        
        // posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;
        posearr.header.frame_id = cfg_->robot_frame_id;

        if (selectedGap.goal.discard) {
            ROS_WARN_STREAM("This waypoint is discard.");
            return posearr;
        }

        Bezier::Bezier<2> qudraBezier;
        bool success = findBezierControlPts(selectedGap, qudraBezier, curr_odom, odom2rbt);
        
        if(!success)
        {
            ROS_WARN_STREAM("No path is generated.");
            return posearr;
        }
        else
        {
            if(!cfg_->traj.bezier_interp)
            {
                for(float t = 0; t <= 1; t+=0.02)
                {
                    geometry_msgs::Pose pose;
                    pose.position.x = qudraBezier.valueAt(t, 0);
                    pose.position.y = qudraBezier.valueAt(t, 1);
                    pose.orientation = getTang2Quat(qudraBezier.tangentAt(t));
                    posearr.poses.push_back(pose);
                }
                return posearr;
            }
            else
            {
                interpBezierTraj<2>(qudraBezier, posearr.poses);
                return posearr;
            }
        }
    }

    geometry_msgs::PoseArray GapTrajGenerator::genMultiBezierTrajs(potential_gap::Gap selectedGap, nav_msgs::Odometry curr_odom)
    {
        geometry_msgs::PoseArray posearr;
        posearr.header.stamp = ros::Time::now();
        
        // posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;
        posearr.header.frame_id = cfg_->robot_frame_id;

        if (selectedGap.goal.discard) {
            ROS_WARN_STREAM("This waypoint is discard.");
            return posearr;
        }
        Eigen::Vector2f goal_vec(selectedGap.goal.x, selectedGap.goal.y);
        bool goal_within = false;
        if(goal_vec.norm() <= selectedGap.inf_gap.inf_min_safe_dist)
            goal_within = true;

        bool local_goal_found = true;
        Eigen::Vector2f local_goal = goal_vec;
        bool local_goal_within = false;
        if(!goal_within)
        {
            local_goal_found = regulateLocalGoal(selectedGap, local_goal);
            if(local_goal.norm() <= selectedGap.inf_gap.inf_min_safe_dist)
                local_goal_within = true;
        }

        Bezier::Bezier<3> init_bezier;
        Bezier::Bezier<2> goal_bezier;
        bool init_suc = false, goal_suc = false;

        // ROS_INFO_STREAM("local goal: " << local_goal);

        Eigen::Vector2f cp2;

        if((goal_within || local_goal_within) && local_goal_found)
        {
            ROS_INFO_STREAM("Goal is within inflated circle.");
            init_suc = robotInitBezierCurve(selectedGap, local_goal, init_bezier, cp2, curr_odom);
            goal_suc = true;
        }
        else if(!goal_within && !local_goal_within && local_goal_found)
        {
            Eigen::Vector2f circ_pt = getCircPt(selectedGap, local_goal);
            // ROS_INFO_STREAM("circ: " << circ_pt);
            init_suc = robotInitBezierCurve(selectedGap, circ_pt, init_bezier, cp2, curr_odom);
            goal_suc = robotGoalBezierCurve(selectedGap, circ_pt, cp2, local_goal, goal_bezier, curr_odom);
        }
        else
        {
            ROS_WARN_STREAM("No local goal is found.");
            return posearr;
        }
        
        bool success = init_suc && goal_suc;
        
        std::vector<geometry_msgs::Pose> init_poses;
        std::vector<geometry_msgs::Pose> goal_poses;

        if(!success)
        {
            ROS_WARN_STREAM("No path is generated.");
            return posearr;
        }
        else
        {
            if(!cfg_->traj.bezier_interp)
            {
                for(float t = 0; t <= 1; t+=0.02)
                {
                    geometry_msgs::Pose init_pose;
                    init_pose.position.x = init_bezier.valueAt(t, 0);
                    init_pose.position.y = init_bezier.valueAt(t, 1);
                    init_pose.orientation = getTang2Quat(init_bezier.tangentAt(t));
                    init_poses.push_back(init_pose);

                    if(!goal_within && !local_goal_within)
                    {
                        geometry_msgs::Pose goal_pose;
                        goal_pose.position.x = goal_bezier.valueAt(t, 0);
                        goal_pose.position.y = goal_bezier.valueAt(t, 1);
                        goal_pose.orientation = getTang2Quat(goal_bezier.tangentAt(t));
                        goal_poses.push_back(goal_pose);
                    }
                }
            }
            else
            {
                interpBezierTraj<3>(init_bezier, init_poses);
                if(!goal_within && !local_goal_within)
                    interpBezierTraj<2>(goal_bezier, goal_poses);
            }

            if(init_poses.size() < 1 || (goal_poses.size() < 1 && !goal_within && !local_goal_within))
            {
                ROS_ERROR_STREAM("The bezier poses have less than 2 poses. Should not happen. " << init_poses.size() << " " << goal_poses.size());
                return posearr;
            }

            posearr.poses = init_poses;
            if(!goal_within && !local_goal_within)
                posearr.poses.insert(posearr.poses.end(), goal_poses.begin() + 1, goal_poses.end());

            return posearr;
        }
    }

    bool GapTrajGenerator::regulateLocalGoal(const potential_gap::Gap& selectedGap, Eigen::Vector2f& reg_goal)
    {
        if(!selectedGap.mode.inflated)
        {
            ROS_WARN_STREAM("[regulateLocalGoal] The gap is NOT inflated.");
            return false;
        }

        Eigen::Vector2f raw_goal_vec(selectedGap.goal.x, selectedGap.goal.y);

        Eigen::Vector2f l_int_vec = selectedGap.inf_gap.inf_l - selectedGap.inf_gap.inf_lgc_int;
        Eigen::Vector2f r_int_vec = selectedGap.inf_gap.inf_r - selectedGap.inf_gap.inf_rgc_int;

        Eigen::Vector2f goal_lint_vec = raw_goal_vec - selectedGap.inf_gap.inf_lgc_int;
        Eigen::Vector2f goal_rint_vec = raw_goal_vec - selectedGap.inf_gap.inf_rgc_int;

        bool left_to_l = utils::isLeftofLine(selectedGap.inf_gap.inf_lgc_int, selectedGap.inf_gap.inf_l, raw_goal_vec);
        bool left_to_r = utils::isLeftofLine(selectedGap.inf_gap.inf_rgc_int, selectedGap.inf_gap.inf_r, raw_goal_vec);
        bool left_to_nuls = utils::isLeftofLine(selectedGap.inf_gap.inf_lgc_int, selectedGap.inf_gap.inf_rgc_int, raw_goal_vec);

        Eigen::Vector2f gap_mid_pt = (selectedGap.inf_gap.lgap + selectedGap.inf_gap.rgap) / 2;
        if(left_to_nuls)
        {
            reg_goal = gap_mid_pt;
        }
        else
        {
            if(!left_to_l)
            {
                reg_goal = goal_lint_vec.norm() * l_int_vec / l_int_vec.norm();
                reg_goal += selectedGap.inf_gap.inf_lgc_int;
                if(utils::isLeftofLine(selectedGap.inf_gap.inf_rgc_int, selectedGap.inf_gap.inf_r, reg_goal))
                {
                    // reg_goal = selectedGap.inf_gap.inf_l;
                    reg_goal = gap_mid_pt;
                }
            }
            else if(left_to_l && left_to_r)
            {
                reg_goal = goal_rint_vec.norm() * r_int_vec / r_int_vec.norm();
                reg_goal += selectedGap.inf_gap.inf_rgc_int;
                if(!utils::isLeftofLine(selectedGap.inf_gap.inf_lgc_int, selectedGap.inf_gap.inf_l, reg_goal))
                {
                    // reg_goal = selectedGap.inf_gap.inf_r;
                    reg_goal = gap_mid_pt;
                }
            }
            else if(left_to_l && !left_to_r)
            {
                reg_goal = raw_goal_vec;
            }
            else
            {
                ROS_WARN_STREAM("[robotGoalBezierCurve] Goal point is in the wrong region. Should not happen.");
                return false;
            }
        }

        return true;
    }

    Eigen::Vector2f GapTrajGenerator::getCircPt(const potential_gap::Gap& selectedGap, const Eigen::Vector2f& local_goal)
    {
        // Default solution is choosing the middle point on the arc
        float angl = atan2(selectedGap.inf_gap.inf_lgc_int(1), selectedGap.inf_gap.inf_lgc_int(0));
        float angr = atan2(selectedGap.inf_gap.inf_rgc_int(1), selectedGap.inf_gap.inf_rgc_int(0));
        angl = angl <= angr ? angl : angl - 2 * M_PI;
        float mid_ang = (angl + angr) / 2;
        Eigen::Vector2f mid(selectedGap.inf_gap.inf_min_safe_dist * cos(mid_ang), selectedGap.inf_gap.inf_min_safe_dist * sin(mid_ang));
        float mid_ang_reg = atan2(mid(1), mid(0));
        // ROS_INFO_STREAM("Raw mid: " << mid);
        float goal_ang = atan2(local_goal(1), local_goal(0));

        float orient_ang = 0.;
        bool orient_in_between = (angl < orient_ang) && (angr > orient_ang);
        float side_buffer = 0.2 * abs(angr - angl);
        float l_side_ang = angl + side_buffer;
        float r_side_ang = angr - side_buffer;
        Eigen::Vector2f l_side(selectedGap.inf_gap.inf_min_safe_dist * cos(l_side_ang), selectedGap.inf_gap.inf_min_safe_dist * sin(l_side_ang));
        Eigen::Vector2f r_side(selectedGap.inf_gap.inf_min_safe_dist * cos(r_side_ang), selectedGap.inf_gap.inf_min_safe_dist * sin(r_side_ang));

        Eigen::Vector2f cir_pt;

        if(!orient_in_between)
        {
            bool mid_left = mid_ang_reg > orient_ang;
            bool bia_direc = utils::isLeftofLine(Eigen::Vector2f(0, 0), mid, local_goal);
            bool need_bia = mid_left ? !bia_direc : bia_direc;

            if(mid_left && !bia_direc)
            {
                Eigen::Vector2f min_pt = utils::minDistancePt(utils::Point(mid), utils::Point(l_side), utils::Point(local_goal));
                cir_pt = selectedGap.inf_gap.inf_min_safe_dist * min_pt / min_pt.norm();
            }
            else if(!mid_left && bia_direc)
            {
                Eigen::Vector2f min_pt = utils::minDistancePt(utils::Point(mid), utils::Point(r_side), utils::Point(local_goal));
                cir_pt = selectedGap.inf_gap.inf_min_safe_dist * min_pt / min_pt.norm();
            }
            else
            {
                cir_pt = mid;
            }
        }
        else
        {
            int max_it = 5;
            int i = 0;

            Eigen::Vector2f cand_pt = mid;
            float cand_ang = mid_ang_reg;
            Eigen::Vector2f cand_l_side = l_side;
            Eigen::Vector2f cand_r_side = r_side;

            while(i < max_it)
            {
                bool cand_left = cand_ang > orient_ang;
                bool bia_direc = utils::isLeftofLine(Eigen::Vector2f(0, 0), cand_pt, local_goal);

                if(cand_left && !bia_direc)
                {
                    Eigen::Vector2f min_pt = utils::minDistancePt(utils::Point(cand_pt), utils::Point(cand_l_side), utils::Point(local_goal));
                    cand_pt = selectedGap.inf_gap.inf_min_safe_dist * min_pt / min_pt.norm();
                    cand_r_side = cand_pt;
                }
                else if(!cand_left && bia_direc)
                {
                    Eigen::Vector2f min_pt = utils::minDistancePt(utils::Point(cand_pt), utils::Point(cand_r_side), utils::Point(local_goal));
                    cand_pt = selectedGap.inf_gap.inf_min_safe_dist * min_pt / min_pt.norm();
                    cand_l_side = cand_pt;
                }
                else
                {
                    cir_pt = cand_pt;
                    break;
                }
                cand_ang = atan2(cand_pt(1), cand_pt(0));

                cir_pt = cand_pt;
                i++;
            }
        }

        Eigen::Vector2f cir_goal_vec = local_goal - cir_pt;
        float goal_cir_ang = acos(cir_goal_vec.dot(cir_pt) / (cir_goal_vec.norm() * cir_pt.norm()));
        if(goal_cir_ang < 100. / 180. * M_PI)
        {
            bool goal_left_cir = utils::isLeftofLine(Eigen::Vector2f(0, 0), cir_pt, local_goal);
            Eigen::Vector2f side_pt = l_side;
            if(goal_left_cir)
                side_pt = r_side;

            Eigen::Vector2f min_pt = utils::minDistancePt(utils::Point(cir_pt), utils::Point(side_pt), utils::Point(local_goal));
            cir_pt = selectedGap.inf_gap.inf_min_safe_dist * min_pt / min_pt.norm();
        }

        // float mid_goal_ang_diff = acos(mid.dot(local_goal) / (mid.norm() * local_goal.norm()));
        // float bia_angle_range = abs(angr - angl) / 2 - side_buffer;
        // float bia_ang = mid_goal_ang_diff / (M_PI / 4) * bia_angle_range;
        // bia_ang = bia_ang > bia_angle_range ? bia_angle_range : bia_ang;
        // bia_ang = bia_direc ? bia_ang : -bia_ang;

        // float cir_ang = mid_ang + bia_ang;
        // Eigen::Vector2f cir_pt(selectedGap.inf_gap.inf_min_safe_dist * cos(cir_ang), selectedGap.inf_gap.inf_min_safe_dist * sin(cir_ang));

        int circ_pt_m = 1;
        switch (circ_pt_m)
        {
        case 0:
            return mid;
        case 1:
            return cir_pt;
        default:
            return mid;
        }
    }

    bool GapTrajGenerator::robotInitBezierCurve(potential_gap::Gap selectedGap, const Eigen::Vector2f& circ_pt, Bezier::Bezier<3>& init_bezier, Eigen::Vector2f& cp2, nav_msgs::Odometry curr_odom)
    {
        if(!selectedGap.mode.inflated)
        {
            ROS_WARN_STREAM("[robotInitBezierCurve] The gap is NOT inflated.");
            return false;
        }

        Eigen::Vector2f rbt_orient_vec(1, 0);
        float x_speed = curr_odom.twist.twist.linear.x;
        x_speed = x_speed < 0 ? 0. : x_speed;
        float scale_factor = circ_pt.norm() / robot_geo_proc_.getRobotAvgLinSpeed();

        float p1_length = x_speed * scale_factor / 3; // For cubic bezier curve, the ideally length B'(0) = 3 (P_1 - P_0) / scale_factor

        float d_w = getDw(rbt_orient_vec, circ_pt, circ_pt.norm(), robot_geo_proc_.getRobotAvgLinSpeed());
        Eigen::Vector2f est_a = estAcc(robot_geo_proc_.getRobotAvgLinSpeed(), d_w);

        Eigen::Vector2f cp0(0, 0);
        Eigen::Vector2f cp1 = p1_length * rbt_orient_vec;
        cp2 = scale_factor * scale_factor / 6 * est_a - cp0 + 2 * cp1; // For cubic bezier curve, B''(0) = 6 (P_2 - 2 P_1 + P_0) / scale_factor^2
        Eigen::Vector2f cp3 = circ_pt;

        init_bezier = Bezier::Bezier<3>({ {cp0(0), cp0(1)}, {cp1(0), cp1(1)}, 
                                          {cp2(0), cp2(1)}, {cp3(0), cp3(1)} });

        return true;
    }

    bool GapTrajGenerator::robotGoalBezierCurve(potential_gap::Gap selectedGap, const Eigen::Vector2f& circ_pt, const Eigen::Vector2f& prev_cp2, const Eigen::Vector2f& local_goal, Bezier::Bezier<2>& goal_bezier, nav_msgs::Odometry curr_odom)
    {
        if(!selectedGap.mode.inflated)
        {
            ROS_WARN_STREAM("[robotGoalBezierCurve] The gap is NOT inflated.");
            return false;
        }

        // Assign goal within the gap region
        // Eigen::Vector2f l_int_vec = selectedGap.inf_gap.inf_l - selectedGap.inf_gap.inf_gap_int;
        // Eigen::Vector2f r_int_vec = selectedGap.inf_gap.inf_r - selectedGap.inf_gap.inf_gap_int;

        // Eigen::Vector2f goal_int_vec = raw_goal_vec - selectedGap.inf_gap.inf_gap_int;

        // bool larger_than_l = utils::isLargerAngle(goal_int_vec, l_int_vec);
        // bool larger_than_r = utils::isLargerAngle(goal_int_vec, r_int_vec);

        // Eigen::Vector2f goal_vec;

        // if(!larger_than_l)
        // {
        //     goal_vec = goal_int_vec.norm() * l_int_vec / l_int_vec.norm();
        // }
        // else if(larger_than_l && larger_than_r)
        // {
        //     goal_vec = goal_int_vec.norm() * r_int_vec / r_int_vec.norm();
        // }
        // else if(larger_than_l && !larger_than_r)
        // {
        //     goal_vec = goal_int_vec;
        // }
        // else
        // {
        //     ROS_WARN_STREAM("[robotGoalBezierCurve] Goal point is in the wrong region. Should not happen.");
        //     return false;
        // }

        // goal_vec += selectedGap.inf_gap.inf_gap_int;

        Eigen::Vector2f circ_goal_vec = local_goal - circ_pt;

        float scale_factor = circ_goal_vec.norm() / robot_geo_proc_.getRobotAvgLinSpeed();
        Eigen::Vector2f orient_vec = circ_pt - prev_cp2;
        orient_vec /= orient_vec.norm();
        Eigen::Vector2f raw_cp1 = robot_geo_proc_.getRobotAvgLinSpeed() * scale_factor / 2 * orient_vec + circ_pt;

        Eigen::Vector2f cp1;
        bool is_int_l = utils::doIntersect(utils::Point(circ_pt), utils::Point(raw_cp1), utils::Point(selectedGap.inf_gap.inf_lgc_int), utils::Point(selectedGap.inf_gap.inf_l));
        
        if(is_int_l)
        {
            // ROS_INFO_STREAM("is_int_l");
            cp1 = utils::intersectLines(circ_pt, raw_cp1, selectedGap.inf_gap.inf_lgc_int, selectedGap.inf_gap.inf_l);
        }
        else
        {
            bool is_int_r = utils::doIntersect(utils::Point(circ_pt), utils::Point(raw_cp1), utils::Point(selectedGap.inf_gap.inf_rgc_int), utils::Point(selectedGap.inf_gap.inf_r));
            if(is_int_r)
            {
                // ROS_INFO_STREAM("is_int_r");
                cp1 = utils::intersectLines(circ_pt, raw_cp1, selectedGap.inf_gap.inf_rgc_int, selectedGap.inf_gap.inf_r);
            }
            else
            {
                cp1 = raw_cp1;
            }
        }

        if(utils::is_inf(cp1))
            return false;
        // ROS_INFO_STREAM("gap: " << selectedGap.inf_gap.inf_lgc_int << " " << selectedGap.inf_gap.inf_l << " " << selectedGap.inf_gap.inf_rgc_int << " " << selectedGap.inf_gap.inf_r << " " << selectedGap.inf_gap.inf_min_safe_dist);
        // ROS_INFO_STREAM("goal: " << prev_cp2 << " " << circ_pt << " " << raw_cp1 << " " << cp1);
        goal_bezier = Bezier::Bezier<2>({ {circ_pt(0), circ_pt(1)}, {cp1(0), cp1(1)}, 
                                          {local_goal(0), local_goal(1)} });

        return true;
    }
    
    template<int N>
    void GapTrajGenerator::interpBezierTraj(const Bezier::Bezier<N>& bezier_path, std::vector<geometry_msgs::Pose>& interp_pose)
    {
        double des_dist = robot_geo_proc_.getRobotAvgLinSpeed() * cfg_->traj.bezier_unit_time;
        double entire_dist = getBezierDist<N>(bezier_path, 0, 1, 30);
        int num_sampled_pts = int(round(entire_dist / des_dist));
        num_sampled_pts = num_sampled_pts >= 2 ? num_sampled_pts : 2;

        double dist_thresh = des_dist / 10;
        double t_step = 1. / (num_sampled_pts - 1);
        double t_min = 0;
        for(size_t i = 0; i < num_sampled_pts; i++)
        {
            double cur_t = i * t_step;
            double cur_dist = getBezierDist<N>(bezier_path, t_min, cur_t, 5);
            if(abs(cur_dist - des_dist) < dist_thresh)
            {
                geometry_msgs::Pose pose;
                pose.position.x = bezier_path.valueAt(cur_t, 0);
                pose.position.y = bezier_path.valueAt(cur_t, 1);
                pose.orientation = getTang2Quat(bezier_path.tangentAt(cur_t));
                interp_pose.push_back(pose);
                t_min = cur_t;
            }
            else if(cur_dist > des_dist)
            {
                double t_prev = (i - 1) * t_step;
                double t_interp = (cur_t + t_prev) / 2;
                double interp_dist = getBezierDist<N>(bezier_path, t_min, t_interp, 5);

                double t_high = cur_t;
                double t_low = t_prev;
                while(abs(interp_dist - des_dist) > dist_thresh)
                {
                    if(interp_dist < des_dist)
                    {
                        t_low = t_interp;
                        t_interp = (t_interp + t_high) / 2;
                    }
                    else
                    {
                        t_high = t_interp;
                        t_interp = (t_interp + t_low) / 2;
                    }
                    interp_dist = getBezierDist<N>(bezier_path, t_min, t_interp, 5);
                    if(abs(t_interp - t_low) <= 1e-3 && abs(t_interp - t_high) <= 1e-3)
                        break;
                    // ROS_INFO_STREAM(t_interp << " " << t_low << " " << t_high << " " << interp_dist << " " << abs(interp_dist - des_dist) << " " << dist_thresh);
                }
                // ROS_INFO_STREAM("exit");
                geometry_msgs::Pose pose;
                pose.position.x = bezier_path.valueAt(t_interp, 0);
                pose.position.y = bezier_path.valueAt(t_interp, 1);
                pose.orientation = getTang2Quat(bezier_path.tangentAt(t_interp));
                interp_pose.push_back(pose);
                t_min = t_interp;
            }
        }
        if(interp_pose.size() < num_sampled_pts)
        {
            geometry_msgs::Pose pose;
            pose.position.x = bezier_path.valueAt(1, 0);
            pose.position.y = bezier_path.valueAt(1, 1);
            pose.orientation = getTang2Quat(bezier_path.tangentAt(1));
            interp_pose.push_back(pose);
        }
        return;
    }

    template void GapTrajGenerator::interpBezierTraj<2>(const Bezier::Bezier<2>& bezier_path, std::vector<geometry_msgs::Pose>& interp_pose);
    template void GapTrajGenerator::interpBezierTraj<3>(const Bezier::Bezier<3>& bezier_path, std::vector<geometry_msgs::Pose>& interp_pose);

    geometry_msgs::Quaternion GapTrajGenerator::getTang2Quat(Bezier::Tangent tang)
    {
        double x = tang[0];
        double y = tang[1];

        double yaw = atan2(y, x);

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

    geometry_msgs::PoseArray GapTrajGenerator::transformBackTrajectory(
        geometry_msgs::PoseArray posearr,
        geometry_msgs::TransformStamped trans,
        std::string in_frame_id, std::string out_frame_id)
    {
        geometry_msgs::PoseArray retarr;
        geometry_msgs::PoseStamped outplaceholder;
        outplaceholder.header.frame_id = out_frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        inplaceholder.header.frame_id = in_frame_id;
        for (const auto pose : posearr.poses)
        {
            inplaceholder.pose = pose;
            tf2::doTransform(inplaceholder, outplaceholder, trans);
            outplaceholder.pose.position.z = 0;
            retarr.poses.push_back(outplaceholder.pose);
        }
        retarr.header.frame_id = out_frame_id;
        retarr.header.stamp = trans.header.stamp;
        return retarr;
    }

    geometry_msgs::PoseArray GapTrajGenerator::forwardPassTrajectory(geometry_msgs::PoseArray pose_arr)
    {
        Eigen::Quaternionf q;
        geometry_msgs::Pose old_pose;
        old_pose.position.x = 0;
        old_pose.position.y = 0;
        old_pose.position.z = 0;
        old_pose.orientation.x = 0;
        old_pose.orientation.y = 0;
        old_pose.orientation.z = 0;
        old_pose.orientation.w = 1;
        geometry_msgs::Pose new_pose;
        double dx, dy, result;

        std::vector<geometry_msgs::Pose> shortened;
        // shortened.push_back(old_pose);

        if(pose_arr.poses.size() == 0)
        {
            return pose_arr;
        }

        shortened.push_back(pose_arr.poses[0]);
        for (auto pose : pose_arr.poses)
        {
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > 0.05)
                shortened.push_back(pose);
        }

        pose_arr.poses = shortened;

        // Fix rotation
        for (int idx = 1; idx < pose_arr.poses.size(); idx++)
        {
            new_pose = pose_arr.poses[idx];
            old_pose = pose_arr.poses[idx - 1];
            dx = new_pose.position.x - old_pose.position.x;
            dy = new_pose.position.y - old_pose.position.y;
            result = std::atan2(dy, dx);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ());
            q.normalize();
            pose_arr.poses[idx - 1].orientation.x = q.x();
            pose_arr.poses[idx - 1].orientation.y = q.y();
            pose_arr.poses[idx - 1].orientation.z = q.z();
            pose_arr.poses[idx - 1].orientation.w = q.w();
        }
        pose_arr.poses.pop_back();

        return pose_arr;
    }

}
