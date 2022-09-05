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

    [[deprecated("Use single trajectory generation")]]
    std::vector<geometry_msgs::PoseArray> GapTrajGenerator::generateTrajectory(std::vector<potential_gap::Gap> gapset) {
        std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
        return traj_set;
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

        ROS_INFO_STREAM(goal_x << " " << goal_y << " " << selectedGap.goal.goalwithin);

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
        float ideal_min_cp_length = x_speed / 2 * float(cfg_->traj.bezier_cp_scale); // For quadratic bezier curve, the ideally minimum B'(0) = 2 (P_1 - P_0) 

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
            ROS_WARN_STREAM("The circle is smaller than robot diagonal / 2.");
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
                    if(ideal_min_cp_length < circ_r)
                    {
                        float cp_length = circ_r * float(cfg_->traj.bezier_cp_scale);
                        cp = cp_length * rbt_orient_vec;
                    }
                    else
                    {
                        cp = ideal_min_cp_length * rbt_orient_vec;
                    }

                    // Goal point region
                    Eigen::Vector2f cp_l_vec = l_vec - cp;
                    Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                    Eigen::Vector2f cp_r_vec = r_vec - cp;
                    Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                    Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                    Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                    success = true;

                    ROS_INFO_STREAM("Within 1: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);
                    // if(!isLeftofLine(cp, l_new_vec, r_new_vec))
                    if(!isLargerAngle(cp_r_vec_rot, cp_l_vec_rot))
                    {
                        success = false;
                        ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 1]");
                    }
                    else
                    {

                        // bool left_side_of_l = isLeftofLine(cp, l_new_vec, goal_vec);
                        // bool left_side_of_r = isLeftofLine(cp, r_new_vec, goal_vec);
                        bool larger_than_l = isLargerAngle(goal_vec - cp, cp_l_vec_rot);
                        bool larger_than_r = isLargerAngle(goal_vec - cp, cp_r_vec_rot);
                        ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);

                        // if(!left_side_of_l)
                        if(!larger_than_l)
                        {
                            Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                            new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                        }
                        // else if(left_side_of_l && left_side_of_r)
                        else if(larger_than_l && larger_than_r)
                        {
                            Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                            new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
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
                                Eigen::Vector2f cp_r_vec = r_vec - cp;
                                Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                                Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;

                                ROS_INFO_STREAM("Within 2 l side: " << cp[0] << " " << cp[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);
                                // if(!isLeftofLine(origin, cp, r_new_vec))
                                if(!isLargerAngle(cp_r_vec_rot, Eigen::Vector2f(1,0)))
                                {
                                    success = false;
                                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 2 l side]");
                                }
                                else
                                {
                                
                                    // bool left_side_of_l = isLeftofLine(origin, cp, goal_vec);
                                    // bool left_side_of_r = isLeftofLine(cp, r_new_vec, goal_vec);
                                    bool larger_than_l = isLargerAngle(goal_vec - cp, Eigen::Vector2f(1,0));
                                    bool larger_than_r = isLargerAngle(goal_vec - cp, cp_r_vec_rot);
                                    ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);

                                    // if(!left_side_of_l)
                                    if(!larger_than_l)
                                    {
                                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        new_goal = goal_cp_vec.norm() * rbt_orient_vec + cp;
                                    }
                                    // else if(left_side_of_l && left_side_of_r)
                                    else if(larger_than_l && larger_than_r)
                                    {
                                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
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
                                Eigen::Vector2f cp_l_vec = l_vec - cp;
                                Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                                Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                                
                                ROS_INFO_STREAM("Within 2 r side: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1]);

                                // if(isLeftofLine(origin, cp, l_new_vec))
                                if(isLargerAngle(cp_l_vec_rot, Eigen::Vector2f(1,0)))
                                {
                                    success = false;
                                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap 2 r side]");
                                }
                                else
                                {

                                    // bool left_side_of_l = isLeftofLine(cp, l_new_vec, goal_vec);
                                    // bool left_side_of_r = isLeftofLine(origin, cp, goal_vec);
                                    bool larger_than_l = isLargerAngle(goal_vec - cp, cp_l_vec_rot);
                                    bool larger_than_r = isLargerAngle(goal_vec - cp, Eigen::Vector2f(1,0));
                                    ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);

                                    // if(!left_side_of_l)
                                    if(!larger_than_l)
                                    {
                                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                                    }
                                    // else if(left_side_of_l && left_side_of_r)
                                    else if(larger_than_l && larger_than_r)
                                    {
                                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                                        new_goal = goal_cp_vec.norm() * rbt_orient_vec + cp;
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

                // Goal point region
                Eigen::Vector2f cp_l_vec = l_vec - cp;
                Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                Eigen::Vector2f cp_r_vec = r_vec - cp;
                Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;
                Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;
                success = true;
                ROS_INFO_STREAM("Within 1 larger: " << cp[0] << " " << cp[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1] << " " << r_vec[0] << " " << r_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);

                // if(!isLeftofLine(cp, l_new_vec, r_new_vec))
                if(!isLargerAngle(cp_r_vec_rot, cp_l_vec_rot))
                {
                    success = false;
                    ROS_WARN_STREAM("The union region does not exist. [Orientation is within gap larger pi/2]");
                }
                else
                {

                    // bool left_side_of_l = isLeftofLine(cp, l_new_vec, goal_vec);
                    // bool left_side_of_r = isLeftofLine(cp, r_new_vec, goal_vec);
                    bool larger_than_l = isLargerAngle(goal_vec - cp, cp_l_vec_rot);
                    bool larger_than_r = isLargerAngle(goal_vec - cp, cp_r_vec_rot);

                    // if(!left_side_of_l)
                    if(!larger_than_l)
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                    }
                    // else if(left_side_of_l && left_side_of_r)
                    else if(larger_than_l && larger_than_r)
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
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

                Eigen::Vector2f cp_l_vec = l_vec - cp;
                Eigen::Vector2f cp_l_vec_rot = getRotatedVec(cp_l_vec, robot_geo_thresh_dist, true);
                Eigen::Vector2f l_new_vec = cp + cp_l_vec_rot;

                ROS_INFO_STREAM("Not within r side: " << cp[0] << " " << cp[1] << " " << chosen_inter[0] << " " << chosen_inter[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << l_vec[0] << " " << l_vec[1] << " " << l_new_vec[0] << " " << l_new_vec[1]);
                
                // if(!isLeftofLine(cp, l_new_vec, cp + cp_new_inter_vec))
                if(!isLargerAngle(cp_new_inter_vec, cp_l_vec_rot))
                {
                    success = false;
                    ROS_WARN_STREAM("The union region does not exist. [Orientation is not within gap r side]");
                }
                else
                {

                    // bool left_side_of_l = isLeftofLine(cp, l_new_vec, goal_vec);
                    // bool left_side_of_r = isLeftofLine(cp, cp_new_inter_vec, goal_vec);
                    bool larger_than_l = isLargerAngle(goal_vec - cp, cp_l_vec_rot);
                    bool larger_than_r = isLargerAngle(goal_vec - cp, cp_new_inter_vec);
                    ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);
                    // if(!left_side_of_l)
                    if(!larger_than_l)
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_l_vec_rot / cp_l_vec_rot.norm() + cp;
                    }
                    // else if(left_side_of_l && left_side_of_r)
                    else if(larger_than_l && larger_than_r)
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + cp;
                    }
                    else if(larger_than_l && !larger_than_r)
                    {
                        new_goal = goal_vec;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap r side]");
                        success = false;
                    }
                    ROS_INFO_STREAM(goal_vec[0] << " " << goal_vec[1] << " " << new_goal[0] << " " << new_goal[1]);
                }
            }
            else
            {
                Eigen::Vector2f cp_inter_normal_vec(-cp_inter_vec[1], cp_inter_vec[0]);
                Eigen::Vector2f cp_new_inter_vec = robot_geo_thresh_dist * cp_inter_normal_vec / cp_inter_normal_vec.norm() + cp_inter_vec;
                Eigen::Vector2f new_inter_vec = cp_new_inter_vec + cp;

                Eigen::Vector2f cp_r_vec = r_vec - cp;
                Eigen::Vector2f cp_r_vec_rot = getRotatedVec(cp_r_vec, robot_geo_thresh_dist, false);
                Eigen::Vector2f r_new_vec = cp + cp_r_vec_rot;

                ROS_INFO_STREAM("Not within l side: " << cp[0] << " " << cp[1] << " " << chosen_inter[0] << " " << chosen_inter[1] << " " << new_inter_vec[0] << " " << new_inter_vec[1] << " " << r_new_vec[0] << " " << r_new_vec[1]);
                // if(!isLeftofLine(cp, cp + cp_new_inter_vec, r_new_vec))
                if(!isLargerAngle(cp_r_vec_rot, cp_new_inter_vec))
                {
                    success = false;
                    ROS_WARN_STREAM("The union region does not exist. [Orientation is not within gap l side]");
                }
                else
                {

                    // bool left_side_of_l = isLeftofLine(cp, cp_new_inter_vec, goal_vec);
                    // bool left_side_of_r = isLeftofLine(cp, r_new_vec, goal_vec);
                    bool larger_than_l = isLargerAngle(goal_vec - cp, cp_new_inter_vec);
                    bool larger_than_r = isLargerAngle(goal_vec - cp, cp_r_vec_rot);
                    ROS_INFO_STREAM(larger_than_l << " " << larger_than_r);
                    // if(!left_side_of_l)
                    if(!larger_than_l)
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_new_inter_vec / cp_new_inter_vec.norm() + cp;
                    }
                    // else if(left_side_of_l && left_side_of_r)
                    else if(larger_than_l && larger_than_r)
                    {
                        Eigen::Vector2f goal_cp_vec = goal_vec - cp;
                        new_goal = goal_cp_vec.norm() * cp_r_vec_rot / cp_r_vec_rot.norm() + cp;
                    }
                    // else if(left_side_of_l && !left_side_of_r)
                    else if(larger_than_l && !larger_than_r)
                    {
                        new_goal = goal_vec;
                    }
                    else
                    {
                        ROS_WARN_STREAM("Goal point is in the wrong region. [Orientation is not within gap l side]");
                        success = false;
                    }
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
            for(float t = 0; t <= 1; t+=0.02)
            {
                geometry_msgs::Pose pose;
                pose.position.x = qudraBezier.valueAt(t, 0);
                pose.position.y = qudraBezier.valueAt(t, 1);
                posearr.poses.push_back(pose);
            }
            return posearr;
        }
    }

    geometry_msgs::PoseArray GapTrajGenerator::transformBackTrajectory(
        geometry_msgs::PoseArray posearr,
        geometry_msgs::TransformStamped trans)
    {
        geometry_msgs::PoseArray retarr;
        geometry_msgs::PoseStamped outplaceholder;
        // outplaceholder.header.frame_id = cfg_->odom_frame_id;
        outplaceholder.header.frame_id = trans.header.frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        // inplaceholder.header.frame_id = cfg_->robot_frame_id;
        inplaceholder.header.frame_id = trans.child_frame_id;
        for (const auto pose : posearr.poses)
        {
            inplaceholder.pose = pose;
            tf2::doTransform(inplaceholder, outplaceholder, trans);
            retarr.poses.push_back(outplaceholder.pose);
        }
        // retarr.header.frame_id = cfg_->odom_frame_id;
        retarr.header.frame_id = trans.header.frame_id;
        retarr.header.stamp = ros::Time::now();
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
        shortened.push_back(old_pose);
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

    Eigen::Vector2f GapTrajGenerator::getRotatedVec(Eigen::Vector2f orig_vec, float chord_length, bool ccw)
    {
        float r = orig_vec.norm();
        float rotate_angle = acos((2 * r * r - chord_length * chord_length) / (2 * r * r));

        Eigen::Matrix2f rotate_mat;
        if(ccw)
            rotate_mat << cos(rotate_angle), -sin(rotate_angle), sin(rotate_angle), cos(rotate_angle);
        else
            rotate_mat << cos(rotate_angle), sin(rotate_angle), -sin(rotate_angle), cos(rotate_angle);

        Eigen::Vector2f rotated_vec = rotate_mat * orig_vec;

        return rotated_vec;
    }

}