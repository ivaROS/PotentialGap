#include <potential_gap/gap_manip.h>

namespace potential_gap {
    void GapManipulator::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egolock);
        msg = msg_;
        num_of_scan = (int)(msg.get()->ranges.size());
    }

    void GapManipulator::setGapWaypoint(potential_gap::Gap& gap, geometry_msgs::PoseStamped localgoal){
        // TODO: assume there is no idx that will pass 0
        auto half_num_scan = gap.half_scan;
        float x1, x2, y1, y2;
        x1 = (gap.convex.convex_ldist) * cos(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (gap.convex.convex_ldist) * sin(-((float) half_num_scan - gap.convex.convex_lidx) / half_num_scan * M_PI);

        x2 = (gap.convex.convex_rdist) * cos(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (gap.convex.convex_rdist) * sin(-((float) half_num_scan - gap.convex.convex_ridx) / half_num_scan * M_PI);

        Eigen::Vector2f pl(x1, y1);
        Eigen::Vector2f pr(x2, y2);
        
        // if agc. then the shorter side need to be further in
        
        // Get the equivalent passing length
        Eigen::Vector2d orient_vec(1, 0);
        Eigen::Vector2d m_pt_vec = (pl.cast<double>() + pr.cast<double>()) / 2;
        // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
        auto lr = (pr - pl) / (pr - pl).norm() * (epl / 2) * cfg_->traj.inf_ratio + pl;
        auto thetalr = utils::car2pol(lr)(1);
        auto rl = (pl - pr) / (pl - pr).norm() * (epl / 2) * cfg_->traj.inf_ratio + pr;
        auto thetarl = utils::car2pol(rl)(1);

        // if(pl[1] >= 0 && lr[1] < 0 && pl[0] <= 0 && lr[0] < 0)
        //     thetalr = thetalr + 2 * M_PI;
        // if(pr[1] <= 0 && rl[1] > 0 && pr[0] <= 0 && rl[0] < 0)
        //     thetarl = thetarl - 2 * M_PI;

        if(lr[1] > 0 && lr[0] < 0 && rl[1] < 0 && rl[0] < 0)
            thetalr -= 2 * M_PI;

        auto left_ori = gap.convex.convex_lidx * msg.get()->angle_increment + msg.get()->angle_min;
        auto right_ori = gap.convex.convex_ridx * msg.get()->angle_increment + msg.get()->angle_min;
        // ROS_INFO_STREAM("Goal Dist: " << goal_dist << ", R_close: " << r_close << ", Goal Ori: " << goal_orientation << ", left ori: " << left_ori << ", right or: " << right_ori);

        // Second condition: if angle smaller than M_PI / 3
        // Check if arc length < 3 robot width
        bool gap_size_check = (right_ori - left_ori) < M_PI;
        if(!gap_size_check)
        {
            ROS_WARN_STREAM("Current local goal finding doesn't support gap larger than pi.");
            gap.goal.x = (x1 + x2) / 2;
            gap.goal.y = (y1 + y2) / 2;
            gap.goal.discard = true;
            gap.goal.set = true;
            return;
        }

        float dist = 0;
        bool small_gap = false;
        if (gap_size_check && !cfg_->planning.planning_inflated) {
            // if smaller than M_PI/3
            dist = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
            small_gap = dist < 2 * epl;
        }

        // ROS_INFO_STREAM(gap.mode.reduced << " " << gap.convex.convex_lidx << " " << gap.convex.convex_ridx << " " << pl[0] << " " << pl[1] << " " << pr[0] << " " << pr[1] << " " << thetarl << " " << thetalr);

        if (thetarl < thetalr || small_gap) {
            gap.goal.x = (x1 + x2) / 2;
            gap.goal.y = (y1 + y2) / 2;
            gap.goal.discard = thetarl < thetalr;
            gap.goal.set = true;
            // ROS_INFO_STREAM("Gap L(" << x1 << ", " << y1 << "), R(" << x2 << ", " << y2 << ")");
            // ROS_INFO_STREAM("Gap goal (" << gap.goal.x << ", " << gap.goal.y);
            return;
        }

        if (checkGoalVisibility(localgoal)) {
            gap.goal.x = localgoal.pose.position.x;
            gap.goal.y = localgoal.pose.position.y;
            gap.goal.set = true;
            gap.goal.goalwithin = true;
            // ROS_INFO_STREAM("Local goal location");
            return;
        }
        
        float goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        float confined_theta = std::min(thetarl, std::max(thetalr, goal_orientation));
        float confined_r = (gap.convex.convex_rdist - gap.convex.convex_ldist) * (confined_theta - thetalr) / (thetarl - thetalr)
            + gap.convex.convex_ldist;
        float xg = confined_r * cos(confined_theta);
        float yg = confined_r * sin(confined_theta);
        Eigen::Vector2f anchor(xg, yg);
        // Eigen::Matrix2f r_negpi2;
        //     r_negpi2 << 0,1,-1,0;
        // auto offset = r_negpi2 * (pr - pl);
        // auto goal_pt = offset / offset.norm() * (epl / 2) * cfg_->traj.inf_ratio + anchor;
        Eigen::Vector2f goal_pt;
        float waypoint_dist_thresh = (epl) * cfg_->traj.inf_ratio; // 0.1 // TODO: change this 1.5 to param
        
        if((goal_orientation - thetalr) > 0 && (goal_orientation - thetarl) < 0 && (anchor - lr).norm() >= waypoint_dist_thresh && (anchor - rl).norm() >= waypoint_dist_thresh)
        {
            goal_pt = anchor;
        }
        else
        {
            Eigen::Vector2f mid_pt = (lr + rl) / 2;
            float mid_pt_angle = atan2(mid_pt[1], mid_pt[0]);
            float mid_pt_side_length = (mid_pt - lr).norm();

            float ang_anchor_lr = abs(goal_orientation - thetalr);
            float ang_anchor_rl = abs(goal_orientation - thetarl);

            if(ang_anchor_lr <= ang_anchor_rl)
            {
                Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (rl - lr) / (rl - lr).norm() + anchor;
                float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
                if(offset_anchor_angle < mid_pt_angle)
                {
                    goal_pt = offset_anchor;
                }
                else
                {
                    goal_pt = mid_pt;
                }
            }
            else
            {
                Eigen::Vector2f offset_anchor = waypoint_dist_thresh * (lr - rl) / (lr - rl).norm() + anchor;
                float offset_anchor_angle = atan2(offset_anchor[1], offset_anchor[0]);
                if(offset_anchor_angle > mid_pt_angle)
                {
                    goal_pt = offset_anchor;
                }
                else
                {
                    goal_pt = mid_pt;
                }
            }
        }
        Eigen::Matrix2f r_negpi2;
            r_negpi2 << 0,1,-1,0;
        auto offset = r_negpi2 * (pr - pl);
        goal_pt += robot_geo_proc_.getRobotMaxRadius() * offset / offset.norm();

        // ROS_INFO_STREAM("l gap [" << pl[0] << " , " << pl[1] << "], r gap [" << pr[0] << " , " << pr[1] << "], thetalr: " << thetalr << " thetarl: " << thetarl << " goal orient: " << goal_orientation << " Anchor [" << anchor[0] << " , " << anchor[1] << "], Waypoint [" << goal_pt[0] << " , " << goal_pt[1] << "]");
        // float half_max_r = robot_geo_proc_.getRobotMaxRadius() / 2;
        // auto goal_pt = offset * half_max_r * cfg_->traj.inf_ratio + anchor;

        // float r1 = gap.convex.convex_ldist;
        // float r2 = gap.convex.convex_rdist;
        // double r_close = (double) std::min(r1, r2);
        // double goal_dist = sqrt(
        //     pow(localgoal.pose.position.y, 2) + 
        //     pow(localgoal.pose.position.x, 2)
        // );

        


        gap.goal.x = goal_pt(0);
        gap.goal.y = goal_pt(1);
        gap.goal.set = true;

    }

    bool GapManipulator::checkGoalVisibility(geometry_msgs::PoseStamped localgoal) {
        boost::mutex::scoped_lock lock(egolock);
        double dist2goal = sqrt(pow(localgoal.pose.position.x, 2) + pow(localgoal.pose.position.y, 2));

        auto scan = *msg.get();
        auto min_val = *std::min_element(scan.ranges.begin(), scan.ranges.end());

        // If sufficiently close to robot
        Eigen::Vector2d orient_vec(1, 0);
        Eigen::Vector2d goal_vec(localgoal.pose.position.x, localgoal.pose.position.y);
        double er = robot_geo_proc_.getRobotMaxRadius();
        if (dist2goal < 2 * er) {
            return true;
        }

        // If within closest configuration space
        double er_max = robot_geo_proc_.getRobotMaxRadius(); //TODO: check
        if (dist2goal < min_val - cfg_->traj.inf_ratio * er_max) {
            return true;
        }

        // Should be sufficiently far, otherwise we are in trouble
        double goal_angle = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int incident_angle = (int) round((goal_angle - scan.angle_min) / scan.angle_increment);

        // double half_angle = std::asin(cfg_->rbt.r_inscr / dist2goal);
        // int index = std::ceil(half_angle / scan.angle_increment) * 1.5;
        int index = (int)(scan.ranges.size()) / 8;
        int lower_bound = std::max(incident_angle - index, 0);
        int upper_bound = std::min(incident_angle + index, int(scan.ranges.size() - 1));
        auto min_val_round_goal = *std::min_element(scan.ranges.begin() + lower_bound, scan.ranges.begin() + upper_bound);
        return dist2goal < min_val_round_goal;
    }

    // In place modification
    void GapManipulator::reduceGap(potential_gap::Gap& gap, geometry_msgs::PoseStamped localgoal) {
        int lidx = gap.LIdx();
        int ridx = gap.RIdx();
        if (!msg) return; 
        double angular_size = (ridx - lidx) * (msg.get()->angle_increment);

        if (angular_size < cfg_->gap_manip.reduction_threshold){
            return;
        }

        int gap_size = cfg_->gap_manip.reduction_target / msg.get()->angle_increment;
        int l_biased_r = lidx + gap_size;
        int r_biased_l = ridx - gap_size;

        double goal_orientation = std::atan2(localgoal.pose.position.y, localgoal.pose.position.x);
        int goal_idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);

        int acceptable_dist = int(round(gap_size / 2));

        int new_l, new_r;
        if (goal_idx + acceptable_dist > ridx){
            // r-biased Gap
            new_r = ridx;
            new_l = r_biased_l;
        } else if (goal_idx - acceptable_dist < lidx) {
            // l-biased gap
            new_r = l_biased_r;
            new_l = lidx;
        } else {
            // Lingering in center
            new_l = goal_idx - acceptable_dist;
            new_r = goal_idx + acceptable_dist;
        }

        // ROS_INFO_STREAM(lidx << " " << ridx << " " << l_biased_r << " " << r_biased_l << " " << goal_idx + acceptable_dist << " " << goal_idx - acceptable_dist << " " << new_l << " " << new_r);
        
        if(new_l == new_r)
        {
            if(new_l == (num_of_scan - 1))
                new_l--;
            else
                new_r++;
        }

        float ldist = gap.LDist();
        float rdist = gap.RDist();
        float new_ldist = float(new_l - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        float new_rdist = float(new_r - lidx) / float(ridx - lidx) * (rdist - ldist) + ldist;
        gap.convex.convex_lidx = new_l;
        gap.convex.convex_ridx = new_r;
        gap.convex.convex_ldist = new_ldist + cfg_->gap_viz.viz_jitter;
        gap.convex.convex_rdist = new_rdist + cfg_->gap_viz.viz_jitter;
        gap.life_time = 50;
        gap.mode.reduced = true;

        // ROS_INFO_STREAM("LIdx: " << gap.convex.convex_lidx << ", RIdx: " << gap.convex.convex_ridx <<
        //                 ", LDist: " << gap.convex.convex_ldist << ", RDist: " << gap.convex.convex_rdist);

        return;
    }

    void GapManipulator::convertAxialGap(potential_gap::Gap& gap) {
        // Return if not axial gap or disabled
        if (!gap.isAxial() || !cfg_->gap_manip.axial_convert) {
            // ROS_INFO_STREAM("Swept gap.");
            return;
        }

        auto stored_scan_msgs = *msg.get();
        
        bool left = gap.isLeftType();
        // Extend of rotation to the radial gap 
        // amp-ed by a **small** ratio to ensure the local goal does not exactly fall on the
        // visibility line
        // float rot_val = (float) std::atan2(cfg_->gap_manip.epsilon2 * cfg_->gap_manip.rot_ratio, cfg_->gap_manip.epsilon1);

        auto half_num_scan = gap.half_scan;

        int l_idx, r_idx;
        float l_dist, r_dist;
        if(gap.mode.reduced)
        {
            l_idx = gap.convex.convex_lidx;
            l_dist = gap.convex.convex_ldist;
            r_idx = gap.convex.convex_ridx;
            r_dist = gap.convex.convex_rdist;
        }
        else
        {
            l_idx = gap.LIdx();
            l_dist = gap.LDist();
            r_idx = gap.RIdx();
            r_dist = gap.RDist();
        }

        float x1, x2, y1, y2;

        x1 = (l_dist) * cos(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);
        y1 = (l_dist) * sin(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);

        x2 = (r_dist) * cos(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);
        y2 = (r_dist) * sin(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);

        Eigen::Vector2d l_vec(x1, y1);
        Eigen::Vector2d r_vec(x2, y2);
        Eigen::Vector2d mid = (l_vec + r_vec) / 2;
        Eigen::Vector2d robot_orient(1,0);
        float robot_el = float(robot_geo_proc_.getLinearDecayEquivalentPL(robot_orient, mid, mid.norm()));
        float robot_er = float(robot_geo_proc_.getLinearDecayEquivalentRL(robot_orient, mid, mid.norm()));
        
        float rot_val = (float) std::atan2(robot_el / 2 * cfg_->gap_manip.rot_ratio, robot_er / 2);
        float theta = left ? (rot_val + 1e-3): -(rot_val + 1e-3);
        int near_idx, far_idx;
        float near_dist, far_dist;
        
        if (left) {
            // near_idx = gap.LIdx();
            // far_idx = gap.RIdx();
            // near_dist = gap.LDist();
            // far_dist = gap.RDist();
            near_idx = l_idx;
            far_idx = r_idx;
            near_dist = l_dist;
            far_dist = r_dist;
        } else {
            far_idx = l_idx;
            near_idx = r_idx;
            far_dist = l_dist;
            near_dist = r_dist;
        }

        // ROS_INFO_STREAM(far_idx << " " << near_idx << " " << far_dist << " " << near_dist);
        
        Eigen::Matrix3f rot_mat;
        rot_mat << cos(theta), -sin(theta), 0,
                    sin(theta), cos(theta), 0,
                    0, 0, 1;
        
        Eigen::Matrix3f near_rbt;
        near_rbt << 1, 0, near_dist * cos(M_PI / half_num_scan * (near_idx - half_num_scan)),
                    0, 1, near_dist * sin(M_PI / half_num_scan * (near_idx - half_num_scan)),
                    0, 0, 1;
        Eigen::Matrix3f far_rbt;
        far_rbt  << 1, 0, far_dist * cos(M_PI / half_num_scan * (far_idx - half_num_scan)),
                    0, 1, far_dist * sin(M_PI / half_num_scan * (far_idx - half_num_scan)),
                    0, 0, 1;
        
        Eigen::Matrix3f rot_rbt = near_rbt * (rot_mat * (near_rbt.inverse() * far_rbt));

        float r = float(sqrt(pow(rot_rbt(0, 2), 2) + pow(rot_rbt(1, 2), 2)));
        int idx = int (std::atan2(rot_rbt(1, 2), rot_rbt(0, 2)) / M_PI * half_num_scan) + half_num_scan;

        // Rotation Completed
        // Get minimum dist range val from start to target index location
        // For wraparound
        int offset = left ? r_idx: idx;
        int upperbound = left ? idx : l_idx;
        int intermediate_pt = offset + 1;
        int second_inter_pt = intermediate_pt;
        int size = upperbound - offset;


        if ((upperbound - offset) < 3) {
            // Arbitrary value
            gap.goal.discard = true;
            return;
        }

        offset = std::max(offset, 0);
        upperbound = std::min(upperbound, num_of_scan - 1);
        std::vector<float> min_dist(upperbound - offset);

        if (size == 0) {
            // This shouldn't happen
            ROS_FATAL_STREAM("[convertAxialGap] This should not happen");
            return;
        }


        if (stored_scan_msgs.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect gap manip");
        }

        try{
            for (int i = 0; i < min_dist.size(); i++) {
                float dist_sq = pow(near_dist, 2) + pow(stored_scan_msgs.ranges.at(i + offset), 2) -
                    2 * near_dist * stored_scan_msgs.ranges.at(i + offset) * cos((i + offset - near_idx) * stored_scan_msgs.angle_increment);
                if(dist_sq < 0)
                    min_dist.at(i) = 0;
                else
                    min_dist.at(i) = sqrt(dist_sq);
            }
        } catch(...) {
            ROS_FATAL_STREAM("convertAxialGap outofBound");
        }

        auto farside_iter = std::min_element(min_dist.begin(), min_dist.end());
        float farside = *farside_iter;

        // ROS_INFO_STREAM(r << " " << idx << " " << offset << " " << upperbound << " " << farside << " " << stored_scan_msgs.ranges.at(farside_iter - min_dist.begin() + offset));

        Eigen::Matrix3f far_near = near_rbt.inverse() * far_rbt;
        float coefs = far_near.block<2, 1>(0, 2).norm();
        // ROS_INFO_STREAM()
        far_near(0, 2) *= farside / coefs;
        far_near(1, 2) *= farside / coefs;
        Eigen::Matrix3f short_pt = near_rbt * (rot_mat * far_near);

        r = float(sqrt(pow(short_pt(0, 2), 2) + pow(short_pt(1, 2), 2)));
        if(r == 0)
        {
            gap.goal.discard = true;
            return;
        }
        idx = int (std::atan2(short_pt(1, 2), short_pt(0, 2)) / M_PI * half_num_scan) + half_num_scan;

        // ROS_INFO_STREAM(near_idx << " " << idx << " " << near_dist << " " << r);
        // Recalculate end point location based on length

        // ROS_INFO_STREAM("Before Rotation: " << original_gap._ldist << ", " << original_gap._rdist);
        // ROS_INFO_STREAM("Near Idx: " << near_idx << ", Far Idx" << idx);
        gap.convex.convex_lidx = left ? near_idx : idx;
        gap.convex.convex_ldist = left ? near_dist : r;
        gap.convex.convex_ridx = left ? idx : near_idx;
        gap.convex.convex_rdist = left ? r : near_dist;

        if(gap.convex.convex_lidx == gap.convex.convex_ridx)
        {
            if(gap.convex.convex_lidx == (half_num_scan * 2 - 1))
                gap.convex.convex_lidx--;
            else
                gap.convex.convex_ridx++;
        }

        if (left && gap.convex.convex_ridx < gap.convex.convex_lidx) {
            gap.goal.discard = true;
        }

        if (!left && gap.convex.convex_ridx < gap.convex.convex_lidx) {
            gap.goal.discard = true;
        }

        // ROS_INFO_STREAM("Left: " << left << ", LIdx: " << gap.convex.convex_lidx << ", RIdx: " << gap.convex.convex_ridx <<
        //                 ", LDist: " << gap.convex.convex_ldist << ", RDist: " << gap.convex.convex_rdist);
        gap.mode.agc = true;
    }

    void GapManipulator::radialExtendGap(potential_gap::Gap& selected_gap) {
        if (!cfg_->gap_manip.radial_extend) {
            ROS_DEBUG_STREAM_THROTTLE(1, "Radial Extension is off");
            return;
        }
        // TODO: check if the idx are correct when they cross the 0.
        int half_num_scan = (int)(msg.get()->ranges.size()) / 2;
        float s = selected_gap.getMinSafeDist();

        int l_idx, r_idx;
        float l_dist, r_dist;
        if(selected_gap.mode.reduced || selected_gap.mode.agc)
        {
            l_idx = selected_gap.convex.convex_lidx;
            l_dist = selected_gap.convex.convex_ldist;
            r_idx = selected_gap.convex.convex_ridx;
            r_dist = selected_gap.convex.convex_rdist;
        }
        else
        {
            l_idx = selected_gap.LIdx();
            l_dist = selected_gap.LDist();
            r_idx = selected_gap.RIdx();
            r_dist = selected_gap.RDist();
        }

        float x1, x2, y1, y2;
        x1 = l_dist * cos(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);
        y1 = l_dist * sin(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);

        x2 = r_dist * cos(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);
        y2 = r_dist * sin(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);

        Eigen::Vector2f gL(x1, y1);
        Eigen::Vector2f gR(x2, y2);

        Eigen::Vector2f eL = gL / gL.norm();
        Eigen::Vector2f eR = gR / gR.norm();

        Eigen::Vector2f eB = (eL + eR) / 2;
        eB /= eB.norm();
        float gap_size = std::acos(eL.dot(eR));

        Eigen::Vector2f qB = -s * eB;
        
        // Shifted Back Frame
        Eigen::Vector2f qLp = gL - qB;
        Eigen::Vector2f qRp = gR - qB;

        Eigen::Vector2f pLp = utils::car2pol(qLp);
        // pLp(1) += M_PI;
        Eigen::Vector2f pRp = utils::car2pol(qRp);
        // pRp(1) += M_PI;

        float phiB = pRp(1) - pLp(1);

        Eigen::Vector2f pB = utils::car2pol(-qB);
        // pB(2) += M_PI;

        float thL = pB(1) - gap_size / 4;
        float thR = pB(1) + gap_size / 4;

        // Weighted to get two vectors, not sure why???
        Eigen::Vector2f pLn = utils::pTheta(thL, phiB, pLp, pRp);
        Eigen::Vector2f pRn = utils::pTheta(thR, phiB, pLp, pRp);

        Eigen::Vector2f qLn = utils::pol2car(pLn) + qB;
        Eigen::Vector2f qRn = utils::pol2car(pRn) + qB;

        // Store info back to original gap;
        Eigen::Vector2f polqLn = utils::car2pol(qLn);
        Eigen::Vector2f polqRn = utils::car2pol(qRn);

        selected_gap.convex.convex_lidx = polqLn(1) / M_PI * half_num_scan + half_num_scan;
        selected_gap.convex.convex_ridx = polqRn(1) / M_PI * half_num_scan + half_num_scan;
        selected_gap.convex.convex_ldist = polqLn(0);
        selected_gap.convex.convex_rdist = polqRn(0);
        selected_gap.mode.convex = true;

        if(selected_gap.convex.convex_lidx == selected_gap.convex.convex_ridx)
        {
            if(selected_gap.convex.convex_lidx == (half_num_scan * 2 - 1))
                selected_gap.convex.convex_lidx--;
            else
                selected_gap.convex.convex_ridx++;
        }

        selected_gap.qB = qB;
        ROS_DEBUG_STREAM("l: " << selected_gap._left_idx << " to " << selected_gap.convex.convex_lidx
         << ", r: " << selected_gap._right_idx << " to " << selected_gap.convex.convex_ridx);
        
        ROS_DEBUG_STREAM("ldist: " << selected_gap._ldist << " to " << selected_gap.convex.convex_ldist
        << ", rdist: " << selected_gap._rdist << " to " << selected_gap.convex.convex_rdist);

        return;
    }

    void GapManipulator::inflateGapConserv(potential_gap::Gap& gap)
    {
        if (!cfg_->gap_manip.inflated_gap) {
            ROS_WARN_STREAM("Inflated gap is off");
            return;
        }

        if(cfg_->gap_manip.radial_extend)
        {
            ROS_ERROR_STREAM("This mode doesn't support radial extension yet.");
            return;
        }

        float inf_dist = (float) robot_geo_proc_.getRobotMaxRadius() * cfg_->gap_manip.inflated_gap_scale;
        gap.inf_gap.inf_min_safe_dist = gap.getMinSafeDist() - inf_dist;

        if(gap.inf_gap.inf_min_safe_dist <= 0)
        {
            ROS_WARN_STREAM("The free space circle is too small.");
            return;
        }
        
        int l_idx, r_idx;
        float l_dist, r_dist;
        if(gap.mode.reduced || gap.mode.agc || gap.mode.convex)
        {
            l_idx = gap.convex.convex_lidx;
            l_dist = gap.convex.convex_ldist;
            r_idx = gap.convex.convex_ridx;
            r_dist = gap.convex.convex_rdist;
        }
        else
        {
            l_idx = gap.LIdx();
            l_dist = gap.LDist();
            r_idx = gap.RIdx();
            r_dist = gap.RDist();
        }

        int half_num_scan = (int) ((float) msg.get()->ranges.size()) / 2;
        float xl, xr, yl, yr;
        xl = l_dist * cos(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);
        yl = l_dist * sin(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);
        Eigen::Vector2f lgap(xl, yl);

        xr = r_dist * cos(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);
        yr = r_dist * sin(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);
        Eigen::Vector2f rgap(xr, yr);

        // inflate gap points
        Eigen::Vector2f lgap_n(-lgap(1), lgap(0));
        lgap_n = inf_dist * lgap_n / lgap_n.norm();

        Eigen::Vector2f rgap_n(rgap(1), -rgap(0));
        rgap_n = inf_dist * rgap_n / rgap_n.norm();

        gap.inf_gap.inf_l = lgap_n + lgap;
        gap.inf_gap.inf_r = rgap_n + rgap;

        gap.inf_gap.inf_gap_int = utils::intersectLines(gap.inf_gap.inf_l, lgap_n, gap.inf_gap.inf_r, rgap_n);
        // ROS_INFO_STREAM("inflated: " << gap.inf_gap.inf_l << " " << gap.inf_gap.inf_r << " " << gap.inf_gap.inf_gap_int << " " << gap.inf_gap.inf_min_safe_dist);
        if(gap.inf_gap.inf_gap_int.norm() > gap.inf_gap.inf_min_safe_dist)
        {
            ROS_WARN_STREAM("The inflated gap intersection point [" << gap.inf_gap.inf_gap_int.norm() << "] is outside the inflated circle [" << gap.inf_gap.inf_min_safe_dist << "].");
            return;
        }

        if(gap.inf_gap.inf_l.norm() < gap.inf_gap.inf_min_safe_dist)
        {
            gap.inf_gap.inf_l = utils::findScaleLineCirInt(gap.inf_gap.inf_gap_int, gap.inf_gap.inf_l, gap.inf_gap.inf_min_safe_dist, true);
            if(utils::is_inf(gap.inf_gap.inf_l))
            {
                ROS_WARN_STREAM("Cannot extend inf_l.");
                return;
            }
        }

        if(gap.inf_gap.inf_r.norm() < gap.inf_gap.inf_min_safe_dist)
        {
            gap.inf_gap.inf_r = utils::findScaleLineCirInt(gap.inf_gap.inf_gap_int, gap.inf_gap.inf_r, gap.inf_gap.inf_min_safe_dist, true);
            if(utils::is_inf(gap.inf_gap.inf_r))
            {
                ROS_WARN_STREAM("Cannot extend inf_r.");
                return;
            }
        }

        // ROS_INFO_STREAM("raw: " << lgap << " " << rgap);
        // ROS_INFO_STREAM("inflated extended: " << gap.inf_gap.inf_l << " " << gap.inf_gap.inf_r << " " << gap.inf_gap.inf_gap_int << " " << gap.inf_gap.inf_min_safe_dist);
        std::vector<Eigen::Vector2f> lgc_int_list = utils::intersectLineCir(gap.inf_gap.inf_gap_int, gap.inf_gap.inf_l, Eigen::Vector2f(0, 0), gap.inf_gap.inf_min_safe_dist, true);
        std::vector<Eigen::Vector2f> rgc_int_list = utils::intersectLineCir(gap.inf_gap.inf_gap_int, gap.inf_gap.inf_r, Eigen::Vector2f(0, 0), gap.inf_gap.inf_min_safe_dist, true);
        
        if(lgc_int_list.size() == 0 || rgc_int_list.size() == 0 || lgc_int_list.size() > 1 || rgc_int_list.size() > 1)
        {
            // ROS_INFO_STREAM(lgc_int_list.size() << " " << rgc_int_list.size());
            ROS_WARN_STREAM("No inflated gap line circle intersection or more than 1.");
            return;
        }

        gap.inf_gap.inf_lgc_int = lgc_int_list[0];
        gap.inf_gap.inf_rgc_int = rgc_int_list[0];

        gap.mode.inflated = true;

        return;
    }

    void GapManipulator::inflateGapLarge(potential_gap::Gap& gap)
    {
        if(cfg_->planning.use_bezier && !cfg_->gap_manip.inflated_gap)
        {
            ROS_WARN_STREAM("Bezier is used. Inflation is required. Parameter is ignored.");
        }
        else if(!cfg_->planning.use_bezier && !cfg_->gap_manip.inflated_gap) {
            ROS_WARN_STREAM("Inflated gap is off");
            return;
        }

        if(robot_geo_proc_.robot_.shape == RobotShape::box && robot_geo_proc_.robot_.holonomic)
        {
            ROS_WARN_STREAM("Holonomic box robot, inflation is off.");
            return;
        }

        // if(cfg_->gap_manip.radial_extend)
        // {
        //     ROS_ERROR_STREAM("This mode doesn't support radial extension yet.");
        //     return;
        // }

        float inf_dist = (float) robot_geo_proc_.getRobotMaxRadius() * cfg_->gap_manip.inflated_gap_scale;
        gap.inf_gap.inf_min_safe_dist = gap.getMinSafeDist() - inf_dist;

        if(gap.inf_gap.inf_min_safe_dist <= 0)
        {
            ROS_WARN_STREAM("The free space circle is too small.");
            return;
        }
        
        int l_idx, r_idx;
        float l_dist, r_dist;
        if(gap.mode.reduced || gap.mode.agc || gap.mode.convex)
        {
            l_idx = gap.convex.convex_lidx;
            l_dist = gap.convex.convex_ldist;
            r_idx = gap.convex.convex_ridx;
            r_dist = gap.convex.convex_rdist;
        }
        else
        {
            l_idx = gap.LIdx();
            l_dist = gap.LDist();
            r_idx = gap.RIdx();
            r_dist = gap.RDist();
        }

        int half_num_scan = (int) ((float) msg.get()->ranges.size()) / 2;
        float xl, xr, yl, yr;
        xl = l_dist * cos(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);
        yl = l_dist * sin(-((float) half_num_scan - l_idx) / half_num_scan * M_PI);
        Eigen::Vector2f lgap(xl, yl);

        xr = r_dist * cos(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);
        yr = r_dist * sin(-((float) half_num_scan - r_idx) / half_num_scan * M_PI);
        Eigen::Vector2f rgap(xr, yr);

        gap.inf_gap.min_safe_dist = gap.getMinSafeDist();
        gap.inf_gap.lgap = lgap;
        gap.inf_gap.rgap = rgap;

        ros::WallTime start_time = ros::WallTime::now();
        Eigen::Vector2f lgap_nul = findNearUnblockLine(Eigen::Vector2f(0, 0), lgap, gap.getMinSafeDist(), false);
        Eigen::Vector2f rgap_nul = findNearUnblockLine(Eigen::Vector2f(0, 0), rgap, gap.getMinSafeDist(), true);
        ros::WallDuration unl_d = ros::WallTime::now() - start_time;
        // ROS_INFO_STREAM("Find NUL time: " << ((float) unl_d.toNSec()) / 1e6 << " ms.");

        // inflate gap points
        Eigen::Vector2f lgap_nul_inf, rgap_nul_inf;

        if(abs(lgap.norm() - gap.getMinSafeDist()) <= 1e-3)
        {
            gap.inf_gap.inf_l = (gap.inf_gap.inf_min_safe_dist + 1e-2) * lgap / lgap.norm();
            lgap_nul_inf = Eigen::Vector2f(0, 0);
        }
        else
        {
            Eigen::Vector2f l_nul_vec = lgap - lgap_nul;
            Eigen::Vector2f lgap_n(-l_nul_vec(1), l_nul_vec(0));
            lgap_n = inf_dist * lgap_n / lgap_n.norm();

            gap.inf_gap.inf_l = lgap_n + lgap;
            lgap_nul_inf = lgap_nul + lgap_n;
        }

        if(abs(rgap.norm() - gap.getMinSafeDist()) <= 1e-3)
        {
            gap.inf_gap.inf_r = (gap.inf_gap.inf_min_safe_dist + 1e-2) * rgap / rgap.norm();
            rgap_nul_inf = Eigen::Vector2f(0, 0);
        }
        else
        {
            Eigen::Vector2f r_nul_vec = rgap - rgap_nul;
            Eigen::Vector2f rgap_n(r_nul_vec(1), -r_nul_vec(0));
            rgap_n = inf_dist * rgap_n / rgap_n.norm();

            gap.inf_gap.inf_r = rgap_n + rgap;
            rgap_nul_inf = rgap_nul + rgap_n;
        }

        // ROS_INFO_STREAM("Orig: " << lgap << " " << lgap_nul << " " << rgap << " " << rgap_nul << " " << gap.getMinSafeDist());
        // ROS_INFO_STREAM("Orig Inflated: " << gap.inf_gap.inf_l << " " << lgap_nul_inf << " " << gap.inf_gap.inf_r << " " << rgap_nul_inf << " " << gap.inf_gap.inf_min_safe_dist);

        if(!utils::ptInsideCirc(Eigen::Vector2f(0, 0), lgap_nul_inf, gap.inf_gap.inf_min_safe_dist))
        {
            lgap_nul_inf = utils::findScaleLineCirInt(gap.inf_gap.inf_l, lgap_nul_inf, gap.inf_gap.inf_min_safe_dist, false);
            if(utils::is_inf(lgap_nul_inf))
            {
                ROS_WARN_STREAM("Cannot extend lgap_nul_inf.");
                return;
            }
        }

        if(!utils::ptInsideCirc(Eigen::Vector2f(0, 0), rgap_nul_inf, gap.inf_gap.inf_min_safe_dist))
        {
            rgap_nul_inf = utils::findScaleLineCirInt(gap.inf_gap.inf_r, rgap_nul_inf, gap.inf_gap.inf_min_safe_dist, false);
            if(utils::is_inf(rgap_nul_inf))
            {
                ROS_WARN_STREAM("Cannot extend rgap_nul_inf.");
                return;
            }
        }

        // if(utils::isLeftVec(gap.inf_gap.inf_l - rgap_nul_inf, gap.inf_gap.inf_r - rgap_nul_inf) &&
        //    utils::isLeftVec(lgap_nul_inf - rgap_nul_inf, gap.inf_gap.inf_r - rgap_nul_inf))
        if(utils::isLeftofLine(rgap_nul_inf, gap.inf_gap.inf_r, gap.inf_gap.inf_l) &&
           utils::isLeftofLine(rgap_nul_inf, gap.inf_gap.inf_r, lgap_nul_inf))
        {
            ROS_WARN_STREAM("Inflated left gap pt is on the left of right gap line.");
            return;
        }

        bool has_inter = utils::doIntersect(gap.inf_gap.inf_l, lgap_nul_inf, gap.inf_gap.inf_r, rgap_nul_inf);
        if(has_inter)
        {
            if(!utils::isLeftofLine(rgap_nul_inf, gap.inf_gap.inf_r, lgap_nul_inf))
            {
                Eigen::Vector2f gaps_inter = utils::intersectLines(gap.inf_gap.inf_l, lgap_nul_inf, gap.inf_gap.inf_r, rgap_nul_inf);
                if(gaps_inter.norm() < gap.inf_gap.inf_min_safe_dist)
                {
                    ROS_WARN_STREAM("The inflated gap lines are intersected and inside of inflated circle.");
                    return;
                }
            }
            else
            {
                Eigen::Vector2f gaps_inter = utils::intersectLines(gap.inf_gap.inf_l, lgap_nul_inf, gap.inf_gap.inf_r, rgap_nul_inf);
                if(gaps_inter.norm() > gap.inf_gap.inf_min_safe_dist)
                {
                    ROS_WARN_STREAM("The inflated gap lines are intersected and outside of inflated circle.");
                    return;
                }
            }
        }

        Eigen::Vector2f lgap_inf_vec = gap.inf_gap.inf_l - lgap_nul_inf;
        Eigen::Vector2f rgap_inf_vec = gap.inf_gap.inf_r - rgap_nul_inf;

        if(utils::ptInsideCirc(Eigen::Vector2f(0, 0), gap.inf_gap.inf_l, gap.inf_gap.inf_min_safe_dist))
        {
            gap.inf_gap.inf_l = utils::findScaleLineCirInt(lgap_nul_inf, gap.inf_gap.inf_l, gap.inf_gap.inf_min_safe_dist, true);
            if(utils::is_inf(gap.inf_gap.inf_l))
            {
                ROS_WARN_STREAM("Cannot extend inf_l.");
                return;
            }
        }

        if(utils::ptInsideCirc(Eigen::Vector2f(0, 0), gap.inf_gap.inf_r, gap.inf_gap.inf_min_safe_dist))
        {
            gap.inf_gap.inf_r = utils::findScaleLineCirInt(rgap_nul_inf, gap.inf_gap.inf_r, gap.inf_gap.inf_min_safe_dist, true);
            if(utils::is_inf(gap.inf_gap.inf_r))
            {
                ROS_WARN_STREAM("Cannot extend inf_r.");
                return;
            }
        }

        // ROS_INFO_STREAM("raw: " << lgap << " " << rgap);
        // ROS_INFO_STREAM("Inflated: " << gap.inf_gap.inf_l << " " << lgap_nul_inf << " " << gap.inf_gap.inf_r << " " << rgap_nul_inf << " " << gap.inf_gap.inf_min_safe_dist);
        // ROS_INFO_STREAM("Left: ");
        std::vector<Eigen::Vector2f> lgc_int_list = utils::intersectLineCir(lgap_nul_inf, gap.inf_gap.inf_l, Eigen::Vector2f(0, 0), gap.inf_gap.inf_min_safe_dist, true);
        // ROS_INFO_STREAM("Right: ");
        std::vector<Eigen::Vector2f> rgc_int_list = utils::intersectLineCir(rgap_nul_inf, gap.inf_gap.inf_r, Eigen::Vector2f(0, 0), gap.inf_gap.inf_min_safe_dist, true);
        
        if(lgc_int_list.size() == 0 || rgc_int_list.size() == 0 || lgc_int_list.size() > 1 || rgc_int_list.size() > 1)
        {
            // ROS_INFO_STREAM(lgc_int_list.size() << " " << rgc_int_list.size());
            ROS_WARN_STREAM("No inflated gap line circle intersection or more than 1.");
            return;
        }

        gap.inf_gap.inf_lgc_int = lgc_int_list[0];
        gap.inf_gap.inf_rgc_int = rgc_int_list[0];

        gap.mode.inflated = true;

        return;
    }

    bool GapManipulator::checkLineEgoBlocked(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
    {
        int scan_size = msg.get()->ranges.size();
        float angle_increment = (float) msg.get()->angle_increment;

        float ang1 = atan2(p1(1), p1(0));
        float ang2 = atan2(p2(1), p2(0));

        float angl, angu;
        if(abs(ang1 - ang2) > M_PI)
        {
            float ang_change = ang1 > M_PI ? (ang1 - 2 * M_PI) : (ang2 - 2 * M_PI);
            float ang_unchange = ang1 > M_PI ? ang2 : ang1;

            utils::arrangeAngCCW(ang_change, ang_unchange, angl, angu);
        }
        else
        {
            utils::arrangeAngCCW(ang1, ang2, angl, angu);
        }
        
        int idx1 = (int) round((angl - -M_PI) / angle_increment);
        idx1 = idx1 < 0 ? 0 : idx1;
        idx1 = idx1 >= scan_size ? (scan_size - 1) : idx1;
        int idx2 = (int) round((angu - -M_PI) / angle_increment);
        idx2 = idx2 < 0 ? 0 : idx2;
        idx2 = idx2 >= scan_size ? (scan_size - 1) : idx2;

        if(idx1 > idx2)
        {
            ROS_ERROR_STREAM("[checkLineEgoBlocked] idx1 > idx2");
            throw;
        }

        auto start_it = msg.get()->ranges.begin() + idx1;
        auto end_it = msg.get()->ranges.begin() + idx2 + 1;
        std::vector<float> sub_scan(start_it, end_it);

        auto min_dist_it = min_element(sub_scan.begin(), sub_scan.end());
        int min_dist_idx = min_dist_it - sub_scan.begin();
        float min_dist = *min_dist_it;
        
        float min_diff_ang = min_dist_idx * angle_increment;
        float min_dist_ang = min_diff_ang + angl;
        Eigen::Vector2f min_dist_unit_vec(cos(min_dist_ang), sin(min_dist_ang));

        Eigen::Vector2f line_dist_pt = utils::intersectLines(p1, p2, Eigen::Vector2f(0, 0), min_dist_unit_vec);
        if(utils::is_inf(line_dist_pt))
        {
            ROS_INFO_STREAM("[checkLineEgoBlocked] Cannot find intersection point. Parallel [" << line_dist_pt(0) << "," << line_dist_pt(1) << "]");
            return false;
        }

        float line_dist = line_dist_pt.norm();

        if(min_dist < line_dist)
            return true;

        return false;
    }

    bool GapManipulator::checkLineEgoBlocked(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float& nearest_dist)
    {
        int scan_size = msg.get()->ranges.size();
        float angle_increment = (float) msg.get()->angle_increment;

        float ang1 = atan2(p1(1), p1(0));
        float ang2 = atan2(p2(1), p2(0));

        float angl, angu;
        if(abs(ang1 - ang2) > M_PI)
        {
            float ang_change = ang1 > M_PI ? (ang1 - 2 * M_PI) : (ang2 - 2 * M_PI);
            float ang_unchange = ang1 > M_PI ? ang2 : ang1;

            utils::arrangeAngCCW(ang_change, ang_unchange, angl, angu);
        }
        else
        {
            utils::arrangeAngCCW(ang1, ang2, angl, angu);
        }

        int idx1 = (int) round((angl - -M_PI) / angle_increment);
        idx1 = idx1 < 0 ? 0 : idx1;
        idx1 = idx1 >= scan_size ? (scan_size - 1) : idx1;
        int idx2 = (int) round((angu - -M_PI) / angle_increment);
        idx2 = idx2 < 0 ? 0 : idx2;
        idx2 = idx2 >= scan_size ? (scan_size - 1) : idx2;

        if(idx1 > idx2)
        {
            ROS_ERROR_STREAM("[checkLineEgoBlocked] idx1 > idx2");
            throw;
        }

        auto start_it = msg.get()->ranges.begin() + idx1;
        auto end_it = msg.get()->ranges.begin() + idx2 + 1;
        std::vector<float> sub_scan(start_it, end_it);

        nearest_dist = utils::float_inf;
        bool blocked = false;
        for(size_t i = 0; i < sub_scan.size(); i++)
        {
            float dist_i = sub_scan[i];
            float ang_i = i + angle_increment + angl;
            Eigen::Vector2f i_vec(cos(ang_i), sin(ang_i));

            Eigen::Vector2f line_i_pt = utils::intersectLines(p1, p2, Eigen::Vector2f(0, 0), i_vec);
            if(utils::is_inf(line_i_pt))
            {
                ROS_INFO_STREAM("[checkLineEgoBlocked] Cannot find intersection point. Parallel [" << line_i_pt(0) << "," << line_i_pt(1) << "]");
                return false;
            }
            float line_i_pt_dist = line_i_pt.norm();

            float dist_diff = abs(dist_i - line_i_pt_dist);
            if(dist_diff < nearest_dist)
                nearest_dist = dist_diff;

            if(dist_i < line_i_pt_dist)
                blocked = true;
        }

        return blocked;
    }

    Eigen::Vector2f GapManipulator::findNearUnblockLine(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, float r, bool ccw)
    {
        Eigen::Vector2f tang_pt = utils::circTangPt(circ_orig, pt, r, ccw);
        ROS_DEBUG_STREAM("Tangent pt: " << tang_pt);

        Eigen::Vector2f near_line_pt;
        bool use_s1 = true;

        if(use_s1)
        {
            int step = 5;
            int max_it = 20;
            near_line_pt = searchNULS1(circ_orig, pt, tang_pt, step, max_it);
        }
        else
        {
            int max_it = 20;
            float max_dist = 5e-2;
            near_line_pt = searchNULS2(circ_orig, pt, tang_pt, max_it, max_dist);
        }

        return near_line_pt;
    }

    Eigen::Vector2f GapManipulator::searchNULS1(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, const Eigen::Vector2f& tang_pt, int fixed_it, int max_it)
    {
        Eigen::Vector2f p1, p2, nul_pt;

        bool blocked = checkLineEgoBlocked(tang_pt, pt);
        if(blocked)
        {
            p1 = circ_orig;
            p2 = tang_pt;
        }
        else
        {
            ROS_DEBUG_STREAM("Tangent point is valid.");
            Eigen::Vector2f small_n = -1e-2 * (tang_pt - circ_orig) / (tang_pt - circ_orig).norm();
            return tang_pt + small_n;
        }
        
        int i = 0;
        while(blocked || i < fixed_it)
        {
            if(i + fixed_it >= max_it)
            {
                ROS_WARN_STREAM("Not found unblocked within " << max_it << " iterations.");
                break;
            }

            nul_pt = utils::getMidPt(pt, p1, p2);
            blocked = checkLineEgoBlocked(nul_pt, pt);
            if(blocked)
            {
                p2 = nul_pt;
            }
            else
            {
                p1 = nul_pt;
            }
            i++;
        }

        return nul_pt;
    }

    Eigen::Vector2f GapManipulator::searchNULS2(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, const Eigen::Vector2f& tang_pt, int max_it, float max_dist)
    {
        Eigen::Vector2f p1, p2, nul_pt;
        
        bool blocked = checkLineEgoBlocked(tang_pt, pt);
        if(blocked)
        {
            p1 = circ_orig;
            p2 = tang_pt;
        }
        else
        {
            ROS_DEBUG_STREAM("Tangent point is valid.");
            Eigen::Vector2f small_n = -5e-2 * (tang_pt - circ_orig) / (tang_pt - circ_orig).norm();
            return tang_pt + small_n;
        }
        
        int i = 0;
        while(i < max_it)
        {
            nul_pt = utils::getMidPt(pt, p1, p2);
            float near_dist;
            blocked = checkLineEgoBlocked(nul_pt, pt, near_dist);
            if(blocked)
            {
                p2 = nul_pt;
            }
            else
            {
                if(near_dist <= max_dist)
                    return nul_pt;

                p1 = nul_pt;
            }
            i++;
        }

        if(blocked)
        {
            while(blocked)
            {
                nul_pt = utils::getMidPt(pt, p1, p2);
                blocked = checkLineEgoBlocked(nul_pt, pt);
                if(blocked)
                {
                    p2 = nul_pt;
                }
                else
                {
                    break;
                }
            }
        }

        return nul_pt;
    }
}
