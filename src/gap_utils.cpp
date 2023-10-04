#include <potential_gap/gap_utils.h>

namespace potential_gap {
    GapUtils::GapUtils() {}

    GapUtils::~GapUtils() {}

    GapUtils::GapUtils(const PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc) {
        cfg_ = & cfg;
        robot_geo_proc_ = robot_geo_proc;
    }

    void GapUtils::hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<potential_gap::Gap> & observed_gaps)
    {
        observed_gaps.clear();
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        float half_scan = float(stored_scan_msgs.ranges.size() / 2);
        bool prev = true;
        auto max_dist_iter = std::max_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());
        float max_scan_dist = *max_dist_iter;
        auto min_dist = *std::min_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());

        // ROS_INFO_STREAM("Max Dist: " << max_scan_dist << ", min dist: " << min_dist);
        int gap_size = 0;
        std::string frame = stored_scan_msgs.header.frame_id;
        int gap_lidx = 0;
        float gap_ldist = stored_scan_msgs.ranges[0];
        float last_scan = stored_scan_msgs.ranges[0];
        bool prev_lgap = gap_ldist >= max_scan_dist;
        // ROS_INFO_STREAM("gapldist: " << gap_ldist << ", max " << max_scan_dist);
        float scan_dist;
        float scan_diff;
        int wrap = 0;

        for (std::vector<float>::size_type it = 1; it < stored_scan_msgs.ranges.size(); ++it)
        {
            scan_dist = stored_scan_msgs.ranges[it];
            scan_diff = scan_dist - last_scan;
            
            // Arbitrary small threshold for a range difference to be considered
            if (std::abs(scan_diff) > 0.2) 
            {
                // If both current and last values are not infinity, meaning this is not a swept gap
                if (scan_dist < max_scan_dist && last_scan < max_scan_dist) 
                {
                    potential_gap::Gap detected_gap(frame, it - 1, last_scan, true, half_scan);
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    // Find equivalent passing length
                    Eigen::Vector2d orient_vec(1, 0);
                    Eigen::Vector2d m_pt_vec = detected_gap.get_middle_pt_vec();
                    // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    if (detected_gap.get_dist_side() > epl || cfg_->planning.planning_inflated) observed_gaps.push_back(detected_gap);
                }
                
            }

            // Beginning or the end of a reading into infinity => swept gap
            if (last_scan < max_scan_dist != scan_dist < max_scan_dist)
            {
                // If previously marked gap, meaning ending of a gap
                if (prev_lgap)
                {
                    prev_lgap = false;
                    potential_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, half_scan);
                    detected_gap.addRightInformation(it, scan_dist);
                    detected_gap.setMinSafeDist(min_dist);
                    // Inscribed radius gets enforced here, or unless using inflated egocircle,
                    // then no need for range diff
                    // Find equivalent passing length
                    Eigen::Vector2d orient_vec(1, 0);
                    Eigen::Vector2d m_pt_vec = detected_gap.get_middle_pt_vec();
                    // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
                    if (detected_gap.get_dist_side() > epl || cfg_->planning.planning_inflated) observed_gaps.push_back(detected_gap);
                }
                else // previously not marked a gap, not marking the gap
                {
                    gap_lidx = it - 1;
                    gap_ldist = last_scan;
                    prev_lgap = true;
                }
            }
            last_scan = scan_dist;
        }

        // Catch the last gap
        if (prev_lgap) 
        {
            // ROS_INFO_STREAM("catching last gap");
            potential_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, half_scan);
            detected_gap.addRightInformation(int(stored_scan_msgs.ranges.size() - 1), *(stored_scan_msgs.ranges.end() - 1));
            detected_gap.setMinSafeDist(min_dist);
            Eigen::Vector2d orient_vec(1, 0);
            Eigen::Vector2d m_pt_vec = detected_gap.get_middle_pt_vec();
            // double epl = robot_geo_proc_.getDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
            double epl = robot_geo_proc_.getLinearDecayEquivalentPL(orient_vec, m_pt_vec, m_pt_vec.norm());
            if (detected_gap._right_idx - detected_gap._left_idx > 500 || detected_gap.get_dist_side() > epl) observed_gaps.push_back(detected_gap);
        }
        
        // Bridge the last gap around
        if (observed_gaps.size() > 1)
        {
            if (observed_gaps[0].LIdx() == 0 && observed_gaps[observed_gaps.size() - 1].RIdx() == stored_scan_msgs.ranges.size() - 1) // Magic number?
            {
                // Both ends
                float start_side_dist = observed_gaps[0].RDist();
                float end_side_dist = observed_gaps[observed_gaps.size() - 1].LDist();
                int start_side_idx = observed_gaps[0].RIdx();
                int end_side_idx = observed_gaps[observed_gaps.size() - 1].LIdx();

                // float result = (end_side_dist - start_side_dist) * start_side_idx / (observed_gaps.size() - end_side_idx + start_side_idx) + start_side_dist;
                int total_size = 511 - end_side_idx + start_side_idx;
                float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
                observed_gaps[0].setLeftObs();
                observed_gaps[observed_gaps.size() - 1].setRightObs();
                observed_gaps[observed_gaps.size() - 1].addRightInformation(511, result);
                observed_gaps[0].setLDist(result);
            }
        }
    }

    void GapUtils::followtheGap(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<potential_gap::Gap>& observed_gaps)
    {
        observed_gaps.clear();
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        float max_scan_dist = 2.82;
        float scan_dist;
        float last_scan = stored_scan_msgs.ranges[0];
        bool prev = last_scan > max_scan_dist - 0.1;
        int gap_lidx = 0;
        float gap_ldist = last_scan;
        float scan_diff;
        for (std::vector<float>::size_type it = 1; it < stored_scan_msgs.ranges.size(); ++it)
        {
            scan_dist = stored_scan_msgs.ranges[it];
            scan_diff = scan_dist - last_scan;
            if (prev)
            {
                // End of Gap
                if (scan_diff < -0.2) 
                {
                    potential_gap::Gap detected_gap(cfg_->sensor_frame_id, gap_lidx, gap_ldist);
                    detected_gap.addRightInformation(it - 1, scan_dist);
                    observed_gaps.push_back(detected_gap);
                    prev = false;
                }
            } else 
            {
                if (scan_dist > max_scan_dist - 0.1)
                {
                    // Now a free space
                    prev = true;
                    gap_lidx = it;
                    gap_ldist = last_scan;
                }
            }

            last_scan = scan_dist;
        }

        if (prev)
        {
            potential_gap::Gap detected_gap(cfg_->sensor_frame_id, gap_lidx, gap_ldist);
            detected_gap.addRightInformation(511, stored_scan_msgs.ranges[511]);
            observed_gaps.push_back(detected_gap);
        }

        // if (observed_gaps.size() > 1) 
        // {
        //     if (observed_gaps[0].LIdx() == 0 && observed_gaps[observed_gaps.size() - 1].RIdx() == stored_scan_msgs.ranges.size() - 1) // Magic number?
        //     {
        //         // Both ends
        //         float start_side_dist = observed_gaps[0].RDist();
        //         float end_side_dist = observed_gaps[observed_gaps.size() - 1].LDist();
        //         int start_side_idx = observed_gaps[0].RIdx();
        //         int end_side_idx = observed_gaps[observed_gaps.size() - 1].LIdx();

        //         float result = (end_side_dist - start_side_dist) * start_side_idx / (observed_gaps.size() - end_side_idx + start_side_idx) + start_side_dist;

        //         observed_gaps[0].setLeftObs();
        //         observed_gaps[observed_gaps.size() - 1].setRightObs();
        //         observed_gaps[observed_gaps.size() - 1].addRightInformation(511, result);
        //         observed_gaps[0].setLDist(result);
        //     }
        // }
    }


    void GapUtils::safeGapScan(
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<potential_gap::Gap>& left_gap,
        std::vector<potential_gap::Gap>& right_gap,
        std::vector<potential_gap::Gap>& central_gap)
    {
        left_gap.clear();
        right_gap.clear();
        central_gap.clear();
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        int gap_lidx = 0;
        float gap_ldist = stored_scan_msgs.ranges[0];
        float last_scan = stored_scan_msgs.ranges[0];
        float max_scan_dist = 2.82;
        float scan_dist, scan_diff;
        bool type2start = false;
        float half_scan = float(stored_scan_msgs.ranges.size() / 2);
        // bool prev = last_scan > max_scan_dist - 0.1;
        float min_dist = 3;
        int min_idx = 0;
        // ROS_INFO_STREAM("Safe Gap Implementation");
        for (std::vector<float>::size_type it = 1; it < stored_scan_msgs.ranges.size(); ++it) 
        {
            // This is the forward scan, only look for rising edges
            scan_dist = stored_scan_msgs.ranges[it];
            scan_diff = scan_dist - last_scan;
            if (scan_diff > 0.05) 
            {
                if (scan_dist > max_scan_dist - 0.01) {
                    // The start of a type 2 edge
                    gap_lidx = it - 1;
                    gap_ldist = last_scan;
                    type2start = true;
                }
                
                if (scan_diff > 0.2) {
                    potential_gap::Gap detected_gap(cfg_->sensor_frame_id, it - 1, last_scan);
                    min_dist = 3;
                    detected_gap.addRightInformation(it, scan_dist);
                    left_gap.push_back(detected_gap);
                }
            }

            if (scan_diff < -0.05)
            {
                if (type2start && last_scan > max_scan_dist - 0.01) 
                {
                    potential_gap::Gap detected_gap(cfg_->sensor_frame_id, gap_lidx, gap_ldist);
                    detected_gap.addRightInformation(it, scan_dist);
                    central_gap.push_back(detected_gap);
                    // ROS_INFO_STREAM("start " << gap_lidx << ", end " << it);
                    type2start = false;
                }

                // if (scan_diff < -0.2) {
                //     potential_gap::Gap detected_gap("camera_link", it - 1, last_scan);
                //     min_dist = 3;
                //     detected_gap.addRightInformation(it, scan_dist);
                //     left_gap.push_back(detected_gap);
                // }
            }
            last_scan = scan_dist;
        }

        scan_dist = stored_scan_msgs.ranges[511];
        last_scan = stored_scan_msgs.ranges[511];
        gap_lidx = (int) stored_scan_msgs.ranges.size();
        gap_ldist = scan_dist;

        for (std::vector<float>::size_type it = stored_scan_msgs.ranges.size() - 2; it > 0; it--)
        {
            scan_dist = stored_scan_msgs.ranges[it];
            scan_diff = scan_dist - last_scan;
            if (scan_diff > 0.2 && scan_dist < max_scan_dist - 0.1) 
            {
                min_dist = 3;
                potential_gap::Gap detected_gap(cfg_->sensor_frame_id, it, scan_dist);
                detected_gap.addRightInformation(it + 1, last_scan);
                right_gap.push_back(detected_gap);
            }
            last_scan = scan_dist;
        }

        // observed_gaps.pop_back();
    }

    void GapUtils::safeGapClose(boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<potential_gap::Gap>& left_gap,
        std::vector<potential_gap::Gap>& right_gap,
        std::vector<potential_gap::Gap>& central_gap,
        std::vector<potential_gap::Gap>& return_sets)
    {
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        return_sets.clear();
        for (auto & lg : left_gap)
        {
            int l_idx = lg.LIdx();
            float l_dist = lg.LDist();
            int min_idx = lg.RIdx();
            float r_dist = lg.RDist();
            float min_dist = pow(l_dist, 2) + pow(r_dist, 2) - 2 * l_dist * r_dist * std::cos((min_idx - l_idx) * stored_scan_msgs.angle_increment);
            // Potentially re-write with Eigen library for vectorization?
            for (int min_dist_it = l_idx + 1; min_dist_it < stored_scan_msgs.ranges.size() && (int)(min_dist_it - l_idx) < 255; min_dist_it++)
            {
                float angle = ((float) (min_dist_it - l_idx))* stored_scan_msgs.angle_increment;
                float target_dist = stored_scan_msgs.ranges[min_dist_it];
                float dist = pow(l_dist, 2) + 
                    pow(target_dist, 2) - 
                    2 * l_dist * target_dist * std::cos(angle);
                if (dist <= min_dist)
                {
                    min_dist = dist;
                    min_idx = min_dist_it;
                }
            }
            lg.addRightInformation(min_idx, stored_scan_msgs.ranges[min_idx]);
        }

        for (auto & rg : right_gap) 
        {
            int r_idx = rg.RIdx();
            float r_dist = rg.RDist();
            int min_idx = rg.LIdx();
            float l_dist = rg.LDist();
            float min_dist = pow(l_dist, 2) + pow(r_dist, 2) - 2 * l_dist * r_dist * std::cos((r_idx - min_idx) * stored_scan_msgs.angle_increment);
            for (int min_dist_it = r_idx - 1; min_dist_it > 0 && (int)(r_idx - min_dist_it) < 255; min_dist_it--)
            {
                float angle = ((float) (r_idx - min_dist_it))* stored_scan_msgs.angle_increment;
                float target_dist = stored_scan_msgs.ranges[min_dist_it];
                float dist = pow(r_dist, 2) + 
                    pow(target_dist, 2) - 
                    2 * r_dist * target_dist * std::cos(angle);
                if (dist <= min_dist)
                {
                    min_dist = dist;
                    min_idx = min_dist_it;
                }
            }
            rg.setLIdx(min_idx);
            rg.setLDist(stored_scan_msgs.ranges[min_idx]);
            // lg.addRightInformation(min_idx, stored_scan_msgs.ranges[min_idx]);
        }

        std::copy(left_gap.begin(), left_gap.end(), std::back_inserter(return_sets));
        std::copy(right_gap.begin(), right_gap.end(), std::back_inserter(return_sets));
        std::copy(central_gap.begin(), central_gap.end(), std::back_inserter(return_sets));
    }

    void GapUtils::safeGapMerge(std::vector<potential_gap::Gap>& observed_gaps) 
    {
        std::vector<potential_gap::Gap> second_gap;
        for (int i = 0; i < observed_gaps.size(); i++)
        {
            bool inside = false;
            int l_idx = observed_gaps[i].LIdx();
            int r_idx = observed_gaps[i].RIdx();
            float l_dist = observed_gaps[i].LDist();
            float r_dist = observed_gaps[i].RDist();
            for (int j = 0; j < observed_gaps.size(); j++)
            {
                if (j == i) continue;

                int tmp_l = observed_gaps[j].LIdx();
                int tmp_r = observed_gaps[j].RIdx();
                float tmp_ldist = observed_gaps[j].LDist();
                float tmp_rdist = observed_gaps[j].RDist();
                bool l = l_idx >= tmp_l && l_idx < tmp_r;
                bool r = r_idx > tmp_l && r_idx <= tmp_r;

                // When two gaps evaluated to have the same l and r index
                bool identical = l_idx == tmp_l && r_idx == tmp_r ? i < j : true;
                bool ldst = l_dist >= tmp_ldist || true;
                bool rdst = r_dist >= tmp_rdist || true;

                if (l && r && ldst && rdst && identical)
                {
                    ROS_DEBUG_STREAM("Gap: " << i <<" conflict with Gap " << j << "l: " << l << ", r: " << r);
                    ROS_DEBUG_STREAM("l_idx: " << l_idx << ", r_idx: " << r_idx << ", tmp_l: " << tmp_l << ", tmp_r: " << tmp_r);
                    inside = true;
                    break;
                } 
            }

            if (!inside)
            {
                second_gap.push_back(observed_gaps[i]);
            }
        }

        observed_gaps.clear();
        observed_gaps = second_gap;
    }


    void GapUtils::mergeGapsOneGo(
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<potential_gap::Gap>& observed_gaps)
    {
        // int left_idx = -1;
        // int right_idx = -1;
        // float right_dist = 3;
        // float left_dist = 3; // TODO: Make this reconfigurable
        int observed_size = (int) observed_gaps.size();
        std::vector<potential_gap::Gap> second_gap;

        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // Termination Condition
        // Smaller than first gap

        // Insert first
        bool mark_to_start = true;
        bool last_type_left = true;
        int left_counter = 0;
        bool changed = true;
        for (int i = 0; i < (int) observed_gaps.size(); i++)
        {
            if (mark_to_start && observed_gaps.at(i).isAxial() && observed_gaps.at(i).isLeftType())
            {
                // Wait until the first mergable gap aka swept left type gap
                mark_to_start = false;
                second_gap.push_back(observed_gaps[i]);
            } else {
                if (!mark_to_start)
                {
                    if (observed_gaps.at(i).isAxial())
                    {
                        if (observed_gaps.at(i).isLeftType())
                        {
                            second_gap.push_back(observed_gaps[i]);
                            // ROS_INFO_STREAM();
                        }
                        else
                        {
                            float curr_rdist = observed_gaps[i].RDist();
                            int erase_counter = 0;
                            int last_mergable = -1;

                            float coefs = cfg_->planning.planning_inflated ? 0 : 1;
                            for (int j = (int) (second_gap.size() - 1); j >= 0; j--)
                            {
                                int start_idx = std::min(second_gap[j].RIdx(), observed_gaps[i].LIdx());
                                int end_idx = std::max(second_gap[j].RIdx(), observed_gaps[i].LIdx());
                                auto farside_iter = std::min_element(stored_scan_msgs.ranges.begin() + start_idx, stored_scan_msgs.ranges.begin() + end_idx);
                                int farside_idx = farside_iter - stored_scan_msgs.ranges.begin();
                                // TODO: what number to use? Currently, use the max radius. The merging will not happen frequently.
                                // double max_r_er = robot_geo_proc_.getRobotMaxRadius();
                                double farside_angle = farside_idx * stored_scan_msgs.angle_increment + stored_scan_msgs.angle_min;
                                Eigen::Vector2d farside_vec(cos(farside_angle), sin(farside_angle));
                                Eigen::Vector2d orient_vec(1, 0);
                                double erl_rdist = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, farside_vec, curr_rdist);
                                double erl_ldist = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, farside_vec, second_gap[j].LDist());
                                bool second_test = curr_rdist <= (*farside_iter - coefs * erl_rdist) && second_gap[j].LDist() <= (*farside_iter - coefs * erl_ldist);
                                bool dist_diff = second_gap[j].isLeftType() || !second_gap[j].isAxial();
                                bool idx_diff = observed_gaps[i].RIdx() - second_gap[j].LIdx() < cfg_->gap_manip.max_idx_diff;
                                if (second_test && dist_diff && idx_diff) {
                                    last_mergable = j;
                                } 
                            }

                            if (last_mergable != -1) {
                                second_gap.erase(second_gap.begin() + last_mergable + 1, second_gap.end());
                                second_gap.back().addRightInformation(observed_gaps[i].RIdx(), observed_gaps[i].RDist());
                            } else {
                                second_gap.push_back(observed_gaps.at(i));
                            }
                        }
                    }
                    else
                    {
                        // If not axial gap, 
                        float curr_rdist = observed_gaps.at(i).RDist();
                        if (std::abs(curr_rdist - second_gap.back().LDist()) < 0.2 && second_gap.back().isAxial() && second_gap.back().isLeftType())
                        {
                            // ROS_INFO_STREAM("merge radial");
                            // second_gap.back().setRadial();
                            second_gap.back().addRightInformation(observed_gaps[i].RIdx(), observed_gaps[i].RDist());
                        } else {
                            second_gap.push_back(observed_gaps[i]);
                        }
                    }
                }
                else
                {
                    // A swept gap solely on its own
                    second_gap.push_back(observed_gaps[i]);
                }
            }
            last_type_left = observed_gaps[i].isLeftType();
        }

        // ROS_INFO_STREAM(second_gap.size());

        observed_gaps.clear();
        observed_gaps = second_gap;

    }


}
