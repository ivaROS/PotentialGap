#include <potential_gap/gap_utils.h>

namespace potential_gap {
    GapUtils::GapUtils() {}

    GapUtils::~GapUtils() {}

    GapUtils::GapUtils(const PotentialGapConfig& cfg) {
        cfg_ = & cfg;
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
        int gap_size = 0;
        std::string frame = stored_scan_msgs.header.frame_id;
        int gap_lidx = 0;
        float gap_ldist = stored_scan_msgs.ranges[0];
        float last_scan = stored_scan_msgs.ranges[0];
        bool prev_lgap = gap_ldist >= max_scan_dist;
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
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || cfg_->planning.planning_inflated) observed_gaps.push_back(detected_gap);
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
                    if (detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr || 
                        cfg_->planning.planning_inflated) observed_gaps.push_back(detected_gap);
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
            potential_gap::Gap detected_gap(frame, gap_lidx, gap_ldist, half_scan);
            detected_gap.addRightInformation(int(stored_scan_msgs.ranges.size() - 1), *(stored_scan_msgs.ranges.end() - 1));
            detected_gap.setMinSafeDist(min_dist);
            if (detected_gap._right_idx - detected_gap._left_idx > 500 || detected_gap.get_dist_side() > 2 * cfg_->rbt.r_inscr) observed_gaps.push_back(detected_gap);
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
                int total_size = (int)(stored_scan_msgs.ranges.size() - 1) - end_side_idx + start_side_idx;
                float result = (end_side_dist - start_side_dist) * (float (start_side_idx) / float (total_size)) + start_side_dist;
                observed_gaps[observed_gaps.size() - 1].addRightInformation(511, result);
                observed_gaps[observed_gaps.size() - 1].mode.wrap = true;
                observed_gaps[0]._ldist = result;
                observed_gaps[0].mode.wrap = true;;
            }
        }
    }

    void GapUtils::mergeGapsOneGo(
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser,
        std::vector<potential_gap::Gap>& observed_gaps)
    {
        int observed_size = (int) observed_gaps.size();
        std::vector<potential_gap::Gap> second_gap;

        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        // Termination Condition

        // Insert first
        bool mark_to_start = true;
        bool last_type_left = true;
        int left_counter = 0;
        bool changed = true;
        // Forward Pass
        for (int i = 0; i < (int) observed_gaps.size(); i++)
        {
            if (mark_to_start && observed_gaps.at(i).isRadial() && observed_gaps.at(i).isLeftType())
            {
                // Wait until the first mergable gap aka swept left type gap
                mark_to_start = false;
                second_gap.push_back(observed_gaps[i]);
            } else {
                if (!mark_to_start)
                {
                    if (observed_gaps.at(i).isRadial())
                    {
                        if (observed_gaps.at(i).isLeftType())
                        {
                            second_gap.push_back(observed_gaps[i]);
                        }
                        else
                        {
                            float curr_rdist = observed_gaps[i].RDist();
                            int erase_counter = 0;
                            int last_mergable = -1;

                            float coefs = cfg_->planning.planning_inflated ? 0 : 2;
                            for (int j = (int) (second_gap.size() - 1); j >= 0; j--)
                            {
                                int start_idx = std::min(second_gap[j].RIdx(), observed_gaps[i].LIdx());
                                int end_idx = std::max(second_gap[j].RIdx(), observed_gaps[i].LIdx());
                                auto farside_iter = std::min_element(stored_scan_msgs.ranges.begin() + start_idx, stored_scan_msgs.ranges.begin() + end_idx);
                                bool second_test = curr_rdist <= (*farside_iter - coefs * cfg_->rbt.r_inscr) && second_gap[j].LDist() <= (*farside_iter - coefs * cfg_->rbt.r_inscr);
                                bool dist_diff = second_gap[j].isLeftType() || !second_gap[j].isRadial();
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
                        if (std::abs(curr_rdist - second_gap.back().LDist()) < 0.2 && second_gap.back().isRadial() && second_gap.back().isLeftType())
                        {
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

        // Close up backwards radial gaps
        // Unmergable
        float threshold_val = cfg_->rbt.r_inscr;
        int unmerg_idx = -1;
        float back_gap_dist = stored_scan_msgs.range_max;
        float last_dist = stored_scan_msgs.range_max;
        for (int i = 0; i < second_gap.size(); i++){
            if (second_gap.at(i).isLeftType() && second_gap.at(i).isRadial() && !second_gap.at(i).mode.wrap
                || second_gap.at(i)._right_idx > float(stored_scan_msgs.ranges.size()) * 0.25
                || !(second_gap.at(i)._rdist - last_dist < threshold_val)) {
                break;
            } else {
                unmerg_idx = i;
            }
        }
        
        int back_unmerg_idx = -1;
        last_dist = stored_scan_msgs.range_max;
        for (int i = second_gap.size() - 1; i >= 0; i--) {
            if (!second_gap.at(i).isLeftType() && second_gap.at(i).isRadial() && !second_gap.at(i).mode.wrap
                || second_gap.at(i)._left_idx < float(stored_scan_msgs.ranges.size()) * 0.75 
                || !(second_gap.at(i)._ldist - last_dist < threshold_val)) {
                break;
            } else {
                last_dist = second_gap.at(i)._ldist;
                back_unmerg_idx = i;
            }
        }

        // Process gotta be iteratively backwards
        while (unmerg_idx >= 0 && back_unmerg_idx < second_gap.size()) {
            auto min_dist_f = *std::min_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.begin() + second_gap.at(unmerg_idx)._right_idx - 1);
            auto min_dist_b = *std::min_element(stored_scan_msgs.ranges.begin() + second_gap.at(back_unmerg_idx)._left_idx + 1, stored_scan_msgs.ranges.end());
            auto min_dist_rev = std::min(min_dist_f, min_dist_b);
            if (min_dist_rev > second_gap.at(unmerg_idx)._rdist && min_dist_rev > second_gap.at(back_unmerg_idx)._ldist) {
                float forward_r = second_gap.at(unmerg_idx)._right_idx + (int) stored_scan_msgs.ranges.size() - second_gap.at(back_unmerg_idx)._left_idx;
                float front_lr = second_gap.at(unmerg_idx)._rdist + (second_gap.at(back_unmerg_idx)._ldist - second_gap.at(unmerg_idx)._rdist) * second_gap.at(unmerg_idx)._right_idx / forward_r;
                second_gap.at(unmerg_idx)._left_idx = 0;
                second_gap.at(unmerg_idx)._ldist = front_lr;
                second_gap.at(unmerg_idx).convex.convex_lidx = 0;
                second_gap.at(unmerg_idx).convex.convex_ldist = front_lr;
                second_gap.at(back_unmerg_idx)._right_idx = stored_scan_msgs.ranges.size() - 1;
                second_gap.at(back_unmerg_idx)._rdist = front_lr;
                second_gap.at(back_unmerg_idx).convex.convex_ridx = stored_scan_msgs.ranges.size() - 1;
                second_gap.at(back_unmerg_idx).convex.convex_rdist = front_lr;
                second_gap.erase(second_gap.begin() + back_unmerg_idx + 1, second_gap.end());
                second_gap.erase(second_gap.begin(), second_gap.begin() + unmerg_idx);
                second_gap.begin()->mode.wrap = true;
                (second_gap.end() - 1)->mode.wrap = true;
                break;
            } else if (min_dist_rev < second_gap.at(back_unmerg_idx)._ldist) {
                unmerg_idx--;
            } else if (min_dist_rev < second_gap.at(unmerg_idx)._rdist) {
                back_unmerg_idx++;
            }
        }
        
        observed_gaps.clear();
        observed_gaps = second_gap;
    }

}