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
        observed_gaps.clear();
        observed_gaps = second_gap;

    }


}