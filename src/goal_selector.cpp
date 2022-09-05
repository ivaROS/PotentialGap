#include <potential_gap/goal_selector.h>

namespace potential_gap {
    GoalSelector::GoalSelector(ros::NodeHandle& nh,
    const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc) {
        cfg_ = &cfg;
        robot_geo_proc_ = robot_geo_proc;
    }

    bool GoalSelector::setGoal(
        const std::vector<geometry_msgs::PoseStamped> &plan) {
        // Incoming plan is in map frame
        boost::mutex::scoped_lock lock(goal_select_mutex);
        boost::mutex::scoped_lock gplock(gplan_mutex);
        global_plan.clear();
        global_plan = plan;
        // transform plan to robot frame such as base_link
        return true;
    }

    void GoalSelector::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg) {
        boost::mutex::scoped_lock lock(lscan_mutex);
        sharedPtr_laser = msg;
    }

    void GoalSelector::updateLocalGoal(geometry_msgs::TransformStamped map2rbt) {
        if (global_plan.size() < 2) {
            // No Global Goal
            return;
        }

        if ((*sharedPtr_laser.get()).ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect goalselector");
        }

        boost::mutex::scoped_lock glock(goal_select_mutex);
        boost::mutex::scoped_lock llock(lscan_mutex);
        auto local_gplan = getRelevantGlobalPlan(map2rbt);
        if (local_gplan.size() < 1) return;

        auto result_rev = std::find_if(local_gplan.rbegin(), local_gplan.rend(), 
            std::bind1st(std::mem_fun(&GoalSelector::VisibleOrPossiblyObstructed), this));
        auto result_fwd = std::find_if(local_gplan.begin(), local_gplan.end(), 
            std::bind1st(std::mem_fun(&GoalSelector::NoTVisibleOrPossiblyObstructed), this));

        if (cfg_->planning.far_feasible) {
            if (result_rev == local_gplan.rend()) result_rev = std::prev(result_rev); 
            local_goal = *result_rev;
        } else {
            result_fwd = result_fwd == local_gplan.end() ? result_fwd - 1 : result_fwd;
            local_goal = local_gplan.at(result_fwd - local_gplan.begin());
        }
    }

    bool GoalSelector::NoTVisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose) {
        // If all poses are within the egocircle, this will return the end of the plan.
        int laserScanIdx = PoseIndexInSensorMsg(pose);
        // float epsilon2 = float(cfg_->gap_manip.epsilon2);
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        Eigen::Vector2d pose_vec(pose.pose.position.x, pose.pose.position.y);
        Eigen::Vector2d orient_vec(1, 0);
        double buffer_length = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, pose_vec, pose_vec.norm());
        bool check = dist2rbt(pose) >= (double (stored_scan_msgs.ranges.at(laserScanIdx)) - buffer_length);
        // bool check = dist2rbt(pose) >= (double (stored_scan_msgs.ranges.at(laserScanIdx)));
        return check;
    }

    bool GoalSelector::VisibleOrPossiblyObstructed(geometry_msgs::PoseStamped pose) {
        int laserScanIdx = PoseIndexInSensorMsg(pose);
        float epsilon2 = float(cfg_->gap_manip.epsilon2);
        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        Eigen::Vector2d pose_vec(pose.pose.position.x, pose.pose.position.y);
        Eigen::Vector2d orient_vec(1, 0);
        double buffer_length = robot_geo_proc_.getLinearDecayEquivalentRL(orient_vec, pose_vec, pose_vec.norm());
        bool check = dist2rbt(pose) < (double (stored_scan_msgs.ranges.at(laserScanIdx)) - buffer_length) || 
            dist2rbt(pose) > (double (stored_scan_msgs.ranges.at(laserScanIdx)) + epsilon2 * 2);
        // bool check = dist2rbt(pose) < (double (stored_scan_msgs.ranges.at(laserScanIdx))) || 
        //     dist2rbt(pose) > (double (stored_scan_msgs.ranges.at(laserScanIdx)) + epsilon2 * 2);
        return check;
    }

    int GoalSelector::PoseIndexInSensorMsg(geometry_msgs::PoseStamped pose) {
        auto orientation = getPoseOrientation(pose);
        auto index = float(orientation + M_PI) / (sharedPtr_laser.get()->angle_increment);
        return int(std::floor(index));
    }

    double GoalSelector::getPoseOrientation(geometry_msgs::PoseStamped pose) {
        return  std::atan2(pose.pose.position.y + 1e-3, pose.pose.position.x + 1e-3);
    }

    std::vector<geometry_msgs::PoseStamped> GoalSelector::getRelevantGlobalPlan(geometry_msgs::TransformStamped map2rbt) {
        // Global Plan is now in robot frame
        // Do magic with egocircle
        boost::mutex::scoped_lock gplock(gplan_mutex);
        mod_plan.clear();
        mod_plan = global_plan;


        if (mod_plan.size() == 0) {
            ROS_FATAL_STREAM("Global Plan Length = 0");
        }

        for (int i = 0; i < mod_plan.size(); i++) {
            tf2::doTransform(mod_plan.at(i), mod_plan.at(i), map2rbt);
        }

        std::vector<double> distance(mod_plan.size());
        for (int i = 0; i < distance.size(); i++) {
            distance.at(i) = dist2rbt(mod_plan.at(i));
        }

        sensor_msgs::LaserScan stored_scan_msgs = *sharedPtr_laser.get();
        threshold = (double) *std::max_element(stored_scan_msgs.ranges.begin(), stored_scan_msgs.ranges.end());

        // Find closest pose to robot
        // TODO: may need to improve
        auto start_pose = std::min_element(distance.begin(), distance.end());
        auto end_pose = std::find_if(start_pose, distance.end(),
            std::bind1st(std::mem_fun(&GoalSelector::isNotWithin), this));

        if (start_pose == distance.end()) {
            ROS_FATAL_STREAM("No Global Plan pose within Robot scan");
            return std::vector<geometry_msgs::PoseStamped>(0);
        } else if (mod_plan.size() == 0) {
            ROS_WARN_STREAM("No Global Plan Received or Size 0");
            return std::vector<geometry_msgs::PoseStamped>(0);
        }

        int start_idx = std::distance(distance.begin(), start_pose);
        int end_idx = std::distance(distance.begin(), end_pose);

        auto start_gplan = mod_plan.begin() + start_idx;
        auto end_gplan = mod_plan.begin() + end_idx;

        std::vector<geometry_msgs::PoseStamped> local_gplan(start_gplan, end_gplan);
        return local_gplan;
    }

    double GoalSelector::dist2rbt(geometry_msgs::PoseStamped pose) {
        return sqrt(pow(pose.pose.position.x, 2) + pow(pose.pose.position.y, 2));
    }

    bool GoalSelector::isNotWithin(const double dist) {
        return dist > threshold;
    }

    geometry_msgs::PoseStamped GoalSelector::getCurrentLocalGoal(geometry_msgs::TransformStamped rbt2odom) {
        geometry_msgs::PoseStamped result;
        tf2::doTransform(local_goal, result, rbt2odom);
        // This should return something in odom frame
        return result;
    }

    std::vector<geometry_msgs::PoseStamped> GoalSelector::getRawGlobalPlan() {
        return global_plan;
    }


}