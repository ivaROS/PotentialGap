#include <potential_gap/trajectory_scoring.h>

namespace potential_gap {
    TrajectoryArbiter::TrajectoryArbiter(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc)
    {
        cfg_ = & cfg;
        r_inscr = cfg_->rbt.r_inscr;
        rmax = cfg_->traj.rmax;
        cobs = cfg_->traj.cobs;
        w = cfg_->traj.w;
        terminal_weight = cfg_->traj.terminal_weight;
        robot_geo_proc_ = robot_geo_proc;
    }

    void TrajectoryArbiter::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg_) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        msg = msg_;
    }
    void TrajectoryArbiter::updateGapContainer(const std::vector<potential_gap::Gap> observed_gaps) {
        boost::mutex::scoped_lock lock(gap_mutex);
        gaps.clear();
        gaps = observed_gaps;
    }

    void TrajectoryArbiter::updateLocalGoal(geometry_msgs::PoseStamped lg, geometry_msgs::TransformStamped odom2rbt) {
        boost::mutex::scoped_lock lock(gplan_mutex);
        tf2::doTransform(lg, local_goal, odom2rbt);
    }

    // Does things in rbt frame
    std::vector<double> TrajectoryArbiter::scoreGaps()
    {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        boost::mutex::scoped_lock egolock(egocircle_mutex);
        if (gaps.size() < 1) {
            ROS_WARN_STREAM("Observed num of gap: 0");
            return std::vector<double>(0);
        }

        // How fix this
        int num_of_scan = msg.get()->ranges.size();
        double goal_orientation = std::atan2(local_goal.pose.position.y, local_goal.pose.position.x);
        int idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);
        ROS_DEBUG_STREAM("Goal Orientation: " << goal_orientation << ", idx: " << idx);
        ROS_DEBUG_STREAM(local_goal.pose.position);
        auto costFn = [](potential_gap::Gap g, int goal_idx) -> double
        {
            int leftdist = std::abs(g._left_idx - goal_idx);
            int rightdist = std::abs(g._right_idx - goal_idx);
            return std::min(leftdist, rightdist);
        };

        std::vector<double> cost(gaps.size());
        for (int i = 0; i < cost.size(); i++) 
        {
            cost.at(i) = costFn(gaps.at(i), idx);
        }

        return cost;
    }

    // Again, in rbt frame
    std::vector<double> TrajectoryArbiter::scoreTrajectories (
        std::vector<geometry_msgs::PoseArray> sample_traj) {
        // This will be in robot frame
        
        return std::vector<double>(sample_traj.size());
    }

    std::vector<double> TrajectoryArbiter::scoreTrajectory(geometry_msgs::PoseArray traj) {
        // Requires LOCAL FRAME
        // Should be no racing condition
        std::vector<double> cost_val(traj.poses.size());
        for (int i = 0; i < cost_val.size(); i++) {
            cost_val.at(i) = scorePose(traj.poses.at(i));
        }

        auto total_val = std::accumulate(cost_val.begin(), cost_val.end(), double(0));

        if (cost_val.size() > 0) // && ! cost_val.at(0) == -std::numeric_limits<double>::infinity())
        {
            auto terminal_cost = terminal_weight * terminalGoalCost(*std::prev(traj.poses.end()));
            if (terminal_cost < 1 && total_val > -10) return std::vector<double>(traj.poses.size(), 100);
            // Should be safe
            cost_val.at(0) -= terminal_cost;
        }
        
        return cost_val;
    }

    double TrajectoryArbiter::terminalGoalCost(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        // ROS_INFO_STREAM(pose);
        double dx = pose.position.x - local_goal.pose.position.x;
        double dy = pose.position.y - local_goal.pose.position.y;
        return sqrt(pow(dx, 2) + pow(dy, 2));
    }

    double TrajectoryArbiter::dist2Pose(float theta, float dist, geometry_msgs::Pose pose) {
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }

    double TrajectoryArbiter::scorePose(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        sensor_msgs::LaserScan stored_scan = *msg.get();

        // double pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        // int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) stored_scan.ranges.size();
        std::vector<double> dist(scan_size);
        std::vector<double> rmax_offset(scan_size);

        // This size **should** be ensured
        if (stored_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }

        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        Eigen::Vector2d orient_vec(cos(euler[2]), sin(euler[2]));
        Eigen::Vector2d pose_vec(pose.position.x, pose.position.y); // TODO: pose should be in robot frame
        double pose_angle = atan2(pose_vec[1], pose_vec[0]);
        int pose_idx = int(round((pose_angle - stored_scan.angle_min) / stored_scan.angle_increment));
        pose_idx = pose_idx >= 0 ? pose_idx : 0;
        pose_idx = pose_idx < stored_scan.ranges.size() ? pose_idx : (stored_scan.ranges.size() - 1);
        float pose_ego_dist = stored_scan.ranges[pose_idx];
        if(pose_vec.norm() >= pose_ego_dist)
            return -std::numeric_limits<double>::infinity();

        for (int i = 0; i < dist.size(); i++) {
            float this_dist = stored_scan.ranges.at(i);
            this_dist = this_dist == 3 ? this_dist + cfg_->traj.rmax : this_dist;

            // Iterate through robot boundary
            
            double pt_ang = i * stored_scan.angle_increment - M_PI;
            pt_ang = (pt_ang >= -M_PI) ? pt_ang : -M_PI;
            pt_ang = (pt_ang <= M_PI) ? pt_ang : M_PI;
            Eigen::Vector2d pt_vec(cos(pt_ang), sin(pt_ang));
            pt_vec = this_dist * pt_vec;
            
            Eigen::Vector2d rel_pt_vec = pt_vec - pose_vec;
            double nearest_dist = robot_geo_proc_.getNearestDistance(orient_vec, rel_pt_vec);
            dist.at(i) = nearest_dist;
            // ROS_INFO_STREAM(dist.at(i));
            // rmax_offset.at(i) = rmax - robot_geo_proc_.getRobotMaxRadius() * cfg_->traj.inf_ratio;
            
            // Get the robot equivalent radius
            
            // Eigen::Vector2d pose_position_vec(pose.position.x, pose.position.y);
            // Eigen::Vector2d scan_pt_vec(this_dist * cos(i * stored_scan.angle_increment - M_PI), this_dist * sin(i * stored_scan.angle_increment - M_PI));
            // Eigen::Vector2d relative_vec = scan_pt_vec - pose_position_vec;
            // relative_vec = relative_vec / relative_vec.norm();
            // double robot_er = robot_geo_proc_.getEquivalentR(orient_vec, relative_vec);
            // dist.at(i) = dist2Pose(i * stored_scan.angle_increment - M_PI,
            //     this_dist, pose);
            // dist.at(i) -= robot_er * cfg_->traj.inf_ratio;
            // rmax_offset.at(i) = rmax - robot_er * cfg_->traj.inf_ratio;
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        // double rmax_offset_val = rmax_offset[iter - dist.begin()];
        double rmax_offset_val = rmax - robot_geo_proc_.getRobotMaxRadius() * cfg_->traj.inf_ratio;
        return chapterScore(*iter, rmax_offset_val);
    }

    double TrajectoryArbiter::chapterScore(double d, double rmax_offset_val) {
        if (d <= 0) return -std::numeric_limits<double>::infinity();
        if (d > rmax_offset_val) return 0;
        return cobs * std::exp(- w * (d));
    }

    // int TrajectoryArbiter::searchIdx(geometry_msgs::Pose pose) {
    //     if (!msg) return 1;
    //     double r = sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2));
    //     double eval = double(cfg_->rbt.r_inscr) / r;
    //     if (eval > 1) return 1;
    //     float theta = float(std::acos( eval ));
    //     int searchIdx = (int) std::ceil(theta / msg.get()->angle_increment);
    //     return searchIdx;
    // }

    potential_gap::Gap TrajectoryArbiter::returnAndScoreGaps() {
        boost::mutex::scoped_lock gaplock(gap_mutex);
        std::vector<double> cost = scoreGaps();
        auto decision_iter = std::min_element(cost.begin(), cost.end());
        int gap_idx = std::distance(cost.begin(), decision_iter);
        // ROS_INFO_STREAM("Selected Gap Index " << gap_idx);
        auto selected_gap = gaps.at(gap_idx);
        return selected_gap;
    }

    
}