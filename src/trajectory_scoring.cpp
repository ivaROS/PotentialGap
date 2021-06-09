#include <potential_gap/trajectory_scoring.h>

namespace potential_gap {
    TrajectoryArbiter::TrajectoryArbiter(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg)
    {
        cfg_ = & cfg;
        r_inscr = cfg_->rbt.r_inscr;
        rmax = cfg_->traj.rmax;
        cobs = cfg_->traj.cobs;
        w = cfg_->traj.w;
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

    void TrajectoryArbiter::updateEgocircleCalib(geometry_msgs::PoseStamped rbt_in_cam) {
        boost::mutex::scoped_lock lock(egocircle_mutex);
        rbt_in_cam_lc = rbt_in_cam;
    }


    void TrajectoryArbiter::updateLocalGoal(geometry_msgs::PoseStamped lg, geometry_msgs::TransformStamped odom2rbt) {
        boost::mutex::scoped_lock lock(gplan_mutex);
        tf2::doTransform(lg, local_goal, odom2rbt);
    }

    // rbt frame
    std::vector<double> TrajectoryArbiter::scoreGaps()
    {
        boost::mutex::scoped_lock planlock(gplan_mutex);
        boost::mutex::scoped_lock egolock(egocircle_mutex);
        if (gaps.size() < 1) {
            ROS_WARN_STREAM("Observed num of gap: 0");
            return std::vector<double>(0);
        }

        int num_of_scan = msg.get()->ranges.size();
        double goal_orientation = std::atan2(local_goal.pose.position.y, local_goal.pose.position.x);
        int idx = goal_orientation / (M_PI / (num_of_scan / 2)) + (num_of_scan / 2);
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
    [[deprecated("Implementation removed, use scoreTrajectory instead.")]]
    std::vector<double> TrajectoryArbiter::scoreTrajectories (
        std::vector<geometry_msgs::PoseArray> sample_traj) {
        return std::vector<double>(sample_traj.size());
    }

    std::vector<double> TrajectoryArbiter::scoreTrajectory(geometry_msgs::PoseArray traj) {
        // Requires LOCAL FRAME
        // No racing condition
        std::vector<double> cost_val(traj.poses.size());

        for (int i = 0; i < cost_val.size(); i++) {
            cost_val.at(i) = scorePose(traj.poses.at(i));
        }

        auto total_val = std::accumulate(cost_val.begin(), cost_val.end(), double(0));

        if (cost_val.size() > 0)
        {
            auto terminal_cost = 10 * terminalGoalCost(*std::prev(traj.poses.end()));
            if (terminal_cost < 1 && total_val > -10) return std::vector<double>(traj.poses.size(), 100);
            cost_val.at(0) -= terminal_cost;
        }
        
        return cost_val;
    }

    double TrajectoryArbiter::terminalGoalCost(geometry_msgs::Pose pose) {
        boost::mutex::scoped_lock planlock(gplan_mutex);
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

        double pose_ori = std::atan2(pose.position.y + 1e-3, pose.position.x + 1e-3);
        int center_idx = (int) std::round((pose_ori + M_PI) / msg.get()->angle_increment);
        
        int scan_size = (int) stored_scan.ranges.size();
        std::vector<double> dist(scan_size);

        // This size **should** be ensured
        if (stored_scan.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect scorePose");
        }


        for (int i = 0; i < dist.size(); i++) {
            float this_dist = stored_scan.ranges.at(i);
            float x, y, angle;

            x = this_dist * cos(i * stored_scan.angle_increment - M_PI);
            y = this_dist * sin(i * stored_scan.angle_increment - M_PI);
            // in camera frame
            x -= rbt_in_cam_lc.pose.position.x;
            y -= rbt_in_cam_lc.pose.position.y;
            this_dist = sqrt(pow(x, 2) + pow(y, 2));
            angle = atan2(y, x);
                        
            this_dist = this_dist == 3 ? this_dist + cfg_->traj.rmax : this_dist;
            dist.at(i) = dist2Pose(angle,
                this_dist, pose);
        }

        auto iter = std::min_element(dist.begin(), dist.end());
        return chapterScore(*iter);
    }

    double TrajectoryArbiter::chapterScore(double d) {
        // Or alternatively, -std::numeric_limits<double>::infinity()
        // This serves as the perception space collision checking
        if (d < r_inscr * cfg_->traj.inf_ratio) return -1000;
        if (d > rmax) return 0;
        return cobs * std::exp(- w * (d - r_inscr * cfg_->traj.inf_ratio));
    }

    int TrajectoryArbiter::searchIdx(geometry_msgs::Pose pose) {
        if (!msg) return 1;
        double r = sqrt(pow(pose.position.x, 2) + pow(pose.position.y, 2));
        double eval = double(cfg_->rbt.r_inscr) / r;
        if (eval > 1) return 1;
        float theta = float(std::acos( eval ));
        int searchIdx = (int) std::ceil(theta / msg.get()->angle_increment);
        return searchIdx;
    }

    [[deprecated("Score Trajectory instead.")]]
    potential_gap::Gap TrajectoryArbiter::returnAndScoreGaps() {
        boost::mutex::scoped_lock gaplock(gap_mutex);
        std::vector<double> cost = scoreGaps();
        auto decision_iter = std::min_element(cost.begin(), cost.end());
        int gap_idx = std::distance(cost.begin(), decision_iter);
        auto selected_gap = gaps.at(gap_idx);
        return selected_gap;
    }

    
}