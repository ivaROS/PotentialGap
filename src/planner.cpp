#include <potential_gap/planner.h>
#include "tf/transform_datatypes.h"
#include <tf/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace potential_gap
{
    Planner::Planner()
    {
        // Do something? maybe set names
        ros::NodeHandle nh("planner_node");
    }

    Planner::~Planner() {}

    bool Planner::initialize(const ros::NodeHandle& unh)
    {
        if (initialized())
        {
            ROS_WARN("PotentialGap Planner already initalized");
            return true;
        }

        // Config Setup
        cfg.loadRosParamFromNodeHandle(unh);

        // Visualization Setup
        // Fix this later
        local_traj_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_traj", 500);
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("pg_traj", 10);
        gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("gaps", 1);
        selected_gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("sel_gaps", 1);
        ni_traj_pub = nh.advertise<geometry_msgs::PoseArray>("ni_traj", 10);
        ni_traj_pub_other = nh.advertise<visualization_msgs::MarkerArray>("other_ni_traj", 5);

        // TF Lookup setup
        tfListener = new tf2_ros::TransformListener(tfBuffer);
        _initialized = true;

        finder = new potential_gap::GapUtils(cfg);
        gapvisualizer = new potential_gap::GapVisualizer(nh, cfg);
        goalselector = new potential_gap::GoalSelector(nh, cfg);
        trajvisualizer = new potential_gap::TrajectoryVisualizer(nh, cfg);
        trajArbiter = new potential_gap::TrajectoryArbiter(nh, cfg);
        gapTrajSyn = new potential_gap::GapTrajGenerator(nh, cfg);
        goalvisualizer = new potential_gap::GoalVisualizer(nh, cfg);
        gapManip = new potential_gap::GapManipulator(nh, cfg);
        trajController = new potential_gap::TrajectoryController(nh, cfg);

        map2rbt.transform.rotation.w = 1;
        rbt2map.transform.rotation.w = 1;
        odom2rbt.transform.rotation.w = 1;
        rbt2odom.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg.robot_frame_id;

        log_vel_comp.set_capacity(cfg.planning.halt_size);
        return true;
    }

    bool Planner::initialized()
    {
        return _initialized;
    }

    bool Planner::isGoalReached()
    {
        current_pose_ = sharedPtr_pose;
        double dx = final_goal_odom.pose.position.x - current_pose_.position.x;
        double dy = final_goal_odom.pose.position.y - current_pose_.position.y;
        bool result = sqrt(pow(dx, 2) + pow(dy, 2)) < cfg.goal.goal_tolerance;
        if (result)
        {
            ROS_INFO_STREAM("[Reset] Goal Reached");
            return true;
        }

        double waydx = local_waypoint_odom.pose.position.x - current_pose_.position.x;
        double waydy = local_waypoint_odom.pose.position.y - current_pose_.position.y;
        bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) < cfg.goal.waypoint_tolerance;
        if (wayres) {
            ROS_INFO_STREAM("[Reset] Waypoint reached, getting new one");
            // global_plan_location += global_plan_lookup_increment;
        }
        return false;
    }

    void Planner::inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_inflatedlaser = msg;
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sharedPtr_laser = msg;

        if (cfg.planning.planning_inflated && sharedPtr_inflatedlaser) {
            msg = sharedPtr_inflatedlaser;
        }

        // ROS_INFO_STREAM(msg.get()->ranges.size());

        try {
            boost::mutex::scoped_lock gapset(gapset_mutex);
            finder->hybridScanGap(msg, observed_gaps);
            gapvisualizer->drawGaps(observed_gaps, std::string("raw"));
            finder->mergeGapsOneGo(msg, observed_gaps);
            gapvisualizer->drawGaps(observed_gaps, std::string("fin"));
            // ROS_INFO_STREAM("observed_gaps count:" << observed_gaps.size());
        } catch (...) {
            ROS_FATAL_STREAM("mergeGapsOneGo");
        }

        boost::shared_ptr<sensor_msgs::LaserScan const> tmp;
        if (sharedPtr_inflatedlaser) {
            tmp = sharedPtr_inflatedlaser;
        } else {
            tmp = msg;
        }

        geometry_msgs::PoseStamped local_goal;
        {
            goalselector->updateEgoCircle(tmp);
            goalselector->updateLocalGoal(map2rbt);
            local_goal = goalselector->getCurrentLocalGoal(rbt2odom);
            goalvisualizer->localGoal(local_goal);
        }
        
        trajArbiter->updateEgoCircle(msg);
        trajArbiter->updateLocalGoal(local_goal, odom2rbt);

        gapManip->updateEgoCircle(msg);
        trajController->updateEgoCircle(msg);

    }

    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Transform the msg to odom frame
        if(msg->header.frame_id != cfg.odom_frame_id)
        {
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer.lookupTransform(cfg.odom_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            sharedPtr_pose = out_pose.pose;
        }
        else
        {
            sharedPtr_pose = msg->pose.pose;
        }
    }

    bool Planner::setGoal(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (plan.size() == 0) return true;
        final_goal_odom = *std::prev(plan.end());
        tf2::doTransform(final_goal_odom, final_goal_odom, map2odom);

        // Store New Global Plan to Goal Selector
        goalselector->setGoal(plan);
        
        trajvisualizer->globalPlanRbtFrame(goalselector->getOdomGlobalPlan());

        // Find Local Goal
        goalselector->updateLocalGoal(map2rbt);
        // return local goal (odom) frame
        auto new_local_waypoint = goalselector->getCurrentLocalGoal(rbt2odom);

        {
            // Plan New
            double waydx = local_waypoint_odom.pose.position.x - new_local_waypoint.pose.position.x;
            double waydy = local_waypoint_odom.pose.position.y - new_local_waypoint.pose.position.y;
            bool wayres = sqrt(pow(waydx, 2) + pow(waydy, 2)) > cfg.goal.waypoint_tolerance;
            if (wayres) {
                local_waypoint_odom = new_local_waypoint;
            }
        }

        // Set new local goal to trajectory arbiter
        trajArbiter->updateLocalGoal(local_waypoint_odom, odom2rbt);

        // Visualization only
        try { 
            auto traj = goalselector->getRelevantGlobalPlan(map2rbt);
            geometry_msgs::PoseArray pub_traj;
            if (traj.size() > 0) {
                // Should be safe with this check
                pub_traj.header = traj.at(0).header;
            }
            for (auto trajpose : traj) {
                pub_traj.poses.push_back(trajpose.pose);
            }
            local_traj_pub.publish(pub_traj);
        } catch (...) {
            ROS_FATAL_STREAM("getRelevantGlobalPlan");
        }

        return true;
    }

    void Planner::updateTF()
    {
        try {
            map2rbt  = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2map  = tfBuffer.lookupTransform(cfg.map_frame_id, cfg.robot_frame_id, ros::Time(0));
            odom2rbt = tfBuffer.lookupTransform(cfg.robot_frame_id, cfg.odom_frame_id, ros::Time(0));
            rbt2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.robot_frame_id, ros::Time(0));
            cam2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.sensor_frame_id, ros::Time(0));
            map2odom = tfBuffer.lookupTransform(cfg.odom_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2cam = tfBuffer.lookupTransform(cfg.sensor_frame_id, cfg.robot_frame_id, ros::Time(0));

            tf2::doTransform(rbt_in_rbt, rbt_in_cam, rbt2cam);
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            return;
        }
    }

    [[deprecated("Use Proper trajectory scoring instead")]]
    void Planner::vectorSelectGap(potential_gap::Gap & selected_gap)
    {
        potential_gap::Gap result = trajArbiter->returnAndScoreGaps();
        selected_gap = result;
        return;
    }

    std::vector<potential_gap::Gap> Planner::gapManipulate() {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<potential_gap::Gap> manip_set;
        manip_set = observed_gaps;

        try {
            for (size_t i = 0; i < manip_set.size(); i++)
            {
                gapManip->reduceGap(manip_set.at(i), goalselector->rbtFrameLocalGoal());
                gapManip->convertAxialGap(manip_set.at(i));
                gapManip->radialExtendGap(manip_set.at(i));
                gapManip->setGapWaypoint(manip_set.at(i), goalselector->rbtFrameLocalGoal());
            }
        } catch(...) {
            ROS_FATAL_STREAM("gapManipulate");
        }

        goalvisualizer->drawGapGoals(manip_set);
        gapvisualizer->drawManipGaps(manip_set);
        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<double>> Planner::initialTrajGen(std::vector<potential_gap::Gap> vec, std::vector<geometry_msgs::PoseArray>& res) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<geometry_msgs::PoseArray> ret_traj(vec.size());
        std::vector<std::vector<double>> ret_traj_scores(vec.size());
        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam; // lc as local copy
        try {
            for (size_t i = 0; i < vec.size(); i++) {
                auto tmp = gapTrajSyn->generateTrajectory(vec.at(i), rbt_in_cam_lc);
                tmp = gapTrajSyn->forwardPassTrajectory(tmp);
                ret_traj_scores.at(i) = trajArbiter->scoreTrajectory(tmp);
                ret_traj.at(i) = gapTrajSyn->transformBackTrajectory(tmp, cam2odom);
            }
        } catch (...) {
            ROS_FATAL_STREAM("initialTrajGen");
        }

        trajvisualizer->pubAllScore(ret_traj, ret_traj_scores);
        trajvisualizer->pubAllTraj(ret_traj);
        res = ret_traj;
        return ret_traj_scores;
    }

    geometry_msgs::PoseArray Planner::pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<double>> score) {
        ROS_INFO_STREAM_NAMED("pg_trajCount", "pg_trajCount, " << prr.size());
        if (prr.size() == 0) {
            ROS_WARN_STREAM("No traj synthesized");
            return geometry_msgs::PoseArray();
        }

        if (prr.size() != score.size()) {
            ROS_FATAL_STREAM("pickTraj size mismatch: prr = " << prr.size() << " != score =" << score.size());
            return geometry_msgs::PoseArray();
        }

        std::vector<double> result_score(prr.size());
        
        try {
            if (omp_get_dynamic()) omp_set_dynamic(0);
            for (size_t i = 0; i < result_score.size(); i++) {
                int counts = std::min(cfg.planning.num_feasi_check, int(score.at(i).size()));
                result_score.at(i) = std::accumulate(score.at(i).begin(), score.at(i).begin() + counts, double(0));
                result_score.at(i) = prr.at(i).poses.size() == 0 ? -std::numeric_limits<double>::infinity() : result_score.at(i);
            }
        } catch (...) {
            ROS_FATAL_STREAM("pickTraj");
        }

        auto iter = std::max_element(result_score.begin(), result_score.end());
        int idx = std::distance(result_score.begin(), iter);

        if (result_score.at(idx) == -std::numeric_limits<double>::infinity()) {
            ROS_WARN_STREAM("No executable trajectory, values: ");
            for (auto val : result_score) {
                ROS_INFO_STREAM("Score: " << val);
            }
            ROS_INFO_STREAM("------------------");
        }

        return prr.at(idx);
    }

    geometry_msgs::PoseArray Planner::compareToOldTraj(geometry_msgs::PoseArray incoming) {
        auto curr_traj = getCurrentTraj();

        try {
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajSyn->transformBackTrajectory(incoming, odom2rbt);
            incom_rbt.header.frame_id = cfg.robot_frame_id;
            auto incom_score = trajArbiter->scoreTrajectory(incom_rbt);
            // int counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));

            int counts = std::min(cfg.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));

            if (curr_traj.poses.size() == 0) {
                if (incom_subscore == -std::numeric_limits<double>::infinity()) {
                    auto empty_traj = geometry_msgs::PoseArray();
                    setCurrentTraj(empty_traj);
                    return empty_traj;
                } else {
                    setCurrentTraj(incoming);
                    trajectory_pub.publish(incoming);
                    ROS_WARN_STREAM("Old Traj length 0");
                    return incoming;
                }
            } 

            auto curr_rbt = gapTrajSyn->transformBackTrajectory(curr_traj, odom2rbt);
            curr_rbt.header.frame_id = cfg.robot_frame_id;
            int start_position = egoTrajPosition(curr_rbt);
            geometry_msgs::PoseArray reduced_curr_rbt = curr_rbt;
            reduced_curr_rbt.poses = std::vector<geometry_msgs::Pose>(curr_rbt.poses.begin() + start_position, curr_rbt.poses.end());
            if (reduced_curr_rbt.poses.size() < 2) {
                ROS_WARN_STREAM("Old Traj short");
                setCurrentTraj(incoming);
                return incoming;
            }
            auto curr_score = trajArbiter->scoreTrajectory(reduced_curr_rbt);
            counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));
            auto curr_subscore = std::accumulate(curr_score.begin(), curr_score.begin() + counts, double(0));
            incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));

            std::vector<std::vector<double>> ret_traj_scores(2);
            ret_traj_scores.at(0) = incom_score;
            ret_traj_scores.at(1) = curr_score;
            std::vector<geometry_msgs::PoseArray> viz_traj(2);
            viz_traj.at(0) = incom_rbt;
            viz_traj.at(1) = reduced_curr_rbt;
            trajvisualizer->pubAllScore(viz_traj, ret_traj_scores);

            ROS_DEBUG_STREAM("Curr Score: " << curr_subscore << ", incom Score:" << incom_subscore);

            if (curr_subscore == -std::numeric_limits<double>::infinity() && incom_subscore == -std::numeric_limits<double>::infinity()) {
                ROS_WARN_STREAM("Both Failed");
                auto empty_traj = geometry_msgs::PoseArray();
                setCurrentTraj(empty_traj);
                return empty_traj;
            }

            if (incom_subscore > curr_subscore + counts) {
                ROS_WARN_STREAM("Swap to new for better score: " << incom_subscore << " > " << curr_subscore << " + " << counts);
                setCurrentTraj(incoming);
                trajectory_pub.publish(incoming);
                return incoming;
            }

            trajectory_pub.publish(curr_traj);
        } catch (...) {
            ROS_FATAL_STREAM("compareToOldTraj");
        }
        return curr_traj;
    }

    int Planner::egoTrajPosition(geometry_msgs::PoseArray curr) {
        std::vector<double> pose_diff(curr.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (size_t i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr.poses.at(i).position.x, 2) + 
                                pow(curr.poses.at(i).position.y, 2));
        }

        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        int closest_pose = std::distance(pose_diff.begin(), min_element_iter) + 1;
        return std::min(closest_pose, int(curr.poses.size() - 1));
    }

    void Planner::setCurrentTraj(geometry_msgs::PoseArray curr_traj) {
        curr_executing_traj = curr_traj;
        return;
    }

    geometry_msgs::PoseArray Planner::getCurrentTraj() {
        return curr_executing_traj;
    }

    void Planner::reset()
    {
        observed_gaps.clear();
        setCurrentTraj(geometry_msgs::PoseArray());
        ROS_INFO_STREAM("log_vel_comp size: " << log_vel_comp.size());
        log_vel_comp.clear();
        ROS_INFO_STREAM("log_vel_comp size after clear: " << log_vel_comp.size() << ", is full: " << log_vel_comp.capacity());
        return;
    }


    bool Planner::isReplan() {
        return replan;
    }

    void Planner::setReplan() {
        replan = false;
    }

    geometry_msgs::Twist Planner::ctrlGeneration(geometry_msgs::PoseArray traj) {
        if (traj.poses.size() < 2){
            ROS_WARN_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 3");
            return geometry_msgs::Twist();
        }

        // Know Current Pose
        geometry_msgs::PoseStamped curr_pose_local;
        curr_pose_local.header.frame_id = cfg.robot_frame_id;
        curr_pose_local.pose.orientation.w = 1;
        geometry_msgs::PoseStamped curr_pose_odom;
        curr_pose_odom.header.frame_id = cfg.odom_frame_id;
        tf2::doTransform(curr_pose_local, curr_pose_odom, rbt2odom);
        geometry_msgs::Pose curr_pose = curr_pose_odom.pose;

        auto orig_ref = trajController->trajGen(traj);
        ctrl_idx = trajController->targetPoseIdx(curr_pose, orig_ref);
        nav_msgs::Odometry ctrl_target_pose;
        ctrl_target_pose.header = orig_ref.header;
        ctrl_target_pose.pose.pose = orig_ref.poses.at(ctrl_idx);
        ctrl_target_pose.twist.twist = orig_ref.twist.at(ctrl_idx);

        sensor_msgs::LaserScan stored_scan_msgs;
        if (cfg.planning.projection_inflated) {
            stored_scan_msgs = *sharedPtr_inflatedlaser.get();
        } else {
            stored_scan_msgs = *sharedPtr_laser.get();
        }

        geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
        auto cmd_vel = trajController->controlLaw(curr_pose, ctrl_target_pose, stored_scan_msgs, rbt_in_cam_lc);

        return cmd_vel;
    }

    void Planner::rcfgCallback(potential_gap::pgConfig &config, uint32_t level)
    {
        cfg.reconfigure(config);
        
        // set_capacity destroys everything if different from original size, 
        // resize only if the new size is greater
        log_vel_comp.clear();
        log_vel_comp.set_capacity(cfg.planning.halt_size);
    }

    geometry_msgs::PoseArray Planner::getPlanTrajectory() {
        updateTF();

        auto gap_set = gapManipulate();
        
        std::vector<geometry_msgs::PoseArray> traj_set;
        
        auto score_set = initialTrajGen(gap_set, traj_set);
        
        auto picked_traj = pickTraj(traj_set, score_set);

        auto final_traj = compareToOldTraj(picked_traj);
        
        return final_traj;
    }

    bool Planner::recordAndCheckVel(geometry_msgs::Twist cmd_vel) {
        double val = std::abs(cmd_vel.linear.x) + std::abs(cmd_vel.linear.y) + std::abs(cmd_vel.angular.z);
        log_vel_comp.push_back(val);
        double cum_vel_sum = std::accumulate(log_vel_comp.begin(), log_vel_comp.end(), double(0));
        bool ret_val = cum_vel_sum > 1.0 || !log_vel_comp.full();
        if (!ret_val && !cfg.man.man_ctrl) {
            ROS_FATAL_STREAM("--------------------------Planning Failed--------------------------");
            reset();
        }
        return ret_val || cfg.man.man_ctrl;
    }

}