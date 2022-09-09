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

        // pnh = unh;
        pnh = ros::NodeHandle(unh.getNamespace() + "/cc");

        // Config Setup
        cfg.loadRosParamFromNodeHandle(unh);

        // Load precomputed robot geo
        std::string file_name = "/home/shiyu/workspaces/cheetah_ws/src/quadruped_nav_benchmark/config/robot_geometry/box_1_geometry.yaml";
        unh.getParam("file_name", file_name);
        unh.setParam("file_name", file_name);

        int shape_id = 1;
        unh.getParam("shape_id", shape_id);
        unh.setParam("shape_id", shape_id);

        double length = 0.7, width = 0.3, decay_factor = 0, avg_lin_speed = 0.2, avg_rot_speed = 0.5;
        unh.getParam("length", length);
        unh.getParam("width", width);
        unh.getParam("decay_factor", decay_factor);
        unh.getParam("avg_lin_speed", avg_lin_speed);
        unh.getParam("avg_rot_speed", avg_rot_speed);
        unh.setParam("length", length);
        unh.setParam("width", width);
        unh.setParam("decay_factor", decay_factor);
        unh.setParam("avg_lin_speed", avg_lin_speed);
        unh.setParam("avg_rot_speed", avg_rot_speed);

        RobotShape robot_shape = static_cast<RobotShape>(shape_id);
        if(robot_shape == RobotShape::circle)
            width = 0;

        Robot robot(robot_shape, length, width, avg_lin_speed, avg_rot_speed);

        use_geo_storage_ = false;
        unh.getParam("use_geo_storage", use_geo_storage_);
        unh.setParam("use_geo_storage", use_geo_storage_);
        
        if(use_geo_storage_)
            robot_geo_storage_ = RobotGeoStorage(file_name);
        else
            robot_geo_proc_ = RobotGeoProc(robot, decay_factor);

        robot_path_orient_linear_decay_ = true;
        virtual_path_decay_enable_ = true;
        unh.getParam("robot_path_orient_linear_decay", robot_path_orient_linear_decay_);
        unh.getParam("virtual_path_decay_enable", virtual_path_decay_enable_);
        unh.setParam("robot_path_orient_linear_decay", robot_path_orient_linear_decay_);
        unh.setParam("virtual_path_decay_enable", virtual_path_decay_enable_);
        speed_factor_ = 2;
        unh.getParam("speed_factor", speed_factor_);
        unh.setParam("speed_factor", speed_factor_);

        // Bezier curve
        use_bezier_ = true;
        unh.getParam("use_bezier", use_bezier_);
        unh.setParam("use_bezier", use_bezier_);
        
        // Debug robot geometry storage and process
        // robot_geo_storage_ = RobotGeoStorage(file_name);
        // robot_geo_proc_ = RobotGeoProc(robot);

        // Eigen::Vector2d orientation_vec(1, 0);
        // Eigen::Vector2d pt_vec(1,0);
        // Eigen::Vector2d motion_vec = pt_vec;
        // double vec_dot_pro = orientation_vec.dot(pt_vec);
        
        // ros::WallTime interp_start = ros::WallTime::now();
        // double interp_er = robot_geo_storage_.getInterpEquivR(vec_dot_pro);
        // double interp_epl = robot_geo_storage_.getInterpEquivPL(vec_dot_pro);
        // ros::WallDuration interp_time = ros::WallTime::now() - interp_start;
        
        // ros::WallTime comp_start = ros::WallTime::now();
        // double er = robot_geo_proc_.getEquivalentR(orientation_vec, pt_vec);
        // double epl = robot_geo_proc_.getEquivalentPL(orientation_vec, motion_vec);
        // ros::WallDuration comp_time = ros::WallTime::now() - comp_start;

        // ROS_INFO_STREAM("Interp er: " << interp_er << ", Interp epl: " << interp_epl << ", time: " << (double)interp_time.toNSec() << " ns");
        // ROS_INFO_STREAM("Comp er: " << er << ", Comp epl: " << epl << ", time: " << (double)comp_time.toNSec() << " ns");
        // throw;
        
        // Debug robot_geo_processor
        // Eigen::Vector2d orientation_vec(1, 0);
        // Eigen::Vector2d p1(-0.35,0);
        // Eigen::Vector2d p2(-0.35,-0.15);
        // Eigen::Vector2d p3(0,-0.15);
        // Eigen::Vector2d p4(0.35,-0.15);
        // Eigen::Vector2d p5(0.35,0);
        // Eigen::Vector2d p6(0.35,0.15);
        // Eigen::Vector2d p7(0,0.15);
        // Eigen::Vector2d p8(-0.35,0.15);
        // Eigen::Vector2d p9(0.1, 0.1);
        // Eigen::Vector2d p10(1, 1);
        // double vec_length = 3; // 3
        // std::vector<Eigen::Vector2d> pt_list{vec_length*p1/p1.norm(), vec_length*p2/p2.norm(), vec_length*p3/p3.norm(), vec_length*p4/p4.norm(), vec_length*p5/p5.norm(), vec_length*p6/p6.norm(), vec_length*p7/p7.norm(), vec_length*p8/p8.norm(), p9, p10};

        // // True 
        // std::vector<double> r{p1.norm(), p2.norm(), p3.norm(), p4.norm(), p5.norm(), p6.norm(), p7.norm(), p8.norm(), sqrt(0.15*0.15*2), sqrt(0.15*0.15*2)};
        // double alpha = 2 * atan2(0.15, 0.35);
        // std::vector<double> el{0.3, 2*p2.norm()*sin(alpha), 0.7, 2*p2.norm()*sin(alpha), 0.3, 2*p2.norm()*sin(alpha), 0.7, 2*p2.norm()*sin(alpha), 2*p2.norm()*cos(M_PI / 4 - alpha/2), 2*p2.norm()*cos(M_PI / 4 - alpha/2)};
        
        
        // double p1_min_dist = vec_length - 0.35;
        // double p1_max_dist = sqrt(pow(vec_length + 0.35, 2) + 0.15 * 0.15);
        // double dia = 2*p2.norm();
        // double p3_min_dist = vec_length - 0.15;
        // double p3_max_dist = sqrt(pow(vec_length + 0.15, 2) + 0.35 * 0.35);
        // std::vector<double> er{p1_max_dist - p1_min_dist, dia, p3_max_dist - p3_min_dist, dia, p1_max_dist - p1_min_dist, dia, p3_max_dist - p3_min_dist, dia, sqrt(pow(0.45, 2) + pow(0.25, 2)), (p10+p6).norm() - (p10-p6).norm()};

        // std::vector<double> n_dist{vec_length - 0.35, vec_length - p2.norm(), vec_length - 0.15, vec_length - p2.norm(), vec_length - 0.35, vec_length - p2.norm(), vec_length - 0.15, vec_length - p2.norm(), -1, (p10-p6).norm()};

        // double rot_ang = M_PI / 2;
        // Eigen::Matrix2d rot;
        //         rot << cos(rot_ang), -sin(rot_ang), sin(rot_ang), cos(rot_ang);
        // for(size_t i = 0; i < pt_list.size(); i++)
        // {   
        //     Eigen::Vector2d p = pt_list[i];
        //     Eigen::Vector2d p_tmp = p;
        //     // True values
        //     double r_dist = r[i];
        //     double er_p = er[i];
        //     double el_p = el[i];
        //     double nd = n_dist[i];

        //     // Calculated values
        //     Eigen::Vector2d o_vec = rot * orientation_vec;
        //     Eigen::Vector2d p_vec = rot * p;
        //     Eigen::Vector2d p_tmp_vec = rot * p_tmp;
        //     double r_c_dist = robot_geo_proc_.getEquivalentR(o_vec, p_vec);
        //     ros::WallTime start_time = ros::WallTime::now();
        //     double er_c_p = robot_geo_proc_.getEquivalentRL(o_vec, p_tmp_vec);
        //     ros::WallDuration d = ros::WallTime::now() - start_time;
        //     ROS_INFO_STREAM("RL time: " << (float) d.toNSec() / 1000 << "mu sec");
        //     double el_c_p = robot_geo_proc_.getEquivalentPL(o_vec, p_vec);
        //     ros::WallTime start_n_time = ros::WallTime::now();
        //     double n_dist_c = robot_geo_proc_.getNearestDistance(o_vec, p_tmp_vec);
        //     ros::WallDuration dn = ros::WallTime::now() - start_n_time;
        //     ROS_INFO_STREAM("Nearest time: " << (float) dn.toNSec() / 1000 << "mu sec");

        //     ROS_INFO_STREAM("True values: " << r_dist << " " << er_p << " " << el_p << " " << nd << "; Calculated values: " << r_c_dist << " " << er_c_p << " " << el_c_p << " " << n_dist_c);
        //     ROS_INFO_STREAM("Equals: " << (r_dist - r_c_dist) << " " << (er_p - er_c_p) << " " << (el_p - el_c_p) << " " << (nd - n_dist_c));
        // }
        // throw;

        // Visualization Setup
        // Fix this later
        local_traj_pub = nh.advertise<geometry_msgs::PoseArray>("relevant_traj", 500);
        trajectory_pub = nh.advertise<geometry_msgs::PoseArray>("pg_traj", 10);
        gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("gaps", 1);
        selected_gap_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("sel_gaps", 1);
        ni_traj_pub = nh.advertise<geometry_msgs::PoseArray>("ni_traj", 10);
        ni_traj_pub_other = nh.advertise<visualization_msgs::MarkerArray>("other_ni_traj", 5);

        transformed_laser_pub = nh.advertise<sensor_msgs::LaserScan>("transformed_laserscan", 5);
        virtual_orient_traj_pub = nh.advertise<geometry_msgs::PoseArray>("picked_virtual_traj", 10);

        // TF Lookup setup
        tfBuffer = std::make_shared<tf2_ros::Buffer>();
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        _initialized = true;

        finder = new potential_gap::GapUtils(cfg, robot_geo_proc_);
        gapvisualizer = new potential_gap::GapVisualizer(nh, cfg);
        goalselector = new potential_gap::GoalSelector(nh, cfg, robot_geo_proc_);
        trajvisualizer = new potential_gap::TrajectoryVisualizer(nh, cfg);
        trajArbiter = new potential_gap::TrajectoryArbiter(nh, cfg, robot_geo_proc_);
        gapTrajSyn = new potential_gap::GapTrajGenerator(nh, cfg, robot_geo_proc_);
        goalvisualizer = new potential_gap::GoalVisualizer(nh, cfg);
        gapManip = new potential_gap::GapManipulator(nh, cfg, robot_geo_proc_);
        trajController = new potential_gap::TrajectoryController(nh, cfg);

        map2rbt.transform.rotation.w = 1;
        rbt2map.transform.rotation.w = 1;
        odom2rbt.transform.rotation.w = 1;
        rbt2odom.transform.rotation.w = 1;
        rbt_in_rbt.pose.orientation.w = 1;
        rbt_in_rbt.header.frame_id = cfg.robot_frame_id;

        reconfigure_server_ = std::make_shared<ReconfigureServer>(pnh);
        reconfigure_server_->setCallback(boost::bind(&Planner::configCB, this, _1, _2));

        log_vel_comp.set_capacity(cfg.planning.halt_size);
        return true;
    }

    void Planner::configCB(potential_gap::CollisionCheckerConfig &config, uint32_t level)
    {
        ROS_INFO_STREAM("CC Reconfigure Request: "); // TODO: print out the cc type and other parameter values

        Lock lock(connect_mutex_);

        collision_checker_enable_ = config.cc_enable;
        if(!collision_checker_enable_)
        {
            ROS_WARN_STREAM("Collision checking is disabled.");
            return;
        }

        if(config.cc_type != cc_type_)
        {
            if(config.cc_type == potential_gap::CollisionChecker_depth)
            {
                ROS_INFO_STREAM("New cc type = depth");
                cc_wrapper_ = std::make_shared<pips_trajectory_testing::DepthImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
            }
            else if(config.cc_type == potential_gap::CollisionChecker_depth_ego)
            {
                ROS_INFO_STREAM("New cc type = depth ego");
                cc_wrapper_ = std::make_shared<pips_egocylindrical::EgocylindricalRangeImageCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
            }
            else if(config.cc_type == potential_gap::CollisionChecker_egocircle)
            {
                ROS_INFO_STREAM("New cc type = egocircle");
                cc_wrapper_ = std::make_shared<pips_egocircle::EgoCircleCCWrapper>(nh, pnh, tf2_utils::TransformManager(tfBuffer, tfListener));
            }

            traj_tester_ = std::make_shared<TurtlebotGenAndTest>(nh, pnh);
            
            cc_wrapper_->init();
            cc_wrapper_->autoUpdate();

            traj_tester_->init();
            traj_tester_->setCollisionChecker(cc_wrapper_->getCC());
            
            cc_type_ = config.cc_type;
        }
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

    boost::shared_ptr<sensor_msgs::LaserScan const> Planner::transformLaserToRbt(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        sensor_msgs::LaserScan transformed_laser;
        transformed_laser.header = msg->header;
        transformed_laser.header.frame_id = cfg.robot_frame_id;
        transformed_laser.angle_min = msg->angle_min;
        transformed_laser.angle_max = msg->angle_max;
        transformed_laser.angle_increment = msg->angle_increment;
        transformed_laser.time_increment = msg->time_increment;
        transformed_laser.scan_time = msg->scan_time;
        transformed_laser.range_min = msg->range_min;
        transformed_laser.range_max = msg->range_max;
        transformed_laser.intensities = msg->intensities;

        std::vector<float> ranges(msg->ranges.size(), msg->range_max);
        transformed_laser.ranges = ranges;

        for(size_t i = 0; i < msg->ranges.size(); i++)
        {
            double orig_range = msg->ranges[i];
            double orig_ang = i * msg->angle_increment + msg->angle_min;
            orig_ang = orig_ang <= msg->angle_max ? orig_ang : msg->angle_max;

            geometry_msgs::PointStamped orig_pt, transformed_pt;
            orig_pt.header = msg->header;
            orig_pt.point.x = orig_range * cos(orig_ang);
            orig_pt.point.y = orig_range * sin(orig_ang);
            
            geometry_msgs::TransformStamped trans = tfBuffer->lookupTransform(cfg.robot_frame_id, cfg.sensor_frame_id, ros::Time(0));
            tf2::doTransform(orig_pt, transformed_pt, trans);
            // ROS_INFO_STREAM(cfg.sensor_frame_id << " " << orig_pt.header.frame_id << " " << transformed_pt.header.frame_id);

            double transformed_range = sqrt(pow(transformed_pt.point.x, 2) + pow(transformed_pt.point.y, 2));
            double transformed_ang = std::atan2(transformed_pt.point.y, transformed_pt.point.x);
            int idx = (int) round((transformed_ang - msg->angle_min) / msg->angle_increment);
            idx = idx < msg->ranges.size() ? idx : (msg->ranges.size() - 1);
            idx = idx >= 0 ? idx : 0;

            if(transformed_range < transformed_laser.ranges[idx])
                transformed_laser.ranges[idx] = transformed_range;
        }

        return boost::make_shared<sensor_msgs::LaserScan const>(transformed_laser);
    }

    void Planner::inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        // sharedPtr_inflatedlaser = msg;
        sharedPtr_inflatedlaser = transformLaserToRbt(msg);
        // TODO: didn't transform to robot frame, may have problem if using inflated egocircle.
    }

    void Planner::laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        // sharedPtr_laser = msg;
        sharedPtr_laser = transformLaserToRbt(msg);
        transformed_laser_pub.publish(sharedPtr_laser);

        boost::shared_ptr<sensor_msgs::LaserScan const> tmp_msg = sharedPtr_laser;

        if (cfg.planning.planning_inflated && sharedPtr_inflatedlaser) {
            // msg = sharedPtr_inflatedlaser;
            // TODO: not sure if the transformed inflated egocircle is right
            tmp_msg = sharedPtr_inflatedlaser;
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

        // boost::shared_ptr<sensor_msgs::LaserScan const> tmp;
        // if (sharedPtr_inflatedlaser) {
        //     tmp = sharedPtr_inflatedlaser;
        // } else {
        //     tmp = msg;
        // }

        // If no global plan, the local goal finding won't execute.
        goalselector->updateEgoCircle(tmp_msg);
        trajArbiter->updateEgoCircle(tmp_msg);

        geometry_msgs::PoseStamped local_goal;
        if(goal_set)
        {
            goalselector->updateLocalGoal(map2rbt);
            local_goal = goalselector->getCurrentLocalGoal(rbt2odom);
            goalvisualizer->localGoal(local_goal);
        
            trajArbiter->updateLocalGoal(local_goal, odom2rbt);
        }

        gapManip->updateEgoCircle(tmp_msg);
        trajController->updateEgoCircle(tmp_msg);

    }

    void Planner::poseCB(const nav_msgs::Odometry::ConstPtr& msg)
    {
        // Transform the msg to odom frame
        if(msg->header.frame_id != cfg.odom_frame_id)
        {
            geometry_msgs::TransformStamped robot_pose_odom_trans = tfBuffer->lookupTransform(cfg.odom_frame_id, msg->header.frame_id, ros::Time(0));

            geometry_msgs::PoseStamped in_pose, out_pose;
            in_pose.header = msg->header;
            in_pose.pose = msg->pose.pose;

            tf2::doTransform(in_pose, out_pose, robot_pose_odom_trans);
            sharedPtr_pose = out_pose.pose;
            sharedPtr_odom = *msg;
            sharedPtr_odom.pose.pose = out_pose.pose;
            sharedPtr_odom.header.frame_id = cfg.odom_frame_id;
        }
        else
        {
            sharedPtr_pose = msg->pose.pose;
            sharedPtr_odom = *msg;
        }
    }

    bool Planner::setGoal(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        updateTF();

        if (plan.size() == 0) return true;

        // plan should be in global frame
        std::vector<geometry_msgs::PoseStamped> global_plan;
        if(plan[0].header.frame_id != cfg.map_frame_id)
        {
            geometry_msgs::TransformStamped other_to_global_trans = tfBuffer->lookupTransform(cfg.map_frame_id, plan[0].header.frame_id, ros::Time(0));
            for(size_t i = 0; i < plan.size(); i++)
            {
                geometry_msgs::PoseStamped plan_pose = plan[i];
                geometry_msgs::PoseStamped out_pose;
                tf2::doTransform(plan_pose, out_pose, other_to_global_trans);
                out_pose.header.stamp = plan_pose.header.stamp;
                out_pose.header.frame_id = cfg.map_frame_id;
                global_plan.push_back(out_pose);
            }
        }
        else
        {
            global_plan = plan;
        }
        
        final_goal_odom = *std::prev(global_plan.end());
        tf2::doTransform(final_goal_odom, final_goal_odom, map2odom);

        // Store New Global Plan to Goal Selector
        goalselector->setGoal(global_plan);
        
        trajvisualizer->rawGlobalPlan(goalselector->getRawGlobalPlan());

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

        goal_set = true;
        return true;
    }

    void Planner::updateTF()
    {
        try {
            map2rbt  = tfBuffer->lookupTransform(cfg.robot_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2map  = tfBuffer->lookupTransform(cfg.map_frame_id, cfg.robot_frame_id, ros::Time(0));
            odom2rbt = tfBuffer->lookupTransform(cfg.robot_frame_id, cfg.odom_frame_id, ros::Time(0));
            rbt2odom = tfBuffer->lookupTransform(cfg.odom_frame_id, cfg.robot_frame_id, ros::Time(0));
            cam2odom = tfBuffer->lookupTransform(cfg.odom_frame_id, cfg.sensor_frame_id, ros::Time(0));
            odom2cam = tfBuffer->lookupTransform(cfg.sensor_frame_id, cfg.odom_frame_id, ros::Time(0));
            map2odom = tfBuffer->lookupTransform(cfg.odom_frame_id, cfg.map_frame_id, ros::Time(0));
            rbt2cam = tfBuffer->lookupTransform(cfg.sensor_frame_id, cfg.robot_frame_id, ros::Time(0));
            cam2rbt = tfBuffer->lookupTransform(cfg.robot_frame_id, cfg.sensor_frame_id, ros::Time(0));

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

        // geometry_msgs::PoseStamped local_goal_sensor_frame;
        // tf2::doTransform(goalselector->rbtFrameLocalGoal(), local_goal_sensor_frame, rbt2cam);
        geometry_msgs::PoseStamped local_goal_rbt_frame = goalselector->rbtFrameLocalGoal();
        try {
            for (size_t i = 0; i < manip_set.size(); i++)
            {
                gapManip->reduceGap(manip_set.at(i), local_goal_rbt_frame);
                gapManip->convertAxialGap(manip_set.at(i));
                gapManip->radialExtendGap(manip_set.at(i));
                gapManip->setGapWaypoint(manip_set.at(i), local_goal_rbt_frame);
            }
        } catch(...) {
            ROS_FATAL_STREAM("gapManipulate");
        }

        goalvisualizer->drawGapGoals(manip_set);
        gapvisualizer->drawManipGaps(manip_set);
        return manip_set;
    }

    // std::vector<geometry_msgs::PoseArray> 
    std::vector<std::vector<double>> Planner::initialTrajGen(std::vector<potential_gap::Gap> vec, std::vector<geometry_msgs::PoseArray>& res, std::vector<geometry_msgs::PoseArray>& virtual_decayed) {
        boost::mutex::scoped_lock gapset(gapset_mutex);
        std::vector<geometry_msgs::PoseArray> ret_traj(vec.size());
        std::vector<geometry_msgs::PoseArray> virtual_traj(vec.size());
        std::vector<std::vector<double>> ret_traj_scores(vec.size());
        // geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam; // lc as local copy
        geometry_msgs::PoseStamped rbt_local_pose;
        rbt_local_pose.header.frame_id = cfg.robot_frame_id;
        rbt_local_pose.header.stamp = rbt2odom.header.stamp;
        rbt_local_pose.pose.orientation.w = 1;
        ROS_INFO_STREAM("Gap number: " << vec.size());
        try {
            for (size_t i = 0; i < vec.size(); i++) {
                // Generate trajectory in robot frame.
                geometry_msgs::PoseArray tmp;
                if(use_bezier_)
                {
                    tmp = gapTrajSyn->generateBezierTrajectory(vec.at(i), sharedPtr_odom, odom2rbt);
                }
                else
                {
                    tmp = gapTrajSyn->generateTrajectory(vec.at(i), rbt_local_pose);
                }
                
                tmp = gapTrajSyn->forwardPassTrajectory(tmp);
                // auto tmp_rbt = gapTrajSyn->transformBackTrajectory(tmp, cam2rbt);
                auto virtual_score_path = getOrientDecayedPath(tmp);
                virtual_traj.at(i) = virtual_score_path;
                ret_traj_scores.at(i) = trajArbiter->scoreTrajectory(virtual_score_path);
                ret_traj.at(i) = gapTrajSyn->transformBackTrajectory(tmp, rbt2odom);
            }
        } catch (...) {
            ROS_FATAL_STREAM("initialTrajGen");
        }
        
        trajvisualizer->pubAllScore(ret_traj, ret_traj_scores);
        trajvisualizer->pubAllTraj(ret_traj);
        res = ret_traj;
        virtual_decayed = virtual_traj;
        return ret_traj_scores;
    }

    geometry_msgs::PoseArray Planner::getOrientDecayedPath(geometry_msgs::PoseArray orig_path)
    {
        // The original path should be in robot frame
        assert(orig_path.header.frame_id == cfg.robot_frame_id);
        geometry_msgs::PoseArray decayed_path;

        if(orig_path.poses.size() <= 1)
        {
            ROS_WARN_STREAM("[getOrientDecayedPath] Original path is too short with size [ " << orig_path.poses.size() << " ].");
            decayed_path = orig_path;
            return decayed_path;
        }
        
        if(robot_geo_proc_.robot_.shape == RobotShape::circle || !virtual_path_decay_enable_)
        {
            decayed_path = orig_path;
        }
        else if(robot_geo_proc_.robot_.shape == RobotShape::box)
        {
            decayed_path.header = orig_path.header;
            geometry_msgs::Pose first_pose = orig_path.poses[0];
            geometry_msgs::Quaternion init_quat;
            init_quat.w = 1;
            first_pose.orientation = init_quat;
            decayed_path.poses.push_back(first_pose);
            double length = 0;
            for(size_t i = 1; i < orig_path.poses.size(); i++)
            {
                if(!robot_path_orient_linear_decay_)
                {
                    geometry_msgs::Pose curr_pose = orig_path.poses[i];
                    curr_pose.orientation = init_quat;
                    decayed_path.poses.push_back(curr_pose);
                }
                else
                {
                    geometry_msgs::Pose curr_pose = orig_path.poses[i];
                    geometry_msgs::Pose prev_pose = orig_path.poses[i-1];
                    double x_diff = curr_pose.position.x - prev_pose.position.x;
                    double y_diff = curr_pose.position.y - prev_pose.position.y;
                    double dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
                    length += dist;
                    // double avg_speed = sqrt(pow(cfg.control.vx_absmax, 2) + pow(cfg.control.vy_absmax, 2)) / speed_factor_;
                    double avg_speed = 0.2;
                    double t = length / avg_speed;
                    double avg_ang = cfg.control.ang_absmax / speed_factor_;

                    Eigen::Quaterniond q(curr_pose.orientation.w, curr_pose.orientation.x, curr_pose.orientation.y, curr_pose.orientation.z);
                    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
                    double ang_diff = std::abs(euler[2]);
                    double decayed_ang = avg_ang * t;
                    decayed_ang = decayed_ang <= ang_diff ? decayed_ang : ang_diff;
                    if(euler[2] <= 0)
                        decayed_ang = -decayed_ang;
                    
                    double roll = 0, pitch = 0;    
                    Eigen::Quaterniond e;
                    e = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(decayed_ang, Eigen::Vector3d::UnitZ());
                    
                    curr_pose.orientation.w = e.w();
                    curr_pose.orientation.x = e.x();
                    curr_pose.orientation.y = e.y();
                    curr_pose.orientation.z = e.z();

                    decayed_path.poses.push_back(curr_pose);
                }
                
            }
        }
        else
        {
            ROS_WARN("Doesn't support robot shape, use original path.");
            decayed_path = orig_path;
        }

        return decayed_path;
    }

    geometry_msgs::PoseArray Planner::pickTraj(std::vector<geometry_msgs::PoseArray> prr, std::vector<std::vector<double>> score, std::vector<geometry_msgs::PoseArray> virtual_path, geometry_msgs::PoseArray& chosen_virtual_path) {
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
                ROS_DEBUG_STREAM("Score: " << result_score.at(i));
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
        chosen_virtual_path = virtual_path.at(idx);
        ROS_INFO_STREAM("Picked [" << idx << "] traj" );
        return prr.at(idx);
    }

    geometry_msgs::PoseArray Planner::compareToOldTraj(geometry_msgs::PoseArray incoming, geometry_msgs::PoseArray& virtual_curr_traj) {
        auto curr_traj = getCurrentTraj();

        try {
            // Both Args are in Odom frame
            auto incom_rbt = gapTrajSyn->transformBackTrajectory(incoming, odom2rbt);
            incom_rbt.header.frame_id = cfg.robot_frame_id;
            auto virtual_score_path = getOrientDecayedPath(incom_rbt);
            auto incom_score = trajArbiter->scoreTrajectory(virtual_score_path);
            // int counts = std::min(cfg.planning.num_feasi_check, (int) std::min(incom_score.size(), curr_score.size()));
            
            int counts = std::min(cfg.planning.num_feasi_check, (int) incom_score.size());
            auto incom_subscore = std::accumulate(incom_score.begin(), incom_score.begin() + counts, double(0));

            if (curr_traj.poses.size() == 0) {
                if (incom_subscore == -std::numeric_limits<double>::infinity()) {
                    auto empty_traj = geometry_msgs::PoseArray();
                    setCurrentTraj(empty_traj);
                    virtual_curr_traj = empty_traj;
                    ROS_WARN_STREAM("Old Traj length 0, curr traj score -inf.");
                    return empty_traj;
                } else {
                    setCurrentTraj(incoming);
                    virtual_curr_traj = gapTrajSyn->transformBackTrajectory(virtual_score_path, rbt2odom);
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
                virtual_curr_traj = gapTrajSyn->transformBackTrajectory(virtual_score_path, rbt2odom);
                return incoming;
            }
            auto virtual_curr_score_path = getOrientDecayedPath(reduced_curr_rbt);
            auto curr_score = trajArbiter->scoreTrajectory(virtual_curr_score_path);
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

            ROS_INFO_STREAM("Curr Score: " << curr_subscore << ", incom Score:" << incom_subscore);

            if (curr_subscore == -std::numeric_limits<double>::infinity() && incom_subscore == -std::numeric_limits<double>::infinity()) {
                ROS_WARN_STREAM("Both Failed");
                auto empty_traj = geometry_msgs::PoseArray();
                setCurrentTraj(empty_traj);
                virtual_curr_traj = empty_traj;
                return empty_traj;
            }

            if (incom_subscore > curr_subscore + counts) {
                ROS_WARN_STREAM("Swap to new for better score: " << incom_subscore << " > " << curr_subscore << " + " << counts);
                setCurrentTraj(incoming);
                virtual_curr_traj = gapTrajSyn->transformBackTrajectory(virtual_score_path, rbt2odom);
                trajectory_pub.publish(incoming);
                return incoming;
            }
            auto virtual_score_path_curr = getOrientDecayedPath(curr_rbt);
            virtual_curr_traj = gapTrajSyn->transformBackTrajectory(virtual_score_path_curr, rbt2odom);
            trajectory_pub.publish(curr_traj);
        } catch (...) {
            ROS_FATAL_STREAM("compareToOldTraj");
        }
        return curr_traj;
    }

    CollisionResults Planner::checkCollision(const geometry_msgs::PoseArray path)
    {
        // Convert the trajectory from odom to base frame
        auto path_rbt = gapTrajSyn->transformBackTrajectory(path, odom2rbt);
        geometry_msgs::Pose curr_pose;
        curr_pose.orientation.w = 1;
        auto orig_ref = trajController->trajGen(path_rbt);
        orig_ref.header.frame_id = cfg.robot_frame_id;
        ctrl_idx = trajController->targetPoseIdx(curr_pose, orig_ref);

        pips_trajectory_msgs::trajectory_points local_traj;
        local_traj.header.frame_id = cfg.robot_frame_id;
        for(int i = ctrl_idx; i < orig_ref.poses.size(); i++)
        {
            pips_trajectory_msgs::trajectory_point pt;
            pt.x = orig_ref.poses[i].position.x;
            pt.y = orig_ref.poses[i].position.y;

            // ROS_INFO_STREAM(pt.x << " " << pt.y);

            tf2::Quaternion quat_tf;
            tf2::convert(orig_ref.poses[i].orientation, quat_tf);
            tf2::Matrix3x3 m(quat_tf);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            pt.theta = yaw;

            local_traj.points.push_back(pt);
        }
        

        int collision_ind = traj_tester_->evaluateTrajectory(local_traj);

        CollisionResults cc_results(collision_ind, local_traj);

        return cc_results;
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
        if (traj.poses.size() < 1){
            ROS_WARN_STREAM("Available Execution Traj length: " << traj.poses.size() << " < 1");
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

        // geometry_msgs::PoseStamped rbt_in_cam_lc = rbt_in_cam;
        auto cmd_vel = trajController->controlLaw(curr_pose, ctrl_target_pose, stored_scan_msgs, curr_pose_local);

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
        
        std::vector<geometry_msgs::PoseArray> traj_set, virtual_traj_set;
        
        auto score_set = initialTrajGen(gap_set, traj_set, virtual_traj_set);

        geometry_msgs::PoseArray chosen_virtual_traj_set;
        auto picked_traj = pickTraj(traj_set, score_set, virtual_traj_set, chosen_virtual_traj_set);
        virtual_orient_traj_pub.publish(chosen_virtual_traj_set);

        geometry_msgs::PoseArray chosen_final_virtual_traj_set;
        auto final_traj = compareToOldTraj(picked_traj, chosen_final_virtual_traj_set);

        CollisionResults cc_results;
        if(collision_checker_enable_)
        {
            ros::WallTime start = ros::WallTime::now();

            // cc_results = checkCollision(final_traj);
            cc_results = checkCollision(chosen_final_virtual_traj_set);

            ROS_INFO_STREAM("Current trajectory collision checked in " <<  (ros::WallTime::now() - start).toSec() * 1e3 << "ms");
        
            int cc_ite_min = 10;
            double cc_itc_ratio = 0.2;

            if(cc_results.collision_idx_ >= 0 && double(cc_results.collision_idx_) / cc_results.local_traj_.points.size() <= cc_itc_ratio)
            {
                ROS_WARN_STREAM("Current trajectory collides! " << cc_results.collision_idx_ << " " << cc_results.local_traj_.points.size());
                setCurrentTraj(geometry_msgs::PoseArray());
            }
        }
        
        return final_traj;
    }

    geometry_msgs::PoseArray Planner::getSinglePath()
    {
        updateTF();

        auto gap_set = gapManipulate();
        
        std::vector<geometry_msgs::PoseArray> traj_set, virtual_traj_set;
        
        auto score_set = initialTrajGen(gap_set, traj_set, virtual_traj_set);

        geometry_msgs::PoseArray chosen_virtual_traj_set;
        auto picked_traj = pickTraj(traj_set, score_set, virtual_traj_set, chosen_virtual_traj_set);
        virtual_orient_traj_pub.publish(chosen_virtual_traj_set);

        setCurrentTraj(picked_traj);

        pubPickedTraj(picked_traj);

        return picked_traj;
    }

    void Planner::pubPickedTraj(geometry_msgs::PoseArray picked_traj)
    {
        trajectory_pub.publish(picked_traj);
    }

    geometry_msgs::PoseArray Planner::getLocalPath(geometry_msgs::PoseArray input_path)
    {
        return gapTrajSyn->transformBackTrajectory(input_path, odom2rbt);
    }

    bool Planner::reachedTrajEnd()
    {
        auto curr_traj = getCurrentTraj();
        if (curr_traj.poses.size() == 0) {
            return true;
        } 

        // Both Args are in Odom frame
        auto curr_rbt = gapTrajSyn->transformBackTrajectory(curr_traj, odom2rbt);
        curr_rbt.header.frame_id = cfg.robot_frame_id;

        int start_position = egoTrajPosition(curr_rbt);
        geometry_msgs::PoseArray reduced_curr_rbt = curr_rbt;
        reduced_curr_rbt.poses = std::vector<geometry_msgs::Pose>(curr_rbt.poses.begin() + start_position, curr_rbt.poses.end());
        if (reduced_curr_rbt.poses.size() < 5) {
            return true;
        }
        
        return false;
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