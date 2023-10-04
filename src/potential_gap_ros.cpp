#include <ros/ros.h>
#include <potential_gap/potential_gap_ros.h>
#include <potential_gap/gap.h>
#include <pluginlib/class_list_macros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/numeric/odeint.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>

//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(potential_gap::PotentialGapPlannerROS, trajectory_based_local_planner::TrajectoryBaseLocalPlanner<potential_gap::pg_trajectory_ptr>);

namespace potential_gap
{
    PotentialGapPlannerROS::PotentialGapPlannerROS() : 
        initialized_(false),
        new_goal_set_(false)
    {
    }
    
    PotentialGapPlannerROS::~PotentialGapPlannerROS()
    {
    }
    
    void PotentialGapPlannerROS::initialize(
        std::string name, 
        boost::shared_ptr<turtlebot_trajectory_testing::NIConfigUtility>& ni_util, 
        GenAndTest_ptr& traj_tester) 
    {
        name_ = name;
        ROS_ERROR_STREAM_NAMED(name_, "Not valid for this local planner plugin.");
    }
    
    void PotentialGapPlannerROS::initialize(
        std::string name) 
    {
        name_ = name;
        
        if (! isInitialized()) 
        {
            ros::NodeHandle nh(name_);
            ros::NodeHandle pnh("~/" + name_);
            nh_ = nh;
            pnh_ = pnh;
            
            first_loop_ = true;
            initialized_ = true;

            laser_sub_ = pnh.subscribe("/point_scan", 100, &Planner::laserScanCB, &planner_);
            feasi_laser_sub_ = pnh.subscribe("/inflated_point_scan", 100, &Planner::inflatedlaserScanCB, &planner_);
            pose_sub_ = pnh.subscribe("/odom",10, &Planner::poseCB, &planner_);
            planner_.initialize(pnh);
            initialized_ = true;

            pf_local_frame_enable_ = true;
            pnh_.getParam("pf_local_frame_enable", pf_local_frame_enable_);
            pnh_.setParam("pf_local_frame_enable", pf_local_frame_enable_);
            planner_.pf_local_frame_enable_ = pf_local_frame_enable_;

            // Setup dynamic reconfigure
            dynamic_recfg_server = boost::make_shared<dynamic_reconfigure::Server <potential_gap::pgConfig> > (pnh);
            f = boost::bind(&potential_gap::Planner::rcfgCallback, &planner_, _1, _2);
            dynamic_recfg_server->setCallback(f);
        }
        else{
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }

        planner_.updateLocalTF();
        planner_.updateGlobalTF();
    }
    
    bool PotentialGapPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& local_global_plan)
    {
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        planner_.updateLocalTF();
        planner_.updateGlobalTF();
        
        new_goal_set_ = true;
        
        bool goal_succ = planner_.setGoal(local_global_plan);
        
        if(goal_succ)
        {
            first_loop_ = false;
        }
        
        return goal_succ;
    }
    
    void PotentialGapPlannerROS::reset()
    {
        planner_.reset();
        first_loop_ = true;
        return;
    }
    

    
    pg_trajectory_ptr PotentialGapPlannerROS::generateLocalTrajectory(nav_msgs::Odometry::ConstPtr odom)
    {
//         DURATION_INFO_STREAM("total", 1);
        if (!planner_.initialized())
        {
            planner_.initialize(pnh_);
            ROS_WARN_STREAM("generateLocalTrajectory called before initializing planner");
        }
        
        if(first_loop_)
        {
            ROS_ERROR_STREAM("First loop, no goal initialized.");
            return nullptr;
        }
        
        planner_.updateLocalTF();
        planner_.updateGlobalTF();
        
//         if(planner_.getObservedGapsSize() == 0)
//             return nullptr;

        auto gap_set = planner_.gapManipulate();
        
        std::vector<geometry_msgs::PoseArray> traj_set, virtual_traj_set;
        
        auto score_set = planner_.initialTrajGen(gap_set, traj_set, virtual_traj_set);
        
        geometry_msgs::PoseArray chosen_virtual_traj_set;
        auto picked_traj = planner_.pickTraj(traj_set, score_set, gap_set, virtual_traj_set, chosen_virtual_traj_set);
        planner_.setCurrentTraj(picked_traj);
        planner_.autoSetChosenGap();
        planner_.pubPickedTraj(picked_traj);
        
        pg_trajectory_ptr pg_traj_ptr = std::make_shared<PotentialGapTrajectory>();
        if(!pf_local_frame_enable_)
        {
            pg_traj_ptr->setFrame(planner_.getOdomFrameId());
        }
        else
        {
            pg_traj_ptr->setFrame(planner_.getRobotFrameId());
        }
        // pg_traj_ptr->setFrame(planner_.getOdomFrameId());
        
        pg_traj_ptr->setTrajectory(picked_traj);
        
        return pg_traj_ptr;
    }
    
//     bool PotentialGapPlannerROS::trajectoryCheck()
//     {
//         if(new_goal_set_)
//         {
//             planner_.updateTF();
            
// //             bool inc = potential_gap_planner_->incompletionCheck();
// //             bool inc = true;
// //             
// //             bool feasi = potential_gap_planner_->feasibilityCheck();
// //             bool feasi = true;
// //             
// //             if(! (inc && feasi))
// //                 new_goal_set_ = false;
// //             
// //             return inc && feasi;

//             // WARNING: Not used for now
//             bool end_reached = planner_.reachedTrajEnd();
            
//             if(end_reached)
//                 new_goal_set_ = false;
            
//             return !end_reached;
//         }
//         else
//         {
//             return true;
//         }
//     }

    bool PotentialGapPlannerROS::trajectoryCheck()
    {
        if(!pf_local_frame_enable_)
        {
            planner_.updateLocalTF();
            planner_.updateGlobalTF();
        }
        else
        {
            planner_.updateLocalTF();
        }

        return true;
    }
    
}

