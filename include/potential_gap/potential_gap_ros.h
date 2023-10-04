#ifndef POTENTIAL_GAP_PLANNER_ROS_H
#define POTENTIAL_GAP_PLANNER_ROS_H

#include <ros/ros.h>
// #include <nav_core/base_local_planner.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <navfn/navfn_ros.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/gap.h>
#include <potential_gap/helper.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/numeric/odeint.hpp>

// #include <turtlebot_trajectory_functions/path.h>

#include <potential_gap/planner.h>
#include <potential_gap/potential_gap_trajectory_type.h>
#include <dynamic_reconfigure/server.h>
#include <potential_gap/pgConfig.h>

// #include <potential_gap/potential_gap_planner.h>
#include <trajectory_based_local_planner/trajectory_base_local_planner.h>

namespace potential_gap
{
    class PotentialGapPlannerROS : public trajectory_based_local_planner::TrajectoryBaseLocalPlanner<pg_trajectory_ptr>
    {
    public:
        
        PotentialGapPlannerROS();
        
        void initialize(std::string name, boost::shared_ptr<turtlebot_trajectory_testing::NIConfigUtility>& ni_util, GenAndTest_ptr& traj_tester);
        
        void initialize(std::string name);
        
        ~PotentialGapPlannerROS();
        
        pg_trajectory_ptr generateLocalTrajectory(nav_msgs::Odometry::ConstPtr odom);
        
        bool trajectoryCheck();

        /**
        * @brief  Set the plan that the controller is following
        * @param local_global_plan The plan to pass to the controller
        * @return True if the plan was updated successfully, false otherwise
        */
        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& local_global_plan);

        void reset();

        bool isInitialized() {
            return initialized_;
        }
        
    private:
        potential_gap::Planner planner_;
        std::string name_;
        ros::NodeHandle nh_, pnh_;
        
        ros::Subscriber laser_sub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber feasi_laser_sub_;

        bool initialized_;
        bool new_goal_set_;
        bool first_loop_;

        bool pf_local_frame_enable_;
        
        boost::shared_ptr<dynamic_reconfigure::Server<potential_gap::pgConfig> > dynamic_recfg_server;
        dynamic_reconfigure::Server<potential_gap::pgConfig>::CallbackType f;
    };
}

#endif // POTENTIAL_GAP_PLANNER_ROS_H

