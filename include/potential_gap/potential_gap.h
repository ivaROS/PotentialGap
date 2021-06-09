#ifndef POTENTIAL_GAP_H
#define POTENTIAL_GAP_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include <navfn/navfn_ros.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/gap.h>
#include <potential_gap/helper.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <benchmarking_tools/benchmarking_tools.h>
// #include <tf2/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/numeric/odeint.hpp>

#include <potential_gap/planner.h>

#include <dynamic_reconfigure/server.h>
#include <potential_gap/pgConfig.h>

namespace potential_gap {

    class PotentialGapPlanner : public nav_core::BaseLocalPlanner 
    {
        public: 

            PotentialGapPlanner();

            ~PotentialGapPlanner();

            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            bool isGoalReached();

            bool setPlan(const std::vector<geometry_msgs::PoseStamped> & plan);

            void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

            void reset();

        private:
            // potential_gap::pgConfig loadRosParamFromNodeHandle(const ros::NodeHandle& nh);

            potential_gap::Planner planner;
            std::string planner_name;
            ros::NodeHandle nh, pnh;

            ros::Subscriber laser_sub, inflated_laser_sub;
            ros::Subscriber pose_sub;
            ros::Subscriber feasi_laser_sub;

            bool initialized = false;

            boost::shared_ptr<dynamic_reconfigure::Server<potential_gap::pgConfig> > dynamic_recfg_server;
            dynamic_reconfigure::Server<potential_gap::pgConfig>::CallbackType f;
    };
}

#endif 
