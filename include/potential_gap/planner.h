#include <ros/ros.h>
#include <potential_gap/gap.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/numeric/odeint.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>


#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Header.h>
#include "nav_msgs/Odometry.h"
#include "potential_gap/TrajPlan.h"
#include <potential_gap/helper.h>
#include <potential_gap/gap.h>
// #include <potential_gap/trajectory_follower.h>
#include <potential_gap/gap_utils.h>

#include <potential_gap/potentialgap_config.h>
#include <potential_gap/visualization.h>
#include <potential_gap/goal_selector.h>
#include <potential_gap/trajectory_scoring.h>
#include <potential_gap/gap_manip.h>
#include <potential_gap/trajectory_controller.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <benchmarking_tools/benchmarking_tools.h>

#include <omp.h>

#include <dynamic_reconfigure/server.h>
#include <potential_gap/pgConfig.h>

#include <boost/thread/mutex.hpp>


#ifndef PLANNER_H
#define PLANNER_H

namespace potential_gap
{
    class Planner
    {
    private:
        geometry_msgs::TransformStamped map2rbt;        // Transform
        geometry_msgs::TransformStamped rbt2map;
        geometry_msgs::TransformStamped odom2rbt;
        geometry_msgs::TransformStamped rbt2odom;
        geometry_msgs::TransformStamped map2odom;
        geometry_msgs::TransformStamped cam2odom;
        geometry_msgs::TransformStamped rbt2cam;
        geometry_msgs::TransformStamped cam2rbt;

        geometry_msgs::PoseStamped goal_rbt_frame;
        geometry_msgs::PoseStamped curr_pose_odom;
        geometry_msgs::PoseStamped rbt_in_rbt;
        geometry_msgs::PoseStamped rbt_in_cam;
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener *tfListener;
        tf2_ros::TransformBroadcaster goal_br;

        ros::NodeHandle nh;
        ros::Publisher local_traj_pub;
        ros::Publisher trajectory_pub;
        ros::Publisher gap_vis_pub;
        ros::Publisher selected_gap_vis_pub;
        ros::Publisher ni_traj_pub;
        ros::Publisher ni_traj_pub_other;

        // Goals and stuff
        // double goal_orientation;
        geometry_msgs::Pose current_pose_;
        geometry_msgs::PoseStamped local_waypoint_odom; // local_waypoint, 
        geometry_msgs::PoseStamped final_goal_odom;

        // Gaps:
        std::vector<potential_gap::Gap> observed_gaps;
        std::vector<potential_gap::Gap> merged_gaps;
        std::vector<potential_gap::Gap> selected_gap_set;
        std::vector<potential_gap::Gap> ftg_gaps;
        std::vector<potential_gap::Gap> safe_gaps_left;
        std::vector<potential_gap::Gap> safe_gaps_right;
        std::vector<potential_gap::Gap> safe_gaps_central;
        std::vector<potential_gap::Gap> safe_gaps;

        potential_gap::GapUtils *finder;
        potential_gap::GapVisualizer *gapvisualizer;
        potential_gap::GoalSelector *goalselector;
        potential_gap::TrajectoryVisualizer *trajvisualizer;
        potential_gap::GoalVisualizer *goalvisualizer;
        potential_gap::TrajectoryArbiter *trajArbiter;
        potential_gap::GapTrajGenerator *gapTrajSyn;
        potential_gap::GapManipulator *gapManip;
        potential_gap::TrajectoryController *trajController;

        // Status
        bool hasGoal = false;
        bool _initialized = false;

        geometry_msgs::PoseArray pose_arr;
        geometry_msgs::PoseArray pose_arr_odom;

        std::vector<turtlebot_trajectory_generator::ni_state> ctrl;
        int ctrl_idx = 0;

        geometry_msgs::Pose sharedPtr_pose;
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_laser;
        boost::shared_ptr<sensor_msgs::LaserScan const> sharedPtr_inflatedlaser;

        ros::WallTime last_time;
        potential_gap::TrajPlan ni_ref, orig_ref;

        // Dynamic Reconfigure
        boost::shared_ptr<dynamic_reconfigure::Server<potential_gap::pgConfig> > dynamic_recfg_server;
        dynamic_reconfigure::Server<potential_gap::pgConfig>::CallbackType f;

        bool replan = true;
        
        potential_gap::PotentialGapConfig cfg;

        boost::mutex gapset_mutex;

        geometry_msgs::PoseArray curr_executing_traj;

        boost::circular_buffer<double> log_vel_comp;

    public:
        Planner();

        ~Planner();

        /**
         * Set ros Buffer and etc.
         * 
         * @param None
         * @return initialization success / failure
         */
        bool initialize(const ros::NodeHandle&);

        /**
         * @brief Return initialization status
         * @param None
         * @return bool initialization status
         */
        bool initialized();

        /**
         * @brief Check if reached goal using euclidean dist
         * @param None, internally stored goal location and robot position
         * @return bool reached
         */
        bool isGoalReached();

        /**
         * @brief call back function to laserscan, externally linked
         * @param msg laser scan msg
         * @return None, laser scan msg stored locally
         */
        void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
        void inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

        /**
         * @brief call back function to pose, pose information obtained here only used when a new goal is used
         * @param msg pose msg
         * @return None
         */
        void poseCB(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief Interface function for receiving global plan
         * @param plan, vector of PoseStamped
         * @return boolean type on whether successfully registered goal
         */
        bool setGoal(const std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * @brief update all tf transform at the beginning of every planning cycle
         * @param None, all tf received via TF
         * @return None, all registered via internal variables in TransformStamped
         */
        void updateTF();

        /**
         * @brief select the gap to pass through based on where the goal is
         * TODO: make this polymorphism so more than one strategy can be adopted
         * @param selected_gap that will be returned by the same variable
         * @return selected_gap via the passed in variable
         */
        void vectorSelectGap(potential_gap::Gap & selected_gap);

        /**
         * @brief Generate ctrl command to a target pose
         * TODO: fix vector pop and get rid of pose_counter
         * @param pose_arr_odom
         * @return cmd_vel by assigning to pass by reference
         */
        geometry_msgs::Twist ctrlGeneration(geometry_msgs::PoseArray traj);
        
        /**
         * @brief Take current observed gaps and perform gap conversion
         * @param None, directly taken from private variable space
         * @return gap_set, simplfied radial prioritized gaps
         */
        std::vector<potential_gap::Gap> gapManipulate();

        /**
         * @brief Perform trajectory synthesis, subsampling and scoring
         * @param gaps
         * @param trajectories
         */
        std::vector<std::vector<double>> initialTrajGen(std::vector<potential_gap::Gap>, std::vector<geometry_msgs::PoseArray>&);

        /**
         * @brief Callback function to config object
         * @param incoming config
         * @param level Level of incoming config
         */
        void rcfgCallback(potential_gap::pgConfig &config, uint32_t level);

        /**
         * @brief Pick the best trajectory from the current set
         * @param Vector of PoseArray
         * @param Vector of corresponding trajectory scores
         * @return the best trajectory
         */
        geometry_msgs::PoseArray pickTraj(std::vector<geometry_msgs::PoseArray>, std::vector<std::vector<double>>);

        /**
         * @brief Compare to the old trajectory and pick the best one
         * @param incoming trajectory
         * @return the best trajectory  
         */
        geometry_msgs::PoseArray compareToOldTraj(geometry_msgs::PoseArray);

        /**
         * @brief Setter and Getter of Current Trajectory, this is performed in the compareToOldTraj function
         */
        void setCurrentTraj(geometry_msgs::PoseArray);        
        geometry_msgs::PoseArray getCurrentTraj();

        /**
         * @brief Conglomeration of getting a plan Trajectory
         * @return the trajectory
         */
        geometry_msgs::PoseArray getPlanTrajectory();        

        /**
         * @brief Gets the current position along the currently executing Trajectory
         */
        int egoTrajPosition(geometry_msgs::PoseArray curr);

        /**
         * @brief Reset Planner, clears current observedSet
         */
        void reset();
        bool isReplan();
        void setReplan();

        /**
         * @brief Check if the robot has been stuck
         * @param command velocity
         * @return False if robot has been stuck for the past cfg.planning.halt_size iterations
         */
        bool recordAndCheckVel(geometry_msgs::Twist cmd_vel);
    
    };
}

#endif