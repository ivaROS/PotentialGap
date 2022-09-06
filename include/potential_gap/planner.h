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
#include <potential_gap/trajectory_follower.h>
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

#include <tf2_utils/transform_manager.h>

#include <omp.h>

#include <dynamic_reconfigure/server.h>
#include <potential_gap/pgConfig.h>

#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>

#include <turtlebot_trajectory_testing/turtlebot_trajectory_tester.h>
#include <pips_trajectory_testing/pips_trajectory_tester.h>
#include <pips_trajectory_msgs/trajectory_points.h>
#include <pips_trajectory_testing/pips_cc_wrapper.h>
#include <pips_trajectory_testing/depth_image_cc_wrapper.h>
#include <pips_egocylindrical/egocylindrical_image_cc_wrapper.h>
#include <pips_egocircle/egocircle_cc_wrapper.h>

#include <potential_gap/CollisionCheckerConfig.h>

#include <potential_gap/robot_geo_parser.h>

#ifndef PLANNER_H
#define PLANNER_H

namespace potential_gap
{
    struct CollisionResults
    {
        int collision_idx_ = -1;
        pips_trajectory_msgs::trajectory_points local_traj_;
        
        CollisionResults()
        {
            collision_idx_ = -1;
        }

        CollisionResults(int collision_idx, pips_trajectory_msgs::trajectory_points local_traj)
        {
            collision_idx_ = collision_idx;
            local_traj_ = local_traj;
        }
    };

    class Planner
    {
    public:
        typedef TurtlebotGenAndTest::trajectory_ptr trajectory_ptr;
        typedef TurtlebotGenAndTest::traj_func_type traj_func_type;
        typedef TurtlebotGenAndTest::traj_func_ptr traj_func_ptr;
        typedef TurtlebotGenAndTest::trajectory_points trajectory_points;
        typedef TurtlebotGenAndTest::TrajBridge TrajBridge;
        typedef std::shared_ptr<TurtlebotGenAndTest> GenAndTest_ptr;

        std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> cc_wrapper_;
        GenAndTest_ptr traj_tester_;

        bool collision_checker_enable_ = false;
        int cc_type_ = -1;

        using Mutex = boost::mutex;
        using Lock = Mutex::scoped_lock;
        Mutex connect_mutex_;

        typedef dynamic_reconfigure::Server<potential_gap::CollisionCheckerConfig> ReconfigureServer;
        std::shared_ptr<ReconfigureServer> reconfigure_server_;

        void configCB(potential_gap::CollisionCheckerConfig &config, uint32_t level);

    private:
        geometry_msgs::TransformStamped map2rbt;        // Transform
        geometry_msgs::TransformStamped rbt2map;
        geometry_msgs::TransformStamped odom2rbt;
        geometry_msgs::TransformStamped rbt2odom;
        geometry_msgs::TransformStamped map2odom;
        geometry_msgs::TransformStamped cam2odom;
        geometry_msgs::TransformStamped odom2cam;
        geometry_msgs::TransformStamped rbt2cam;
        geometry_msgs::TransformStamped cam2rbt;

        geometry_msgs::PoseStamped goal_rbt_frame;
        geometry_msgs::PoseStamped curr_pose_odom;
        geometry_msgs::PoseStamped rbt_in_rbt;
        geometry_msgs::PoseStamped rbt_in_cam;
        
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;
        std::shared_ptr<tf2_ros::TransformListener> tfListener;
        tf2_ros::TransformBroadcaster goal_br;

        ros::NodeHandle nh, pnh;
        ros::Publisher local_traj_pub;
        ros::Publisher trajectory_pub;
        ros::Publisher gap_vis_pub;
        ros::Publisher selected_gap_vis_pub;
        ros::Publisher ni_traj_pub;
        ros::Publisher ni_traj_pub_other;

        ros::Publisher transformed_laser_pub;
        ros::Publisher virtual_orient_traj_pub;

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

        // std::vector<turtlebot_trajectory_generator::ni_state> ctrl;
        int ctrl_idx = 0;

        geometry_msgs::Pose sharedPtr_pose;
        nav_msgs::Odometry sharedPtr_odom;
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

        bool goal_set = false;
        // Box modification
        bool use_geo_storage_;
        RobotGeoStorage robot_geo_storage_;
        RobotGeoProc robot_geo_proc_;
        bool robot_path_orient_linear_decay_, virtual_path_decay_enable_;
        double speed_factor_;

        // Bezier curve
        bool use_bezier_;

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
         * Return initialization status
         * @param None
         * @return bool initialization status
         */
        bool initialized();

        /**
         * Check if reached goal using euclidean dist
         * @param None, internally stored goal location and robot position
         * @return bool reached
         */
        bool isGoalReached();

        /**
         * call back function to laserscan, externally linked
         * @param msg laser scan msg
         * @return None, laser scan msg stored locally
         */
        void laserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);
        void inflatedlaserScanCB(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

        boost::shared_ptr<sensor_msgs::LaserScan const> transformLaserToRbt(boost::shared_ptr<sensor_msgs::LaserScan const> msg);

        /**
         * call back function to pose, pose information obtained here only used when a new goal is used
         * @param msg pose msg
         * @return None
         */
        void poseCB(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * Interface function for receiving global plan
         * @param plan, vector of PoseStamped
         * @return boolean type on whether successfully registered goal
         */
        bool setGoal(const std::vector<geometry_msgs::PoseStamped> &plan);

        /**
         * update all tf transform at the beginning of every planning cycle
         * @param None, all tf received via TF
         * @return None, all registered via internal variables in TransformStamped
         */
        void updateTF();

        /**
         * select the gap to pass through based on where the goal is
         * TODO: make this polymorphism so more than one strategy can be adopted
         * @param selected_gap that will be returned by the same variable
         * @return selected_gap via the passed in variable
         */
        void vectorSelectGap(potential_gap::Gap & selected_gap);

        /**
         * Generate ctrl command to a target pose
         * TODO: fix vector pop and get rid of pose_counter
         * @param pose_arr_odom
         * @return cmd_vel by assigning to pass by reference
         */
        geometry_msgs::Twist ctrlGeneration(geometry_msgs::PoseArray traj);
        
        /**
         * Take current observed gaps and perform gap conversion
         * @param None, directly taken from private variable space
         * @return gap_set, simplfied radial prioritized gaps
         */
        std::vector<potential_gap::Gap> gapManipulate();

        /**
         * 
         *
         */
        std::vector<std::vector<double>> initialTrajGen(std::vector<potential_gap::Gap>, std::vector<geometry_msgs::PoseArray>&, std::vector<geometry_msgs::PoseArray>& virtual_decayed);

        /**
         * Callback function to config object
         * @param incoming config
         * @param level Level of incoming config
         */
        void rcfgCallback(potential_gap::pgConfig &config, uint32_t level);

        /**
         * Pick the best trajectory from the current set
         * @param Vector of PoseArray
         * @param Vector of corresponding trajectory scores
         * @return the best trajectory
         */
        geometry_msgs::PoseArray pickTraj(std::vector<geometry_msgs::PoseArray>, std::vector<std::vector<double>>, std::vector<geometry_msgs::PoseArray> virtual_path, geometry_msgs::PoseArray& chosen_virtual_path);

        /**
         * Compare to the old trajectory and pick the best one
         * @param incoming trajectory
         * @return the best trajectory  
         */
        geometry_msgs::PoseArray compareToOldTraj(geometry_msgs::PoseArray, geometry_msgs::PoseArray& virtual_curr_traj);

        geometry_msgs::PoseArray getOrientDecayedPath(geometry_msgs::PoseArray);

        CollisionResults checkCollision(const geometry_msgs::PoseArray path);

        /**
         * Setter and Getter of Current Trajectory, this is performed in the compareToOldTraj function
         */
        void setCurrentTraj(geometry_msgs::PoseArray);        
        geometry_msgs::PoseArray getCurrentTraj();

        /**
         * Conglomeration of getting a plan Trajectory
         * @return the trajectory
         */
        geometry_msgs::PoseArray getPlanTrajectory();    

        geometry_msgs::PoseArray getSinglePath();

        void pubPickedTraj(geometry_msgs::PoseArray picked_traj);

        geometry_msgs::PoseArray getLocalPath(geometry_msgs::PoseArray input_path);

        bool reachedTrajEnd();

        /**
         * Gets the current position along the currently executing Trajectory
         */
        int egoTrajPosition(geometry_msgs::PoseArray curr);


        /**
         * Reset Planner, clears current observedSet
         */
        void reset();
        bool isReplan();
        void setReplan();

        /**
         * Check if the robot has been stuck
         * @param command velocity
         * @return False if robot has been stuck for the past cfg.planning.halt_size iterations
         */
        bool recordAndCheckVel(geometry_msgs::Twist cmd_vel);
        
        void setCCWrapper(const std::shared_ptr<pips_trajectory_testing::PipsCCWrapper>& cc_wrapper)
        {
            cc_wrapper_ = cc_wrapper;
        }

        std::shared_ptr<pips_trajectory_testing::PipsCCWrapper> getCCWrapper()
        {
            return cc_wrapper_;
        }

        bool ccEnabled()
        {
            return collision_checker_enable_;
        }
    };
}

#endif