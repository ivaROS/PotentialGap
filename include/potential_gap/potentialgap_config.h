#ifndef PG_CONFIG_H
#define PG_CONFIG_H

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>

namespace potential_gap {
    class PotentialGapConfig {
        public:
            std::string map_frame_id = "map"; /**< Map frame ID */
            std::string odom_frame_id = "TBD"; /**< Odometry frame ID */
            std::string robot_frame_id = "TBD"; /**< Robot frame ID */
            std::string sensor_frame_id = "TBD"; /**< Sensor frame ID */

            std::string odom_topic = "TBD"; /**< Odometry ROS topic */
            std::string acc_topic = "TBD"; /**< IMU ROS topic */
            std::string scan_topic = "TBD"; /**< Laser scan ROS topic */
            std::string ped_topic = "/pedsim_simulator/simulated_agents";


            struct Robot {
                float r_inscr = 0.2;
            } rbt;

            /**
            * \brief Hyperparameters for laser scan
            */
            struct Scan
            {
                // will get overriden in updateParamFromScan
                float angle_min = -M_PI; /**< minimum angle value in scan */
                float angle_max = M_PI; /**< maximum angle value in scan */
                int half_scan = 256; /**< Half of total rays in scan (integer) */
                float half_scan_f = 256.; /**< Half of total rays in scan (float) */
                int full_scan = 512; /**< Total ray count in scan (integer) */
                float full_scan_f = 512.; /**< Total ray count in scan (float) */
                float angle_increment = (2 * M_PI) / (full_scan_f - 1); /**< Angular increment between consecutive scan indices */
                float range_min = 0.03; /**< Minimum detectable range in scan */
                float range_max = -1e10; /**< Maximum detectable range in scan */
            } scan;

            struct GapVisualization {
                int min_resoln = 2;
                bool close_gap_vis = true;
                bool follow_the_gap_vis = true;
                bool fig_gen = false;
                double viz_jitter = 0.05;
                bool debug_viz = true;
            } gap_viz;

            struct GapManipulation {
                double gap_diff = 0.1;
                double epsilon2 = 0.18;
                double epsilon1 = 0.18;
                double sigma;
                double reduction_threshold = M_PI / 2;
                double reduction_target = M_PI / 4;
                int max_idx_diff = 256;
                bool radial_extend = true;
                bool axial_convert = true;
                double rot_ratio = 1.5;
            } gap_manip;

            struct ControlParams {
                double k_drive_x = 3.5;
                double k_drive_y = 3.5;
                double k_turn = 0.5;
                double v_ang_const = 0.0;
                double v_lin_x_const = 0.0;
                double v_lin_y_const = 0.0;
                int ctrl_ahead_pose = 1;
                double vx_absmax = 0.5;
                double vy_absmax = 0.5;
            } control;
            
            struct ProjectionParam {
                double k_po = 1.0;
                double r_min = 0.5;
                double r_norm = 1.0;
                double r_norm_offset = 0.5;
                double k_po_turn = 1.0;
            } projection;

            struct Waypoint {
                int global_plan_lookup_increment = 75;
                double global_plan_change_tolerance = 0.1;
            } waypoint;

            struct PlanningMode {
                bool holonomic = true;
                bool full_fov = true;
                bool projection_operator = true;
                bool niGen_s;
                bool far_feasible = true;
                int num_feasi_check = 20;
                int halt_size;
            } planning;

            struct Goal {
                double goal_tolerance = 0.5;
                double waypoint_tolerance = 0.1;
            } goal;

            struct Trajectory {
                bool synthesized_frame = true;
                double scale = 1.0;
                double integrate_maxt;
                double integrate_stept;
                double rmax;
                double cobs;
                double w;
                double inf_ratio = 1.2;
                double terminal_weight = 10;
                double waypoint_ratio = 1.5;
            } traj;

        PotentialGapConfig() {
            map_frame_id = "map";
            odom_frame_id = "odom";
            robot_frame_id = "base_link";
            sensor_frame_id = "camera_link";

            gap_viz.min_resoln = 2;
            gap_viz.close_gap_vis = true;
            gap_viz.follow_the_gap_vis = true;
            gap_viz.fig_gen = false;
            gap_viz.viz_jitter = 0.05;
            gap_viz.debug_viz = true;

            gap_manip.gap_diff = 0.1;
            gap_manip.epsilon1 = 0.18;
            gap_manip.epsilon2 = 0.18;
            gap_manip.sigma = 0.3;
            gap_manip.rot_ratio = 1.5;
            gap_manip.reduction_threshold = M_PI / 2;
            gap_manip.reduction_target = M_PI / 4;
            gap_manip.radial_extend = true;
            gap_manip.axial_convert = true;
            
            control.k_drive_x = 3.5;
            control.k_drive_y = 3.5;
            control.k_turn = 0.5;
            control.v_ang_const = 0;
            control.v_lin_x_const = 0;
            control.v_lin_y_const = 0;
            control.ctrl_ahead_pose = 1;
            control.vx_absmax = 0.5;
            control.vy_absmax = 0.5;

            projection.k_po = 1.0;
            projection.k_po_turn = 1.0;
            projection.r_min = 0.5;
            projection.r_norm = 1.0;
            projection.r_norm_offset = 0.5;

            waypoint.global_plan_lookup_increment = 75;
            waypoint.global_plan_change_tolerance = 0.1;

            planning.holonomic = true;
            planning.full_fov = true;
            planning.projection_operator = true;
            planning.niGen_s = true;
            planning.num_feasi_check = 20;
            planning.far_feasible = true;
            planning.halt_size = 5;

            goal.goal_tolerance = 0.5;
            goal.waypoint_tolerance = 0.1;
            
            traj.synthesized_frame = true;
            traj.scale = 1;
            traj.integrate_maxt = 10;
            traj.integrate_stept = 0.5;
            traj.rmax = 0.5;
            traj.cobs = -1;
            traj.w = 3;
            traj.inf_ratio = 1.2;
            traj.terminal_weight = 10;
            traj.waypoint_ratio = 1.5;

            rbt.r_inscr = 0.18;
        }

        /**
        * \brief Load in planner hyperparameters from node handle (specified in launch file and yamls)
        */
        void loadRosParamFromNodeHandle(const std::string & name);

        /**
        * \brief Load in hyperparameters from current laser scan
        */
        void updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr);

        // void reconfigure(pgConfig& cfg);

        boost::mutex & configMutex() {return config_mutex;}

        private: 
            boost::mutex config_mutex; 
    };
}

#endif