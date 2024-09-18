#include <potential_gap/potentialgap_config.h>

namespace potential_gap {
    void PotentialGapConfig::loadRosParamFromNodeHandle(const std::string & name)
    {
        ros::NodeHandle nh("~/" + name);

        ROS_INFO_STREAM("Setting nh to: " << "~/" << name);

        std::string model;
        nh.param("/model", model, model); // Must write as "/model" with leading slash

        if (model == "rto")
        {
            // format: key, value, default value
            odom_frame_id = model + "/odom";
            robot_frame_id = model + "/base_link";
            sensor_frame_id = model + "/hokuyo_link";
            
            odom_topic = "odom"; // model + "/odom";
            acc_topic = "acc"; // model + "/acc";
            scan_topic = "scan"; // model + "/scan";
            
            // Robot
            nh.param("robot_radius", rbt.r_inscr, rbt.r_inscr);

            // Environment
            // nh.param("num_agents", env.num_agents, env.num_agents);

            nh.param("max_vel_x",control.vx_absmax, control.vx_absmax);
            nh.param("max_vel_y",control.vy_absmax, control.vy_absmax);

            // Scan
            nh.param("max_range", scan.range_max, scan.range_max);

            // Goal Param
            nh.param("goal_tolerance", goal.goal_tolerance, goal.goal_tolerance);
            nh.param("waypoint_tolerance", goal.waypoint_tolerance, goal.waypoint_tolerance);

            // Gap Manipulation
            nh.param("gap_diff", gap_manip.gap_diff, gap_manip.gap_diff);
            nh.param("epsilon2", gap_manip.epsilon2, gap_manip.epsilon2);
            nh.param("epsilon1", gap_manip.epsilon1, gap_manip.epsilon1);
            nh.param("sigma", gap_manip.sigma, gap_manip.sigma);
            nh.param("rot_ratio", gap_manip.rot_ratio, gap_manip.rot_ratio);
            nh.param("reduction_threshold", gap_manip.reduction_threshold, gap_manip.reduction_threshold);
            nh.param("reduction_target", gap_manip.reduction_target, gap_manip.reduction_target);        
            nh.param("max_idx_diff", gap_manip.max_idx_diff, gap_manip.max_idx_diff);
            nh.param("radial_extend", gap_manip.radial_extend, gap_manip.radial_extend);
            nh.param("axial_convert", gap_manip.axial_convert, gap_manip.axial_convert);

            // Control Params
            nh.param("k_drive_x",control.k_drive_x, control.k_drive_x);
            nh.param("k_drive_y",control.k_drive_y, control.k_drive_y);
            nh.param("k_turn",control.k_turn, control.k_turn);
            nh.param("v_ang_const",control.v_ang_const, control.v_ang_const);
            nh.param("v_lin_x_const",control.v_lin_x_const, control.v_lin_x_const);
            nh.param("v_lin_y_const",control.v_lin_y_const, control.v_lin_y_const);
            nh.param("ctrl_ahead_pose",control.ctrl_ahead_pose, control.ctrl_ahead_pose);

            // Projection Params
            nh.param("k_po", projection.k_po, projection.k_po);
            nh.param("k_po_turn", projection.k_po, projection.k_po);
            nh.param("r_min", projection.r_min, projection.r_min);
            nh.param("r_norm", projection.r_norm, projection.r_norm);
            nh.param("r_norm_offset", projection.r_norm_offset, projection.r_norm_offset);

            // Waypoint Params
            nh.param("global_plan_lookup_increment", waypoint.global_plan_lookup_increment, waypoint.global_plan_lookup_increment);
            nh.param("global_plan_change_tolerance", waypoint.global_plan_change_tolerance, waypoint.global_plan_change_tolerance);

            // General Planning Mode Params
            nh.param("holonomic", planning.holonomic, planning.holonomic);
            nh.param("full_fov", planning.full_fov, planning.full_fov);
            nh.param("projection_operator", planning.projection_operator, planning.projection_operator);
            nh.param("niGen_s", planning.niGen_s, planning.niGen_s);
            nh.param("num_feasi_check", planning.num_feasi_check, planning.num_feasi_check);
            nh.param("num_feasi_check", planning.far_feasible, planning.far_feasible);

            // Trajectory
            nh.param("synthesized_frame", traj.synthesized_frame, traj.synthesized_frame);
            nh.param("scale", traj.scale, traj.scale);
            nh.param("integrate_maxt", traj.integrate_maxt, traj.integrate_maxt);
            nh.param("integrate_stept", traj.integrate_stept, traj.integrate_stept);
            nh.param("rmax", traj.rmax, traj.rmax);
            nh.param("inf_ratio", traj.inf_ratio, traj.inf_ratio);
            nh.param("terminal_weight", traj.terminal_weight, traj.terminal_weight);
            nh.param("waypoint_ratio", traj.waypoint_ratio, traj.waypoint_ratio);

            // Gap Visualization
            nh.param("min_resoln", gap_viz.min_resoln, gap_viz.min_resoln);
            nh.param("close_gap", gap_viz.close_gap_vis, gap_viz.close_gap_vis);
            nh.param("follow_the_gap", gap_viz.follow_the_gap_vis, gap_viz.follow_the_gap_vis);
            nh.param("fig_gen", gap_viz.fig_gen, gap_viz.fig_gen);
            nh.param("viz_jitter", gap_viz.viz_jitter, gap_viz.viz_jitter);
            nh.param("debug_viz", gap_viz.debug_viz, gap_viz.debug_viz);

        } else
        {
            throw std::runtime_error("Model " + model + " not implemented!");
        }
    }

    void PotentialGapConfig::updateParamFromScan(boost::shared_ptr<sensor_msgs::LaserScan const> scanPtr)
    {
        sensor_msgs::LaserScan incomingScan = *scanPtr.get();
        scan.angle_min = incomingScan.angle_min;
        scan.angle_max = incomingScan.angle_max;
        scan.full_scan = incomingScan.ranges.size();
        scan.full_scan_f = float(scan.full_scan);
        scan.half_scan = scan.full_scan / 2;
        scan.half_scan_f = float(scan.half_scan);        
        scan.angle_increment = (2 * M_PI) / (scan.full_scan_f - 1);

        scan.range_max = incomingScan.range_max; // this is the maximum possible range, not the max range within a particular scan
        scan.range_min = incomingScan.range_min;

        // ROS_INFO_STREAM("scan.angle_increment: " << scan.angle_increment);
    }


}