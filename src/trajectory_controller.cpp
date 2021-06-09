#include <potential_gap/trajectory_controller.h>

namespace potential_gap{
    TrajectoryController::TrajectoryController(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg) {
        projection_viz = nh.advertise<visualization_msgs::Marker>("po_dir", 10);
        cfg_ = & cfg;
        thres = 0.1;
        last_time = ros::Time::now();
    }

    void TrajectoryController::updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const> msg)
    {
        boost::mutex::scoped_lock lock(egocircle_l);
        msg_ = msg;
    }

    bool TrajectoryController::leqThres(const double dist) {
        return dist <= thres;
    }

    bool TrajectoryController::geqThres(const double dist) {
        return dist >= thres;
    }

    double TrajectoryController::polDist(float l1, float t1, float l2, float t2) {
        return abs(double(pow(l1, 2) + pow(l2, 2) - 2 * l1 * l2 * std::cos(t1 - t2)));
    }

    geometry_msgs::Twist TrajectoryController::controlLaw(
        geometry_msgs::Pose current, nav_msgs::Odometry desired,
        sensor_msgs::LaserScan inflated_egocircle, geometry_msgs::PoseStamped rbt_in_cam_lc
    ) {
        // Setup Vars
        boost::mutex::scoped_lock lock(egocircle_l);
        bool holonomic = cfg_->planning.holonomic;
        bool full_fov = cfg_->planning.full_fov;
        bool projection_operator = cfg_->planning.projection_operator;
        double k_turn_ = cfg_->control.k_turn;
        if (holonomic && full_fov) k_turn_ = 0.8;
        double k_drive_x_ = cfg_->control.k_drive_x;
        double k_drive_y_ = cfg_->control.k_drive_y;
        double k_po_ = cfg_->projection.k_po;
        float v_ang_const = cfg_->control.v_ang_const;
        float v_lin_x_const = cfg_->control.v_lin_x_const;
        float v_lin_y_const = cfg_->control.v_lin_y_const;
        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;
        float r_norm_offset = cfg_->projection.r_norm_offset; 
        float k_po_turn_ = cfg_->projection.k_po_turn;

        geometry_msgs::Twist cmd_vel;
        geometry_msgs::Point position = current.position;
        geometry_msgs::Quaternion orientation = current.orientation;

        tf::Quaternion q_c(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_c(q_c);
        double c_roll, c_pitch, c_yaw;
        m_c.getRPY(c_roll, c_pitch, c_yaw);

        Eigen::Matrix2cd g_curr = getComplexMatrix(position.x, position.y, c_yaw);

        position = desired.pose.pose.position;
        orientation = desired.pose.pose.orientation;

        tf::Quaternion q_d(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w);
        tf::Matrix3x3 m_d(q_d);
        double d_roll, d_pitch, d_yaw;
        m_d.getRPY(d_roll, d_pitch, d_yaw);

        Eigen::Matrix2cd g_des = getComplexMatrix(position.x, position.y, d_yaw);

        Eigen::Matrix2cd g_error = g_curr.inverse() * g_des;
        float theta_error = std::arg(g_error(0, 0));
        float x_error = g_error.real()(0, 1);
        float y_error = g_error.imag()(0, 1);


        float u_add_x = 0;
        float u_add_y = 0;
        float v_ang_fb = 0;
        double v_lin_x_fb = 0;
        double v_lin_y_fb = 0;

        if (cfg_->man.man_ctrl) {
            ROS_INFO_STREAM("Manual Control");
            v_ang_fb = cfg_->man.man_theta;
            v_lin_x_fb = cfg_->man.man_x;
            v_lin_y_fb = cfg_->man.man_y;
        } else {
            v_ang_fb = theta_error * k_turn_;
            v_lin_x_fb = x_error * k_drive_x_;
            v_lin_y_fb = y_error * k_drive_y_;
        }

        float min_dist_ang = 0;
        float min_dist = 0;

        float min_diff_x = 0;
        float min_diff_y = 0;
        
        Eigen::Vector3d comp;
        double prod_mul;
        Eigen::Vector2d si_der;
        Eigen::Vector2d v_err(v_lin_x_fb, v_lin_y_fb);


        if (inflated_egocircle.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect controlLaw");
        }

        if(projection_operator)
        {
            std::vector<double> min_dist_arr(inflated_egocircle.ranges.size());
            for (int i = 0; i < min_dist_arr.size(); i++) {
                float angle = i * inflated_egocircle.angle_increment - M_PI;
                float dist = inflated_egocircle.ranges.at(i);
                min_dist_arr.at(i) = dist2Pose(angle, dist, rbt_in_cam_lc.pose);
            }
            int min_idx = std::min_element( min_dist_arr.begin(), min_dist_arr.end() ) - min_dist_arr.begin();

            last_time = ros::Time::now();
            float r_max = r_norm + r_norm_offset;
            min_dist = (float) min_dist_arr.at(min_idx);
            min_dist = min_dist >= r_max ? r_max : min_dist;
            if (min_dist <= 0) ROS_INFO_STREAM("Min dist <= 0, : " << min_dist);
            min_dist = min_dist <= 0 ? 0.01 : min_dist;

            min_dist_ang = (float)(min_idx) * inflated_egocircle.angle_increment + inflated_egocircle.angle_min;
            float min_x = min_dist * cos(min_dist_ang) - rbt_in_cam_lc.pose.position.x;
            float min_y = min_dist * sin(min_dist_ang) - rbt_in_cam_lc.pose.position.y;
            min_dist = sqrt(pow(min_x, 2) + pow(min_y, 2));

            min_diff_x = - min_x;
            min_diff_y = - min_y;
            comp = projection_method(min_diff_x, min_diff_y);
            si_der = Eigen::Vector2d(comp(0), comp(1));
            si_der(1) /= 3;
            prod_mul = v_err.dot(si_der);

            if(comp(2) >= 0 && prod_mul <= 0)
            {
                u_add_x = comp(2) * prod_mul * - comp(0);
                u_add_y = comp(2) * prod_mul * - comp(1);
            }

            // Visualization
            {
                visualization_msgs::Marker res;
                res.header.frame_id = cfg_->robot_frame_id;
                res.type = visualization_msgs::Marker::ARROW;
                res.action = visualization_msgs::Marker::ADD;
                res.pose.position.x = 0;
                res.pose.position.y = 0;
                res.pose.position.z = 1;
                double dir = std::atan2(u_add_y, u_add_x);
                tf2::Quaternion dir_quat;
                dir_quat.setRPY(0, 0, dir);
                res.pose.orientation = tf2::toMsg(dir_quat);

                float var = sqrt(pow(u_add_y, 2) + pow(u_add_x, 2));
                res.scale.x = (var - 5) - log(1 + exp(var - 5)) + 5;
                res.scale.y = 0.01; 
                res.scale.y = 0.01; 
                res.scale.y = 0.01; 
                res.scale.z = 0.01;
                
                res.color.a = 1;
                res.color.r = 0.9;
                res.color.g = 0.9;
                res.color.b = 0.9;
                res.id = 0;
                projection_viz.publish(res);

                // Min Direction
                res.header.frame_id = cfg_->sensor_frame_id;
                res.scale.x = 1;
                dir_quat.setRPY(0, 0, min_dist_ang);
                res.pose.orientation = tf2::toMsg(dir_quat);
                res.id = 1;
                res.color.a = 0.5;
                res.pose.position.z = 0.9;
                // projection_viz.publish(res);

                res.header.frame_id = cfg_->robot_frame_id;
                res.type = visualization_msgs::Marker::SPHERE;
                res.action = visualization_msgs::Marker::ADD;
                res.pose.position.x = -min_diff_x;
                res.pose.position.y = -min_diff_y;
                res.pose.position.z = 1;
                res.scale.x = 0.1;
                res.scale.y = 0.1;
                res.scale.z = 0.1;
                res.id = 2;
            }
        } else {
            ROS_DEBUG_STREAM_THROTTLE(10, "Projection operator off");
        }

        // Make sure no ejection
        u_add_x = std::min(u_add_x, float(0));

        if(holonomic)
        {
            v_ang_fb = v_ang_fb + v_ang_const;
            v_lin_x_fb = abs(theta_error) > M_PI / 3? 0 : v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;
            v_lin_y_fb = abs(theta_error) > M_PI / 3? 0 : v_lin_y_fb + v_lin_y_const + k_po_ * u_add_y;

            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;
        }
        else
        {
            v_ang_fb = v_ang_fb + v_lin_y_fb + k_po_turn_ * u_add_y + v_ang_const;
            v_lin_x_fb = v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;

            if (projection_operator && min_dist_ang > - M_PI / 4 && min_dist_ang < M_PI / 4 && min_dist < cfg_->rbt.r_inscr)
            {
                // No x linear
                v_lin_x_fb = 0;
                v_ang_fb *= 2;
            }

            v_lin_y_fb = 0;

            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;
        }

        cmd_vel.linear.x = std::max(-cfg_->control.vx_absmax, std::min(cfg_->control.vx_absmax, v_lin_x_fb));
        cmd_vel.linear.y = std::max(-cfg_->control.vy_absmax, std::min(cfg_->control.vy_absmax, v_lin_y_fb));
        cmd_vel.angular.z = v_ang_fb;
        return cmd_vel;
    }

    Eigen::Vector3d TrajectoryController::projection_method(float min_diff_x, float min_diff_y) {
        float r_min = cfg_->projection.r_min;
        float r_norm = cfg_->projection.r_norm;

        float min_dist = sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2));
        float si = (r_min / min_dist - r_min / r_norm) / (1. - r_min / r_norm);
        float base_const = sqrt(pow(min_dist, 3)) * (r_min - r_norm);
        float up_const = r_min * r_norm;
        float si_der_x = up_const * - min_diff_x / base_const;
        float si_der_y = up_const * - min_diff_y / base_const;

        float norm_si_der = sqrt(pow(si_der_x, 2) + pow(si_der_y, 2));
        float norm_si_der_x = si_der_x / norm_si_der;
        float norm_si_der_y = si_der_y / norm_si_der;
        return Eigen::Vector3d(norm_si_der_x, norm_si_der_y, si);
    }

    Eigen::Matrix2cd TrajectoryController::getComplexMatrix(
        double x, double y, double quat_w, double quat_z)
    {
        std::complex<double> phase(quat_w, quat_z);
        phase = phase * phase;

        Eigen::Matrix2cd g(2, 2);
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }

    Eigen::Matrix2cd TrajectoryController::getComplexMatrix(
        double x, double y, double theta)
    {
        std::complex<double> phase(std::cos(theta), std::sin(theta));

        Eigen::Matrix2cd g(2, 2);
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }


    int TrajectoryController::targetPoseIdx(geometry_msgs::Pose curr_pose, potential_gap::TrajPlan ref_pose) {
        // Find pose right ahead
        std::vector<double> pose_diff(ref_pose.poses.size());
        for (int i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr_pose.position.x - ref_pose.poses[i].position.x, 2) + 
                                pow(curr_pose.position.y - ref_pose.poses[i].position.y, 2)) + 
                                0.5 * (1 - (curr_pose.orientation.x * ref_pose.poses[i].orientation.x + 
                                        curr_pose.orientation.y * ref_pose.poses[i].orientation.y +
                                        curr_pose.orientation.z * ref_pose.poses[i].orientation.z +
                                        curr_pose.orientation.w * ref_pose.poses[i].orientation.w)
                                );
        }

        auto min_element_iter = std::min_element(pose_diff.begin(), pose_diff.end());
        int target_pose = std::distance(pose_diff.begin(), min_element_iter) + cfg_->control.ctrl_ahead_pose;
        return std::min(target_pose, int(ref_pose.poses.size() - 1));
    }


    potential_gap::TrajPlan TrajectoryController::trajGen(geometry_msgs::PoseArray orig_traj)
    {
        potential_gap::TrajPlan traj;
        traj.header.frame_id = cfg_->odom_frame_id;
        for(size_t i = 0; i < orig_traj.poses.size(); i++)
        {
            geometry_msgs::Pose ni_pose = orig_traj.poses[i];
            geometry_msgs::Twist ni_twist;
            traj.poses.push_back(ni_pose);
            traj.twist.push_back(ni_twist);
        }
        return traj;
    }

    double TrajectoryController::dist2Pose(float theta, float dist, geometry_msgs::Pose pose) {
        float x = dist * std::cos(theta);
        float y = dist * std::sin(theta);
        return sqrt(pow(pose.position.x - x, 2) + pow(pose.position.y - y, 2));
    }

    Eigen::Vector2d TrajectoryController::car2pol(Eigen::Vector2d a) {
        return Eigen::Vector2d(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2d TrajectoryController::pol2car(Eigen::Vector2d a) {
        return Eigen::Vector2d(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }


}