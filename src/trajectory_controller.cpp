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

    [[deprecated("Not Used, Deemed Unnecessary")]]
    std::vector<geometry_msgs::Point> TrajectoryController::findLocalLine(int idx) {
        auto egocircle = *msg_.get();
        std::vector<double> dist(egocircle.ranges.size());

        if (!msg_) {
            return std::vector<geometry_msgs::Point>(0);
        }
        
        if (egocircle.ranges.size() < 500) {
            ROS_FATAL_STREAM("Scan range incorrect findLocalLine");
        }


        // ROS_INFO_STREAM("Min idx: " << idx);

        for (int i = 1; i < dist.size(); i++) {
            float l1 = egocircle.ranges.at(i);
            float t1 = float(i) * egocircle.angle_increment + egocircle.angle_min;
            float l2 = egocircle.ranges.at(i - 1);
            float t2 = float(i - 1) * egocircle.angle_increment + egocircle.angle_min;
            if (l1 > 2.9) {
                dist.at(i) = 10;
            } else {
                dist.at(i) = polDist(l1, t1, l2, t2);
            } 
        }

        dist.at(0) = polDist(egocircle.ranges.at(0), egocircle.angle_min, egocircle.ranges.at(511), float(511) * egocircle.angle_increment + egocircle.angle_min);

        auto result_fwd = std::find_if(dist.begin() + idx, dist.end(), 
            std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));

        // ROS_INFO_STREAM("Find 1 terminated");
        auto res_rev = std::find_if(dist.rbegin() + (dist.size() - idx), dist.rend(),
            std::bind1st(std::mem_fun(&TrajectoryController::geqThres), this));
        
        // ROS_INFO_STREAM("Find terminated");

        if (res_rev == dist.rend()) {
            // ROS_WARN_STREAM("Edge case fix later");
            return std::vector<geometry_msgs::Point>(0);
        }

        int idx_fwd = std::distance(dist.begin(), std::prev(result_fwd));
        int idx_rev = std::distance(res_rev, dist.rend());
        // ROS_INFO_STREAM("Idx fwd: " << idx_fwd << ", Cent: " << idx << ", Rev: " << idx_rev);

        int min_idx_range = 0;
        int max_idx_range = int(egocircle.ranges.size() - 1);
        if (idx_fwd < min_idx_range || idx_fwd > max_idx_range || idx_rev < min_idx_range || idx_rev > max_idx_range) {
            return std::vector<geometry_msgs::Point>(0);
        }
        
        float dist_fwd = egocircle.ranges.at(idx_fwd);
        float dist_rev = egocircle.ranges.at(idx_rev);
        float dist_cent = egocircle.ranges.at(idx);
        // ROS_INFO_STREAM("Dist fwd: " << dist_fwd << ", Cent: " << dist_cent << ", Rev: " << dist_rev);

        double angle_fwd = double(idx_fwd) * egocircle.angle_increment + egocircle.angle_min;
        double angle_rev = double(idx_rev) * egocircle.angle_increment + egocircle.angle_min;
        
        if (idx_fwd < idx || idx_rev > idx) {
            // ROS_WARN_STREAM("Line index weird, return no line");
            // ROS_WARN_STREAM("Fwd Idx: " << idx_fwd << ", Cent: " << idx << ", Rev" << idx_rev);
            return std::vector<geometry_msgs::Point>(0);
        }

        Eigen::Vector2d fwd_pol(dist_fwd, angle_fwd);
        Eigen::Vector2d rev_pol(dist_rev, angle_rev);
        Eigen::Vector2d cent_pol(dist_cent, double(idx) * egocircle.angle_increment + egocircle.angle_min);
        Eigen::Vector2d fwd_car = pol2car(fwd_pol);
        Eigen::Vector2d rev_car = pol2car(rev_pol);
        Eigen::Vector2d cent_car = pol2car(cent_pol);

        Eigen::Vector2d pf;
        Eigen::Vector2d pr;

        if (dist_cent < dist_fwd && dist_cent < dist_rev) {
            // ROS_INFO_STREAM("Non line");
            Eigen::Vector2d a = cent_car - fwd_car;
            Eigen::Vector2d b = rev_car - fwd_car;
            Eigen::Vector2d a1 = (a.dot(b / b.norm())) * (b / b.norm());
            Eigen::Vector2d a2 = a - a1;
            pf = fwd_car + a2;
            pr = rev_car + a2;
        } else {
            pf = fwd_car;
            pr = rev_car;
        }

        geometry_msgs::Point lower_point;
        lower_point.x = pf(0);
        lower_point.y = pf(1);
        lower_point.z = 3;
        geometry_msgs::Point upper_point;
        upper_point.x = pr(0);
        upper_point.y = pr(1);
        upper_point.z = 3;
        // if form convex hull
        std::vector<geometry_msgs::Point> retArr(0);
        retArr.push_back(lower_point);
        retArr.push_back(upper_point);
        return retArr;
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
        sensor_msgs::LaserScan inflated_egocircle, geometry_msgs::PoseStamped init_pose
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

        // auto inflated_egocircle = *msg_.get();
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
        double v_ang_fb = 0;
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

        // ROS_INFO_STREAM(init_pose.pose);
        
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
                min_dist_arr.at(i) = dist2Pose(angle, dist, init_pose.pose);
            }
            int min_idx = std::min_element( min_dist_arr.begin(), min_dist_arr.end() ) - min_dist_arr.begin();

            // ROS_INFO_STREAM("Local Line Start");
            std::vector<geometry_msgs::Point> vec = findLocalLine(min_idx);
            // ROS_INFO_STREAM("Local Line End");
            if (vec.size() > 1) {
                // Visualization and Recenter
                vec.at(0).x -= init_pose.pose.position.x;
                vec.at(0).y -= init_pose.pose.position.y;
                vec.at(1).x -= init_pose.pose.position.x;
                vec.at(1).y -= init_pose.pose.position.y;

                std::vector<std_msgs::ColorRGBA> color;
                std_msgs::ColorRGBA std_color;
                std_color.a = 1;
                std_color.r = 1;
                std_color.g = 0.1;
                std_color.b = 0.1;
                color.push_back(std_color);
                color.push_back(std_color);
                visualization_msgs::Marker line_viz;
                line_viz.header.frame_id = cfg_->robot_frame_id;
                line_viz.type = visualization_msgs::Marker::LINE_STRIP;
                line_viz.action = visualization_msgs::Marker::ADD;
                line_viz.points = vec;
                line_viz.colors = color;
                line_viz.scale.x = 0.01;
                line_viz.scale.y = 0.1;
                line_viz.scale.z = 0.1;
                line_viz.id = 10;
                projection_viz.publish(line_viz);
            }

            // ROS_DEBUG_STREAM("Elapsed: " << (ros::Time::now() - last_time).toSec());
            last_time = ros::Time::now();
            float r_max = r_norm + r_norm_offset;
            min_dist = (float) min_dist_arr.at(min_idx);
            min_dist = min_dist >= r_max ? r_max : min_dist;
            if (min_dist <= 0) ROS_INFO_STREAM("Min dist <= 0, : " << min_dist);
            min_dist = min_dist <= 0 ? 0.01 : min_dist;
            // min_dist -= cfg_->rbt.r_inscr / 2;

            min_dist_ang = (float)(min_idx) * inflated_egocircle.angle_increment + inflated_egocircle.angle_min;
            float min_x = min_dist * cos(min_dist_ang) - init_pose.pose.position.x;
            float min_y = min_dist * sin(min_dist_ang) - init_pose.pose.position.y;
            min_dist = sqrt(pow(min_x, 2) + pow(min_y, 2));

            // ROS_INFO_STREAM("min_dist: " << min_dist);

            if (cfg_->man.line && vec.size() > 0) {
                // Dist to 
                Eigen::Vector2d pt1(vec.at(0).x, vec.at(0).y);
                Eigen::Vector2d pt2(vec.at(1).x, vec.at(1).y);
                Eigen::Vector2d rbt(0, 0);
                Eigen::Vector2d a = rbt - pt1;
                Eigen::Vector2d b = pt2 - pt1;
                Eigen::Vector2d c = rbt - pt2;
                
                if (a.dot(b) < 0) {
                    // Dist to pt1
                    // ROS_INFO_STREAM("Pt1");
                    min_diff_x = - pt1(0);
                    min_diff_y = - pt1(1);
                    comp = projection_method(min_diff_x, min_diff_y);
                    si_der = Eigen::Vector2d(comp(0), comp(1));
                    prod_mul = v_err.dot(si_der);
                } else if (c.dot(-b) < 0) {
                    // Dist to pt2
                    // ROS_INFO_STREAM("Pt2");
                    min_diff_x = - pt2(0);
                    min_diff_y = - pt2(1);
                    comp = projection_method(min_diff_x, min_diff_y);
                    si_der = Eigen::Vector2d(comp(0), comp(1));
                    prod_mul = v_err.dot(si_der);
                } else {
                    // Dist to line
                    // ROS_INFO_STREAM("Line");
                    // ROS_INFO_STREAM("Pt1: " << pt1(0) << ", " << pt1(1) << ", Pt2: " << pt2(0) << ", " << pt2(1));
                    double line_dist = (pt1(0) * pt2(1) - pt2(0) * pt1(1)) / (pt1 - pt2).norm();
                    double sign;
                    sign = line_dist < 0 ? -1 : 1;
                    line_dist *= sign;
                    // ROS_INFO_STREAM("Dist to line: " << line_dist);
                    
                    double line_si = (r_min / line_dist - r_min / r_norm) / (1. - r_min / r_norm);
                    double line_si_der_base = r_min / (pow(line_dist, 2) * (r_min / r_norm - 1));
                    double line_si_der_x = - (pt2(1) - pt1(1)) / (pt1 - pt2).norm() * line_si_der_base;
                    double line_si_der_y = - (pt1(0) - pt2(0)) / (pt1 - pt2).norm() * line_si_der_base;
                    // ROS_INFO_STREAM("si: " << line_si);
                    // ROS_INFO_STREAM("Der x: " << line_si_der_x << ", Der y: " << line_si_der_y);
                    Eigen::Vector2d der(line_si_der_x, line_si_der_y);
                    der /= der.norm();
                    // ROS_INFO_STREAM("Normalized der" << der);
                    comp = Eigen::Vector3d(der(0), der(1), line_si);
                    si_der = der;
                    // si_der(1) = 0;
                    si_der(1) /= 3;
                    prod_mul = v_err.dot(si_der);
                    // ROS_INFO_STREAM("prodmul: " << prod_mul);
                }

            } else {
                // ROS_INFO_STREAM("Point Ver");
                min_diff_x = - min_x;
                min_diff_y = - min_y;
                comp = projection_method(min_diff_x, min_diff_y);
                si_der = Eigen::Vector2d(comp(0), comp(1));
                si_der(1) /= 3;
                prod_mul = v_err.dot(si_der);
                // ROS_INFO_STREAM();
            }

            // ROS_INFO_STREAM("Point res" << projection_method(- min_x, - min_y));
            

            // min_dist = sqrt(pow(min_diff_x, 2) + pow(min_diff_y, 2));
            // float si = (r_min / min_dist - r_min / r_norm) / (1. - r_min / r_norm);
            // float base_const = sqrt(pow(min_dist, 3)) * (r_min - r_norm);
            // float up_const = r_min * r_norm;
            // float si_der_x = up_const * min_x / base_const;
            // float si_der_y = up_const * min_y / base_const;

            // float norm_si_der = sqrt(pow(si_der_x, 2) + pow(si_der_y, 2));
            // float norm_si_der_x = si_der_x / norm_si_der;
            // float norm_si_der_y = si_der_y / norm_si_der;
            // float prod_mul = v_lin_x_fb * norm_si_der_x + v_lin_y_fb * norm_si_der_y;


            // ROS_INFO_STREAM("Diff: " << alter_result - prod_mul);

            // ROS_INFO_STREAM("vx: " << v_lin_x_fb << ", vy: " << v_lin_y_fb);

            // ROS_INFO_STREAM("Min_x: " << min_diff_x << ", Min_y: " << min_diff_y);

            // ROS_INFO_STREAM("der x: " << norm_si_der_x << ", der y: " << norm_si_der_y << "norm si der: " << norm_si_der << ", prod" << prod_mul);

            // ROS_INFO_STREAM("Min diff x: " << min_diff_x << ", min diff y: " << min_diff_y);

            // ROS_INFO_STREAM("Si der x: " << si_der_x << ", Si der y: " << si_der_y);

            // ROS_INFO_STREAM("si: " << si << ", prod_mul: " << prod_mul);


            if(comp(2) >= 0 && prod_mul <= 0)
            {
                u_add_x = comp(2) * prod_mul * - comp(0);
                u_add_y = comp(2) * prod_mul * - comp(1);
                // ROS_INFO_STREAM("comp y" << comp(1));
            }

            // ROS_INFO_STREAM("u add x: " << u_add_x << ", y: " << u_add_y << ", min_dist: " << min_dist);
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

            res.scale.x = sqrt(pow(u_add_y, 2) + pow(u_add_x, 2));
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
            projection_viz.publish(res);

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
            // res.color.a = 0.5;
            // res.pose.position.z = 0.9;
            projection_viz.publish(res);
        } else {
            ROS_DEBUG_STREAM_THROTTLE(10, "Projection operator off");
        }

        // ROS_INFO_STREAM(rbt_in_cam_lc);

        // Issue induced by non-holonomicity
        // The vector is correct, but the trajectory is not

        // Make sure no ejection
        u_add_x = std::min(u_add_x, float(0));

        // ROS_INFO_STREAM("Mid vlinx: " << v_lin_x_fb);

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
            // ROS_INFO_STREAM("Non holonomic");
            v_ang_fb = v_ang_fb + v_lin_y_fb + k_po_turn_ * u_add_y + v_ang_const;
            // v_lin_x_fb = abs(theta_error) > M_PI / 3 ? 0 : v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;
            v_lin_x_fb = v_lin_x_fb + v_lin_x_const + k_po_ * u_add_x;

            if (projection_operator && min_dist_ang > - M_PI / 4 && min_dist_ang < M_PI / 4 && min_dist < cfg_->rbt.r_inscr)
            {
                // ROS_INFO_STREAM("No x linear");
                v_lin_x_fb = 0;
                v_ang_fb *= 2;
            }

            v_lin_y_fb = 0;

            // ROS_INFO_STREAM("fin vx: " << v_lin_x_fb << ", vang: " << v_ang_fb);
            if(v_lin_x_fb < 0)
                v_lin_x_fb = 0;
        }

        cmd_vel.linear.x = std::max(-cfg_->control.vx_absmax, std::min(cfg_->control.vx_absmax, v_lin_x_fb));
        cmd_vel.linear.y = std::max(-cfg_->control.vy_absmax, std::min(cfg_->control.vy_absmax, v_lin_y_fb));
        cmd_vel.angular.z = std::max(-cfg_->control.ang_absmax, std::min(cfg_->control.ang_absmax, v_ang_fb));
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
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
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
        //g.real()(0,0) = phase.real();
        g.real()(0, 1) = x;
        g.real()(1, 0) = 0;
        g.real()(1, 1) = 1;

        //g.imag()(0,0) = phase.imag();
        g.imag()(0, 1) = y;
        g.imag()(1, 0) = 0;
        g.imag()(1, 1) = 0;

        g(0, 0) = phase;

        return g;
    }


    int TrajectoryController::targetPoseIdx(geometry_msgs::Pose curr_pose, potential_gap::TrajPlan ref_pose) {
        // Find pose right ahead
        std::vector<double> pose_diff(ref_pose.poses.size());
        // ROS_INFO_STREAM("Ref_pose length: " << ref_pose.poses.size());
        for (int i = 0; i < pose_diff.size(); i++) // i will always be positive, so this is fine
        {
            pose_diff[i] = sqrt(pow(curr_pose.position.x - ref_pose.poses[i].position.x, 2) + 
                                pow(curr_pose.position.y - ref_pose.poses[i].position.y, 2)) + 
                                0.5 * (1 - (   curr_pose.orientation.x * ref_pose.poses[i].orientation.x + 
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

    potential_gap::TrajPlan TrajectoryController::niGen(double lambda, geometry_msgs::PoseStamped curr_pose_odom, geometry_msgs::PoseArray pose_arr_odom)
    {
        potential_gap::TrajPlan ni_gen;
        // geometry_msgs::PoseArray ni_gen;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(curr_pose_odom.pose.orientation, quat);
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        turtlebot_trajectory_generator::ni_state x0;
        x0[0] = curr_pose_odom.pose.position.x; //x
        x0[1] = curr_pose_odom.pose.position.y; //y
        x0[2] = yaw;                            //theta
        x0[3] = -0.1;                           //v
        x0[4] = 0.0;                            //w
        x0[5] = lambda;                         //lambda: must be > 0!
        x0[6] = curr_pose_odom.pose.position.x; //x_d
        x0[7] = curr_pose_odom.pose.position.y; //y_d
                                                //]
        ni_gen.header.frame_id = cfg_->odom_frame_id;

        turtlebot_trajectory_generator::TrajectoryGenerator trajectory_gen;
        turtlebot_trajectory_generator::desired_traj_func::Ptr traj = std::make_shared<potential_gap::gap_traj_fun>(pose_arr_odom);
        turtlebot_trajectory_generator::near_identity ni(1, 5, 1, .01); //, 1.0, 1.0, 1.0, 1.0);

        turtlebot_trajectory_generator::ni_controller nc(ni);
        nc.setTrajFunc(traj);

        std::vector<turtlebot_trajectory_generator::ni_state> x_vec;
        std::vector<double> times;
        size_t steps;
        trajectory_generator::traj_params params = trajectory_gen.getDefaultParams();
        params.tf = 10;
        params.dt = 0.2;
        
        steps = trajectory_gen.run(nc, x0, x_vec, times, params);
        
        for (turtlebot_trajectory_generator::ni_state state : x_vec)
        {
            geometry_msgs::Pose ni_pose;
            geometry_msgs::Twist ni_twist;
            ni_pose.position.x = state[0];
            ni_pose.position.y = state[1];
            ni_twist.linear.x =  state[3];
            ni_twist.angular.z = state[4];
            tf2::Quaternion myQuaternion;
            myQuaternion.setRPY(0, 0, state[2]);
            myQuaternion.normalize();
            geometry_msgs::Quaternion quat_msg = tf2::toMsg(myQuaternion);
            ni_pose.orientation = quat_msg;
            ni_gen.poses.push_back(ni_pose);
            ni_gen.twist.push_back(ni_twist);
        }
        return ni_gen;
    }


    Eigen::Vector2d TrajectoryController::car2pol(Eigen::Vector2d a) {
        return Eigen::Vector2d(a.norm(), float(std::atan2(a(1), a(0))));
    }

    Eigen::Vector2d TrajectoryController::pol2car(Eigen::Vector2d a) {
        return Eigen::Vector2d(cos(a(1)) * a(0), sin(a(1)) * a(0));
    }


}
