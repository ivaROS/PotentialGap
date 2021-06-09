#include <potential_gap/gap_trajectory_generator.h>

namespace potential_gap{
    geometry_msgs::PoseArray GapTrajGenerator::generateTrajectory(potential_gap::Gap selectedGap, geometry_msgs::PoseStamped curr_pose) {
        geometry_msgs::PoseArray posearr;
        posearr.header.stamp = ros::Time::now();
        
        double coefs = cfg_->traj.scale;
        write_trajectory corder(posearr, cfg_->robot_frame_id, coefs);
        // If sensor frame == rbt frame
        posearr.header.frame_id = cfg_->traj.synthesized_frame ? cfg_->sensor_frame_id : cfg_->robot_frame_id;

        if (selectedGap.goal.discard || selectedGap.get_dist_side() < 1.5 *  cfg_->rbt.r_inscr) {
            return posearr;
        }

        state_type x = {curr_pose.pose.position.x + 1e-5, curr_pose.pose.position.y + 1e-6}; 

        if (selectedGap.goal.goalwithin) {
            // Goal to Goal
            g2g inte_g2g(
                selectedGap.goal.x * coefs,
                selectedGap.goal.y * coefs);
            boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            inte_g2g, x, 0.0,
            cfg_->traj.integrate_maxt,
            cfg_->traj.integrate_stept,
            corder);
            return posearr;
        }

        float x1, x2, y1, y2;
        float half_num_scan = selectedGap.half_scan;
        x1 = (selectedGap.convex.convex_ldist) * cos(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        y1 = (selectedGap.convex.convex_ldist) * sin(-((float) half_num_scan - selectedGap.convex.convex_lidx) / half_num_scan * M_PI);
        x2 = (selectedGap.convex.convex_rdist) * cos(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);
        y2 = (selectedGap.convex.convex_rdist) * sin(-((float) half_num_scan - selectedGap.convex.convex_ridx) / half_num_scan * M_PI);

        // RE gaps need to be shifted
        if (selectedGap.mode.convex) {
            x = {- selectedGap.qB(0) - 1e-6, - selectedGap.qB(1) + 1e-6};
            x1 -= selectedGap.qB(0);
            x2 -= selectedGap.qB(0);
            y1 -= selectedGap.qB(1);
            y2 -= selectedGap.qB(1);
            selectedGap.goal.x -= selectedGap.qB(0);
            selectedGap.goal.y -= selectedGap.qB(1);
        }
        
        polar_gap_field inte(x1 * coefs, x2 * coefs,
                            y1 * coefs, y2 * coefs,
                            selectedGap.goal.x * coefs,
                            selectedGap.goal.y * coefs,
                            selectedGap.isRadial(),
                            cfg_->gap_manip.sigma);
        boost::numeric::odeint::integrate_const(boost::numeric::odeint::euler<state_type>(),
            inte, x, 0.0,
            cfg_->traj.integrate_maxt,
            cfg_->traj.integrate_stept, corder);

        if (selectedGap.mode.convex) {
            // RE Gaps need to be brought back
            for (auto & p : posearr.poses) {
                p.position.x += selectedGap.qB(0);
                p.position.y += selectedGap.qB(1);
            }
        }

        return posearr;
    }

    std::vector<geometry_msgs::PoseArray> GapTrajGenerator::generateTrajectory(std::vector<potential_gap::Gap> gapset) {
        std::vector<geometry_msgs::PoseArray> traj_set(gapset.size());
        return traj_set;
    }

    // Return in Odom frame (used for ctrl)
    geometry_msgs::PoseArray GapTrajGenerator::transformBackTrajectory(
        geometry_msgs::PoseArray posearr,
        geometry_msgs::TransformStamped planning2odom)
    {
        geometry_msgs::PoseArray retarr;
        geometry_msgs::PoseStamped outplaceholder;
        outplaceholder.header.frame_id = cfg_->odom_frame_id;
        geometry_msgs::PoseStamped inplaceholder;
        inplaceholder.header.frame_id = cfg_->robot_frame_id;
        for (const auto pose : posearr.poses)
        {
            inplaceholder.pose = pose;
            tf2::doTransform(inplaceholder, outplaceholder, planning2odom);
            retarr.poses.push_back(outplaceholder.pose);
        }
        retarr.header.frame_id = cfg_->odom_frame_id;
        retarr.header.stamp = ros::Time::now();
        return retarr;
    }

    geometry_msgs::PoseArray GapTrajGenerator::forwardPassTrajectory(geometry_msgs::PoseArray pose_arr)
    {
        Eigen::Quaternionf q;
        geometry_msgs::Pose old_pose;
        old_pose.position.x = 0;
        old_pose.position.y = 0;
        old_pose.position.z = 0;
        old_pose.orientation.x = 0;
        old_pose.orientation.y = 0;
        old_pose.orientation.z = 0;
        old_pose.orientation.w = 1;
        geometry_msgs::Pose new_pose;
        double dx, dy, result;

        std::vector<geometry_msgs::Pose> shortened;
        shortened.push_back(old_pose);
        for (auto pose : pose_arr.poses)
        {
            dx = pose.position.x - shortened.back().position.x;
            dy = pose.position.y - shortened.back().position.y;
            result = sqrt(pow(dx, 2) + pow(dy, 2));
            if (result > 0.05)
                shortened.push_back(pose);
        }

        pose_arr.poses = shortened;

        // Fix rotation inconsistency
        for (int idx = 1; idx < pose_arr.poses.size(); idx++)
        {
            new_pose = pose_arr.poses[idx];
            old_pose = pose_arr.poses[idx - 1];
            dx = new_pose.position.x - old_pose.position.x;
            dy = new_pose.position.y - old_pose.position.y;
            result = std::atan2(dy, dx);
            q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(result, Eigen::Vector3f::UnitZ());
            q.normalize();
            pose_arr.poses[idx - 1].orientation.x = q.x();
            pose_arr.poses[idx - 1].orientation.y = q.y();
            pose_arr.poses[idx - 1].orientation.z = q.z();
            pose_arr.poses[idx - 1].orientation.w = q.w();
        }
        pose_arr.poses.pop_back();

        return pose_arr;
    }

}