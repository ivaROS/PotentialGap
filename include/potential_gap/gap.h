#ifndef GAP_H
#define GAP_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace potential_gap
{
    class Gap
    {
        public:
            Gap() {};

            Gap(std::string frame, int left_idx, float ldist, bool axial = false, float half_scan = 256) : _frame(frame), _left_idx(left_idx), _ldist(ldist), _axial(axial), half_scan(half_scan)
            {};

            ~Gap() {};

            void setLIdx(int lidx)
            {
                _left_idx = lidx;
            }

            void setRIdx(int ridx)
            {
                _right_idx = ridx;
            }

            // Setter and Getter for LR Distance and Index
            void setLDist(float ldist) 
            {
                _ldist = ldist;
            }

            void setRDist(float rdist)
            {
                _rdist = rdist;
            }

            void setAll(float ldist, float rdist, int lidx, int ridx)
            {
                _left_idx = lidx;
                _right_idx = ridx;
                _ldist = ldist;
                _rdist = rdist;
            }

            void setConvexAll(float ldist, float rdist, int lidx, int ridx)
            {
                convex.convex_lidx = lidx;
                convex.convex_ridx = ridx;
                convex.convex_ldist = ldist;
                convex.convex_rdist = rdist;
            }

            void setConvexSame()
            {
                setConvexAll(_ldist, _rdist, _left_idx, _right_idx);
            }

            int LIdx()
            {
                return _left_idx;
            }

            int RIdx()
            {
                return _right_idx;
            }

            float LDist()
            {
                return _ldist;
            }

            float RDist()
            {
                return _rdist;
            }

            void getLRIdx(int &l, int &r)
            {
                l = _left_idx;
                r = _right_idx;
            }

            void transform(geometry_msgs::TransformStamped trans, double trans_min_dist)
            {
                float resoln = M_PI / half_scan;

                geometry_msgs::PointStamped l_orig_pt, l_trans_pt, r_orig_pt, r_trans_pt;
                float lx, ly, rx, ry;

                ros::Time cur_time = ros::Time::now();
                l_orig_pt.header.frame_id = _frame;
                l_orig_pt.header.stamp = cur_time;
                getLCartesian(lx, ly);
                l_orig_pt.point.x = lx;
                l_orig_pt.point.y = ly;
                tf2::doTransform(l_orig_pt, l_trans_pt, trans);
                float l_trans_range = sqrt(pow(l_trans_pt.point.x, 2) + pow(l_trans_pt.point.y, 2));
                float l_trans_ang = std::atan2(l_trans_pt.point.y, l_trans_pt.point.x);

                r_orig_pt.header.frame_id = _frame;
                r_orig_pt.header.stamp = cur_time;
                getRCartesian(rx, ry);
                r_orig_pt.point.x = rx;
                r_orig_pt.point.y = ry;
                tf2::doTransform(r_orig_pt, r_trans_pt, trans);
                float r_trans_range = sqrt(pow(r_trans_pt.point.x, 2) + pow(r_trans_pt.point.y, 2));
                float r_trans_ang = std::atan2(r_trans_pt.point.y, r_trans_pt.point.x);

                // ROS_INFO_STREAM("Orig: Lx: " << lx << " ly: " << ly << " rx: " << rx << " ry: " << ry);
                // ROS_INFO_STREAM("Trans: Lx: " << l_trans_pt.point.x << " ly: " << l_trans_pt.point.y << " rx: " << r_trans_pt.point.x << " ry: " << r_trans_pt.point.y);
                // ROS_INFO_STREAM("L ang: " << l_trans_ang << " L range: " << l_trans_range << " R ang: " << r_trans_ang << " R range: " << r_trans_range);

                if(l_trans_ang > r_trans_ang && l_trans_ang >= 0 && r_trans_ang < 0)
                {
                    if(abs(M_PI - l_trans_ang) < abs(r_trans_ang + M_PI))
                        l_trans_ang = -M_PI;
                    else
                        r_trans_ang = M_PI;
                }
                else if(l_trans_ang > r_trans_ang && l_trans_ang >= 0 && r_trans_ang >= 0)
                {
                    l_trans_ang = -M_PI;
                }
                else if(l_trans_ang > r_trans_ang && l_trans_ang < 0 && r_trans_ang < 0)
                {
                    r_trans_ang = M_PI;
                }

                // ROS_INFO_STREAM("Check L ang: " << l_trans_ang << " L range: " << l_trans_range << " R ang: " << r_trans_ang << " R range: " << r_trans_range);

                int l_idx = int(round((l_trans_ang + M_PI) / resoln));
                l_idx = l_idx < 0 ? 0 : l_idx;
                l_idx = l_idx > (half_scan * 2 - 1) ? (half_scan * 2 - 1) : l_idx;
                int r_idx = int(round((r_trans_ang + M_PI) / resoln));
                r_idx = r_idx < 0 ? 0 : r_idx;
                r_idx = r_idx > (half_scan * 2 - 1) ? (half_scan * 2 - 1) : r_idx;

                if(l_idx == r_idx)
                {
                    if(l_idx == (half_scan * 2 - 1))
                        l_idx--;
                    else
                        r_idx++;
                }

                // ROS_INFO_STREAM("L idx: " << l_idx << " R idx: " << r_idx);

                setAll(l_trans_range, r_trans_range, l_idx, r_idx);
                setConvexSame();
                setMinSafeDist(trans_min_dist);
                _frame = trans.header.frame_id;
                left_type = _ldist < _rdist;
            }

            // Concluding the Gap after constructing with left information
            void addRightInformation(int right_idx, float rdist) 
            {
                _right_idx = right_idx;
                _rdist = rdist;
                left_type = _ldist < _rdist;

                if (!_axial)
                {
                    float resoln = M_PI / half_scan;
                    float angle1 = (_right_idx - _left_idx) * resoln;
                    float short_side = left_type ? _ldist : _rdist;
                    float opp_side = (float) sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (float)cos(angle1));
                    float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                    if (M_PI - small_angle - angle1 > 0.75 * M_PI) 
                    {
                        _axial = true;
                    }
                }

                convex.convex_lidx = _left_idx;
                convex.convex_ridx = _right_idx;
                convex.convex_ldist = _ldist;
                convex.convex_rdist = _rdist;
            }

            // Get Left Cartesian Distance
            void getLCartesian(float &x, float &y)
            {
                x = (_ldist) * cos(-((float) half_scan - _left_idx) / half_scan * M_PI);
                y = (_ldist) * sin(-((float) half_scan - _left_idx) / half_scan * M_PI);
            }

            // Get Right Cartesian Distance
            void getRCartesian(float &x, float &y)
            {
                x = (_rdist) * cos(-((float) half_scan - _right_idx) / half_scan * M_PI);
                y = (_rdist) * sin(-((float) half_scan - _right_idx) / half_scan * M_PI);
            }

            void getRadialExLCartesian(float &x, float &y){
                x = (convex.convex_ldist) * cos(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
                y = (convex.convex_ldist) * sin(-((float) half_scan - convex.convex_lidx) / half_scan * M_PI);
            }

            void getRadialExRCartesian(float &x, float &y){
                x = (convex.convex_rdist) * cos(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
                y = (convex.convex_rdist) * sin(-((float) half_scan - convex.convex_ridx) / half_scan * M_PI);
            }

            void setAGCIdx(int lidx, int ridx) {
                agc_lidx = lidx;
                agc_ridx = ridx;
                agc_ldist = float(lidx - _left_idx) / float(_right_idx - _left_idx) * (_rdist - _ldist) + _ldist;
                agc_rdist = float(ridx - _left_idx) / float(_right_idx - _left_idx) * (_rdist - _ldist) + _ldist;
            }

            void getAGCLCartesian(float &x, float &y){
                x = (agc_ldist) * cos(-((float) half_scan - agc_lidx) / half_scan * M_PI);
                y = (agc_ldist) * sin(-((float) half_scan - agc_lidx) / half_scan * M_PI);
            }

            void getAGCRCartesian(float &x, float &y){
                x = (agc_rdist) * cos(-((float) half_scan - agc_ridx) / half_scan * M_PI);
                y = (agc_rdist) * sin(-((float) half_scan - agc_ridx) / half_scan * M_PI);
            }

            // Decimate Gap 
            void segmentGap2Vec(std::vector<potential_gap::Gap>& gap, int min_resoln)
            {
                int num_gaps = (_right_idx - _left_idx) / min_resoln + 1;
                int idx_step = (_right_idx - _left_idx) / num_gaps;
                float dist_step = (_rdist - _ldist) / num_gaps;
                int sub_gap_lidx = _left_idx;
                float sub_gap_ldist = _ldist;
                int sub_gap_ridx = _left_idx;

                if (num_gaps < 3) {
                    gap.push_back(*this);
                    return;
                }
                
                for (int i = 0; i < num_gaps; i++) {
                    Gap detected_gap(_frame, sub_gap_lidx, sub_gap_ldist);
                    // ROS_DEBUG_STREAM("lidx: " << sub_gap_lidx << "ldist: " << sub_gap_ldist);
                    if (i != 0) {
                        detected_gap.setLeftObs();
                    }

                    if (i != num_gaps - 1) {
                        detected_gap.setRightObs();
                    }

                    sub_gap_lidx += idx_step;
                    sub_gap_ldist += dist_step;
                    // ROS_DEBUG_STREAM("ridx: " << sub_gap_lidx << "rdist: " << sub_gap_ldist);
                    if (i == num_gaps - 1)
                    {
                        detected_gap.addRightInformation(_right_idx, _rdist);
                    } else {
                        detected_gap.addRightInformation(sub_gap_lidx - 1, sub_gap_ldist);
                    }
                    gap.push_back(detected_gap);
                }
            }

            void compareGoalDist(double goal_dist) {
                goal_within = goal_dist < _ldist && goal_dist < _rdist;
            }

            // Getter and Setter for if side is an obstacle
            void setLeftObs() {
                left_obs = false;
            }

            void setRightObs() {
                right_obs = false;
            }

            bool getLeftObs() {
                return left_obs;
            }


            bool getRightObs() {
                return right_obs;
            }

            bool isAxial()
            {
                float resoln = M_PI / half_scan;
                float angle1 = (_right_idx - _left_idx) * resoln;
                float short_side = left_type ? _ldist : _rdist;
                float opp_side = (float) sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                _axial = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                return _axial;
            }

            void setRadial()
            {
                _axial = false;
            }

            bool isLeftType()
            {
                return left_type;
            }

            void resetFrame(std::string frame) {
                _frame = frame;
            }

            void setMinSafeDist(float _dist) {
                min_safe_dist = _dist;
            }

            float getMinSafeDist() {
                return min_safe_dist;
            }

            std::string getFrame() {
                return _frame;
            }

            float get_dist_side() {
                return sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (cos(float(_right_idx - _left_idx) / float(half_scan) * M_PI)));
            }

            Eigen::Vector2d get_middle_pt_vec()
            {
                float lx, ly;
                getLCartesian(lx, ly);
                Eigen::Vector2d l_vec(lx, ly);
                float rx, ry;
                getRCartesian(rx, ry);
                Eigen::Vector2d r_vec(rx, ry);
                Eigen::Vector2d m_vec = (l_vec + r_vec) / 2;
                return m_vec;
            }
            
            bool goal_within = false;
            bool goal_dir_within = false;
            float life_time = 1.0;
            bool agc = false;

            int _left_idx = 0;
            float _ldist = 3;
            int _right_idx = 511;
            float _rdist = 3;
            bool wrap = false;
            bool reduced = false;
            bool convexified = false;
            int convex_lidx;
            int convex_ridx;
            float convex_ldist;
            float convex_rdist;
            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            float half_scan = 256;

            int agc_lidx;
            int agc_ridx;
            float agc_ldist;
            float agc_rdist;
            bool no_agc_coor = false;

            std::string _frame = "";
            bool left_obs = true;
            bool right_obs = true;
            bool _axial = false;
            bool left_type = false;

            struct converted {
                int convex_lidx = 0;
                int convex_ridx = 511;
                float convex_ldist = 3;
                float convex_rdist = 3;
            } convex;

            struct inflatedGap {
                float min_safe_dist = -1;
                Eigen::Vector2f lgap, rgap;
                float inf_min_safe_dist = -1;
                Eigen::Vector2f inf_gap_int, inf_l, inf_r, inf_lgc_int, inf_rgc_int;
            } inf_gap;

            struct GapMode {
                bool reduced = false;
                bool convex = false; // radial extend
                bool agc = false;
                bool inflated = false;
            } mode;

            struct Goal {
                float x, y;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;
        // private:
    };

    class StaticGap
    {
    public:
        ros::Time start_time_;
        Gap current_gap_;
        geometry_msgs::PoseStamped robot_map_pose_;
        geometry_msgs::Point l_, map_l_, r_, map_r_, map_goal_;
        bool assigned_, radial_ext_;

        StaticGap()
        {
            assigned_ = false;
            radial_ext_ = false;
        }

        StaticGap(Gap& current_gap, geometry_msgs::TransformStamped rbt2map)
        {
            assigned_ = true;
            current_gap_ = current_gap;
            start_time_ = rbt2map.header.stamp;

            geometry_msgs::PoseStamped robot_pose;
            float lx, ly, rx, ry;

            if(current_gap.mode.convex)
            {
                radial_ext_ = true;
                robot_pose.pose.position.x = current_gap.qB[0];
                robot_pose.pose.position.y = current_gap.qB[1];
                current_gap.getRadialExLCartesian(lx, ly);
                current_gap.getRadialExRCartesian(rx, ry);
            }
            else
            {
                current_gap.getLCartesian(lx, ly);
                current_gap.getRCartesian(rx, ry);
            }
            robot_pose.header.frame_id = rbt2map.header.frame_id;
            robot_pose.pose.orientation.w = 1;
            tf2::doTransform(robot_pose, robot_map_pose_, rbt2map);
            
            l_.x = lx;
            l_.y = ly;
            
            r_.x = rx;
            r_.y = ry;

            geometry_msgs::PoseStamped l_pose, l_map_pose;
            l_pose.header.frame_id = rbt2map.header.frame_id;
            l_pose.pose.orientation.w = 1;
            l_pose.pose.position = l_;
            tf2::doTransform(l_pose, l_map_pose, rbt2map);
            map_l_ = l_map_pose.pose.position;

            geometry_msgs::PoseStamped r_pose, r_map_pose;
            r_pose.header.frame_id = rbt2map.header.frame_id;
            r_pose.pose.orientation.w = 1;
            r_pose.pose.position = r_;
            tf2::doTransform(r_pose, r_map_pose, rbt2map);
            map_r_ = r_map_pose.pose.position;

            geometry_msgs::PoseStamped goal_pose, goal_map_pose;
            goal_pose.header.frame_id = rbt2map.header.frame_id;
            goal_pose.pose.orientation.w = 1;
            goal_pose.pose.position.x = current_gap.goal.x;
            goal_pose.pose.position.y = current_gap.goal.y;
            tf2::doTransform(goal_pose, goal_map_pose, rbt2map);
            map_goal_ = goal_map_pose.pose.position;
        }

        ~StaticGap()
        {
            assigned_ = false;
            radial_ext_ = false;
        }

        geometry_msgs::PoseStamped getRobotMapPose()
        {
            return robot_map_pose_;
        }

        geometry_msgs::PoseStamped getRobotMapPose() const
        {
            return robot_map_pose_;
        }

        geometry_msgs::Point getLMap()
        {
            return map_l_;
        }

        geometry_msgs::Point getLMap() const
        {
            return map_l_;
        }

        geometry_msgs::Point getRMap()
        {
            return map_r_;
        }

        geometry_msgs::Point getRMap() const
        {
            return map_r_;
        }

        geometry_msgs::Point getGoalMap()
        {
            return map_goal_;
        }

        geometry_msgs::Point getGoalMap() const
        {
            return map_goal_;
        }
    };

    class StaticInfGap
    {
    public:
        ros::Time start_time_;
        Gap current_gap_;
        std::string map_frame_;
        geometry_msgs::PoseStamped robot_map_pose_;
        geometry_msgs::Point map_l_, map_r_, map_l_int_, map_r_int_;
        double min_r_;
        bool assigned_;

        StaticInfGap()
        {
            assigned_ = false;
        }

        StaticInfGap(geometry_msgs::TransformStamped rbt2map)
        {
            assigned_ = false;
            map_frame_ = rbt2map.header.frame_id;
        }

        StaticInfGap(Gap& current_gap, geometry_msgs::TransformStamped rbt2map)
        {
            if(current_gap.mode.inflated)
            {
                current_gap_ = current_gap;
                start_time_ = rbt2map.header.stamp;
                map_frame_ = rbt2map.header.frame_id;

                geometry_msgs::PoseStamped robot_pose;
                robot_pose.header.frame_id = rbt2map.header.frame_id;
                robot_pose.pose.orientation.w = 1;
                tf2::doTransform(robot_pose, robot_map_pose_, rbt2map);

                map_l_ = transPt(current_gap.inf_gap.inf_l, rbt2map);
                map_r_ = transPt(current_gap.inf_gap.inf_r, rbt2map);
                map_l_int_ = transPt(current_gap.inf_gap.inf_lgc_int, rbt2map);
                map_r_int_ = transPt(current_gap.inf_gap.inf_rgc_int, rbt2map);

                min_r_ = (double) current_gap.inf_gap.inf_min_safe_dist;

                assigned_ = true;
            }
        }

        ~StaticInfGap()
        {
            assigned_ = false;
        }

        geometry_msgs::Point transPt(const Eigen::Vector2f& pt, const geometry_msgs::TransformStamped rbt2map)
        {
            geometry_msgs::PoseStamped pose, map_pose;
            pose.header.frame_id = rbt2map.header.frame_id;
            pose.pose.orientation.w = 1;
            geometry_msgs::Point p;
            p.x = (double) pt(0);
            p.y = (double) pt(1);
            p.z = 0;
            pose.pose.position = p;
            tf2::doTransform(pose, map_pose, rbt2map);
            geometry_msgs::Point map_pt = map_pose.pose.position;

            return map_pt;
        }

        std::string getMapFrame()
        {
            return map_frame_;
        }

        std::string getMapFrame() const
        {
            return map_frame_;
        }

        double getMinDist()
        {
            return min_r_;
        }

        double getMinDist() const
        {
            return min_r_;
        }

        geometry_msgs::PoseStamped getOrigMapPose()
        {
            return robot_map_pose_;
        }

        geometry_msgs::PoseStamped getOrigMapPose() const
        {
            return robot_map_pose_;
        }

        Eigen::Vector2d getOrigMapPoseVec()
        {
            return Eigen::Vector2d(robot_map_pose_.pose.position.x, robot_map_pose_.pose.position.y);
        }

        Eigen::Vector2d getOrigMapPoseVec() const
        {
            return Eigen::Vector2d(robot_map_pose_.pose.position.x, robot_map_pose_.pose.position.y);
        }

        geometry_msgs::Point getLMap()
        {
            return map_l_;
        }

        geometry_msgs::Point getLMap() const
        {
            return map_l_;
        }

        Eigen::Vector2d getLMapVec()
        {
            return Eigen::Vector2d(map_l_.x, map_l_.y);
        }

        Eigen::Vector2d getLMapVec() const
        {
            return Eigen::Vector2d(map_l_.x, map_l_.y);
        }

        geometry_msgs::Point getRMap()
        {
            return map_r_;
        }

        geometry_msgs::Point getRMap() const
        {
            return map_r_;
        }

        Eigen::Vector2d getRMapVec()
        {
            return Eigen::Vector2d(map_r_.x, map_r_.y);
        }

        Eigen::Vector2d getRMapVec() const
        {
            return Eigen::Vector2d(map_r_.x, map_r_.y);
        }

        geometry_msgs::Point getLIntMap()
        {
            return map_l_int_;
        }

        geometry_msgs::Point getLIntMap() const
        {
            return map_l_int_;
        }

        Eigen::Vector2d getLIntMapVec()
        {
            return Eigen::Vector2d(map_l_int_.x, map_l_int_.y);
        }

        Eigen::Vector2d getLIntMapVec() const
        {
            return Eigen::Vector2d(map_l_int_.x, map_l_int_.y);
        }

        geometry_msgs::Point getRIntMap()
        {
            return map_r_int_;
        }

        geometry_msgs::Point getRIntMap() const
        {
            return map_r_int_;
        }

        Eigen::Vector2d getRIntMapVec()
        {
            return Eigen::Vector2d(map_r_int_.x, map_r_int_.y);
        }

        Eigen::Vector2d getRIntMapVec() const
        {
            return Eigen::Vector2d(map_r_int_.x, map_r_int_.y);
        }
    };
}

#endif
