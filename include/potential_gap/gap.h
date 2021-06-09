#ifndef GAP_H
#define GAP_H

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/Point.h>
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

            Gap(std::string frame, int left_idx, float ldist, bool axial = false, float half_scan = 256) : _frame(frame), _left_idx(left_idx), _ldist(ldist), _radial(axial), half_scan(half_scan)
            {};

            ~Gap() {};

            // Getters
            int LIdx(){return _left_idx;}

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

            // Concluding the Gap after constructing with left information
            void addRightInformation(int right_idx, float rdist) 
            {
                _right_idx = right_idx;
                _rdist = rdist;
                left_type = _ldist < _rdist;

                if (!_radial)
                {
                    float resoln = M_PI / half_scan;
                    float angle1 = (_right_idx - _left_idx) * resoln;
                    float short_side = left_type ? _ldist : _rdist;
                    float opp_side = (float) sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (float)cos(angle1));
                    float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                    if (M_PI - small_angle - angle1 > 0.75 * M_PI) 
                    {
                        _radial = true;
                    }
                }

                convex.convex_lidx = _left_idx;
                convex.convex_ridx = _right_idx;
                convex.convex_ldist = _ldist;
                convex.convex_rdist = _rdist;
            }

            bool isRadial()
            {
                float resoln = M_PI / half_scan;
                float angle1 = (_right_idx - _left_idx) * resoln;
                float short_side = left_type ? _ldist : _rdist;
                float opp_side = (float) sqrt(pow(_ldist, 2) + pow(_rdist, 2) - 2 * _ldist * _rdist * (float)cos(angle1));
                float small_angle = (float) asin(short_side / opp_side * (float) sin(angle1));
                _radial = (M_PI - small_angle - angle1 > 0.75 * M_PI); 
                return _radial;
            }

            void setSwept()
            {
                _radial = false;
            }

            bool isLeftType()
            {
                return left_type;
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

            float life_time = 0.1;
            bool agc = false;

            int _left_idx = 0;
            float _ldist = 3;
            int _right_idx = 511;
            float _rdist = 3;

            float min_safe_dist = -1;
            Eigen::Vector2f qB;
            float half_scan = 256;

            std::string _frame = "";
            bool _radial = false;
            bool left_type = false;

            struct converted {
                int convex_lidx = 0;
                int convex_ridx = 511;
                float convex_ldist = 3;
                float convex_rdist = 3;
            } convex;

            struct GapMode {
                bool reduced = false;
                bool convex = false;
                bool rgc = false;
                bool wrap = false;
            } mode;

            struct Goal {
                float x, y;
                bool set = false;
                bool discard = false;
                bool goalwithin = false;
            } goal;
    };
}

#endif