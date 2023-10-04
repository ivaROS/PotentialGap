#ifndef GAP_MOD_H
#define GAP_MOD_H


#include <ros/ros.h>
#include <math.h>
#include <potential_gap/gap.h>
#include <potential_gap/potentialgap_config.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sensor_msgs/LaserScan.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/robot_geo_parser.h>
#include <potential_gap/utils.h>

namespace potential_gap {
    class GapManipulator {
        public: 
            GapManipulator(){};
            ~GapManipulator(){};

            GapManipulator(ros::NodeHandle& nh, const potential_gap::PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc) 
            {
                cfg_ = &cfg;
                robot_geo_proc_ = robot_geo_proc;
            };
            GapManipulator& operator=(GapManipulator & other) 
            {
                cfg_ = other.cfg_;
                robot_geo_proc_ = other.robot_geo_proc_;
                return *this;
            };
            GapManipulator(const GapManipulator &t) 
            {
                cfg_ = t.cfg_;
                robot_geo_proc_ = t.robot_geo_proc_;
            };

            void updateEgoCircle(boost::shared_ptr<sensor_msgs::LaserScan const>);

            void setGapWaypoint(potential_gap::Gap&, geometry_msgs::PoseStamped);
            void reduceGap(potential_gap::Gap&, geometry_msgs::PoseStamped);
            void convertAxialGap(potential_gap::Gap&);
            void radialExtendGap(potential_gap::Gap&);
            void inflateGapConserv(potential_gap::Gap&);
            void inflateGapLarge(potential_gap::Gap&);
        private:
            boost::shared_ptr<sensor_msgs::LaserScan const> msg;
            const PotentialGapConfig* cfg_;
            int num_of_scan;
            boost::mutex egolock;

            Eigen::Vector2f car2pol(Eigen::Vector2f);
            Eigen::Vector2f pol2car(Eigen::Vector2f);
            Eigen::Vector2f pTheta(float, float, Eigen::Vector2f, Eigen::Vector2f);

            bool checkGoalVisibility(geometry_msgs::PoseStamped);

            RobotGeoProc robot_geo_proc_;

        public:
            // static float findScaleLineCirInt(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float r);
            // static Eigen::Vector2f intersectLines(Eigen::Vector2f p1, Eigen::Vector2f p2, Eigen::Vector2f p3, Eigen::Vector2f p4 );
            // static std::vector<Eigen::Vector2f> intersectLineCir(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2, const Eigen::Vector2f &cp, float r, bool segment);
            // static Eigen::Vector2f circTangPt(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, float r, bool ccw);
            // static Eigen::Vector2f getMidPt(const Eigen::Vector2f& orig, const Eigen::Vector2f& p1, const Eigen::Vector2f& p2);
            // static void arrangeAngCCW(const float ang1, const float ang2, float& angl, float& angu);
            // static bool ptInsideCirc(const Eigen::Vector2f& orig, const Eigen::Vector2f& pt, float r);

            bool checkLineEgoBlocked(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2);
            bool checkLineEgoBlocked(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2, float& nearest_dist);

            Eigen::Vector2f findNearUnblockLine(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, float r, bool ccw);
            Eigen::Vector2f searchNULS1(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, const Eigen::Vector2f& tang_pt, int fixed_it, int max_it);
            Eigen::Vector2f searchNULS2(const Eigen::Vector2f& circ_orig, const Eigen::Vector2f& pt, const Eigen::Vector2f& tang_pt, int max_it, float max_dist);
    };
}

#endif
