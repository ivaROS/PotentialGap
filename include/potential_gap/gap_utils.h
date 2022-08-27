#ifndef GAP_FINDER_H
#define GAP_FINDER_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <potential_gap/gap.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/potentialgap_config.h>
#include <potential_gap/robot_geo_parser.h>

namespace potential_gap {
    class GapUtils 
    {
        public: 
        GapUtils();

        ~GapUtils();

        GapUtils(const PotentialGapConfig& cfg, RobotGeoProc& robot_geo_proc);

        GapUtils& operator=(GapUtils other) 
        {
            cfg_ = other.cfg_;
            robot_geo_proc_ = other.robot_geo_proc_;
        };

        GapUtils(const GapUtils &t) 
        {
            cfg_ = t.cfg_;
            robot_geo_proc_ = t.robot_geo_proc_;
        };

        void hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const>, std::vector<potential_gap::Gap>&);

        void mergeGapsOneGo(boost::shared_ptr<sensor_msgs::LaserScan const>,
        std::vector<potential_gap::Gap>&);

        void setMergeThreshold(float);
        void setIdxThreshold(int);


        private:
            const PotentialGapConfig* cfg_;
        public:
            RobotGeoProc robot_geo_proc_;

    };


}


#endif