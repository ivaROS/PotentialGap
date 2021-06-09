#ifndef GAP_FINDER_H
#define GAP_FINDER_H

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <potential_gap/gap.h>
#include <boost/shared_ptr.hpp>
#include <potential_gap/potentialgap_config.h>

namespace potential_gap {
    class GapUtils 
    {
        public: 
        GapUtils();

        ~GapUtils();

        GapUtils(const PotentialGapConfig& cfg);

        GapUtils& operator=(GapUtils other) {cfg_ = other.cfg_;};

        GapUtils(const GapUtils &t) {cfg_ = t.cfg_;};

        /**
         * @brief Detection of Gaps
         * @param Laserscan
         * @param gaps to return
         */
        void hybridScanGap(boost::shared_ptr<sensor_msgs::LaserScan const>, std::vector<potential_gap::Gap>&);

        /**
         * @brief Simplification of gaps
         * @param Laserscan
         * @param gaps to simplify and return
         */
        void mergeGapsOneGo(boost::shared_ptr<sensor_msgs::LaserScan const>,
        std::vector<potential_gap::Gap>&);

        /**
         * @brief Setters for threshold
         */
        void setMergeThreshold(float);
        void setIdxThreshold(int);


        private:
            const PotentialGapConfig* cfg_;

    };


}


#endif