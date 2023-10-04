#include <potential_gap/robot_geo_parser.h>

namespace potential_gap{
    RobotGeoStorage::RobotGeoStorage(std::string config_file)
    {
        loadConfigFile(config_file);
        initialized_ = true;
    }

    void RobotGeoStorage::loadConfigFile(std::string config_file)
    {
        YAML::Node config = YAML::LoadFile(config_file);

        for(auto v: config["vec_dot_product"])
        {
            vec_dot_product_.push_back(v.as<double>());
        }
        for(auto er: config["equivalent_radius"])
        {
            equivalent_radius_.push_back(er.as<double>());
        }
        for(auto epl: config["equivalent_pass_len"])
        {
            equivalent_pass_len_.push_back(epl.as<double>());
        }

        // std::cout << vec_dot_product_.size() << " " << equivalent_radius_.size() << " " << equivalent_pass_len_.size() << std::endl;
    }

    double RobotGeoStorage::getInterp(double vec_dot_product, std::vector<double>& target_vec)
    {
        auto it = std::lower_bound (vec_dot_product_.begin(), vec_dot_product_.end(), vec_dot_product);
        if(it == vec_dot_product_.begin())
        {
            double interp_e = target_vec[0];
            return interp_e;
        }
        else
        {
            double low_val = *std::prev(it);
            int low_idx = std::prev(it) - vec_dot_product_.begin();
            double high_val = *it;
            int high_idx = low_idx + 1;
            double e_low = target_vec[low_idx];
            double e_high = target_vec[high_idx];
            double interp_e = (vec_dot_product - low_val) / (high_val - low_val) * (e_high - e_low) + e_low;
            return interp_e;
        }
    }

    double RobotGeoStorage::getInterpEquivR(double vec_dot_product)
    {
        return getInterp(vec_dot_product, equivalent_radius_);
    }

    double RobotGeoStorage::getInterpEquivPL(double vec_dot_product)
    {
        return getInterp(vec_dot_product, equivalent_pass_len_);
    }
}