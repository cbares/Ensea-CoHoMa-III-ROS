#ifndef MODELIDAR_WHEEL_HPP
#define MODELIDAR_WHEEL_HPP


#include <cmath>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace modelidar {

class Wheel {
public:
    
    std::string name;
    int enc;
    double cmd;
    double pos;
    double vel;
    int rad_per_count;

    std::string velocity_name;
    std::string position_name;
    
    Wheel() = default;

    void setup(std::string name, int enc_counts_per_rev) {
        this->name = name;
        this->rad_per_count = 2 * M_PI / enc_counts_per_rev; 
        this->velocity_name = name + "/" + hardware_interface::HW_IF_VELOCITY;
        this->position_name = name + "/" + hardware_interface::HW_IF_POSITION;
    };

    double calc_enc_angle() {
        return enc * rad_per_count;
    };

};


} // namespace modelidar

#endif // MODELIDAR_WHEEL_HPP