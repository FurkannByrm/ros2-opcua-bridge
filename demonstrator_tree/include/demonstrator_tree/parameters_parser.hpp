#include "yaml-cpp/yaml.h"
#include <iostream>
#include <string>
#include <vector>



struct CleaningCobot{

    std::string name;
    std::string cleaning_joint_states;
    std::string cleaning_service_name;
    std::vector<double> cleaning_home_vec;
};

struct SensingCobot{
    
    std::string name;
    std::string sensing_joint_states;
    std::string sensing_service_name;
    std::vector<double> sensing_home_vec;
    
};


struct CobotConfig
{

    CleaningCobot cleaning_group;
    SensingCobot sensing_group;

};


class ConfigLoader{

    public:
    CobotConfig static load_file(const std::string& yaml_path);


};

