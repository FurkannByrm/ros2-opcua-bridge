#include "demonstrator_tree/parameters_parser.hpp"

CobotConfig ConfigLoader::load_file(const std::string& yaml_path){

    CobotConfig cfg;

    YAML::Node root = YAML::LoadFile(yaml_path);
    cfg.sensing_group.name = root["cobot1"]["robot_name"].as<std::string>();
    cfg.sensing_group.sensing_joint_states = root["cobot1"]["sensing_joint_states"].as<std::string>(); 
    cfg.sensing_group.sensing_service_name = root["cobot1"]["sensing_service"].as<std::string>();
    for(const auto& position : root["cobot1"]["home_position"]){
        cfg.sensing_group.sensing_home_vec.emplace_back(position.as<double>());
    }

    
    cfg.cleaning_group.name = root["cobot2"]["robot_name"].as<std::string>();
    cfg.cleaning_group.cleaning_joint_states = root["cobot2"]["cleaning_joint_states"].as<std::string>();
    cfg.cleaning_group.cleaning_service_name = root["cobot2"]["cleaning_service"].as<std::string>();
    for(const auto& position : root["cobot2"]["home_position"]){
        cfg.cleaning_group.cleaning_home_vec.emplace_back(position.as<double>());
    }

    return cfg;
}