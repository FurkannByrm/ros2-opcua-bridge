#include "opcua_to_ros2/config.hpp"
#include "yaml-cpp/yaml.h"
#include <opcua_to_ros2/naming.hpp>


UaConfig ConfigLoader::load_file(const std::string& yaml_path){

    UaConfig cfg;
    YAML::Node root = YAML::LoadFile(yaml_path);

    cfg.endpoint = root["endpoint"].as<std::string>();
    cfg.ns_index = root["namespace_index"].as<int>();
    cfg.timing.rc.initial_ms = root["timing"]["reconnect"]["initial_ms"].as<int>();
    cfg.timing.rc.max_ms     = root["timing"]["reconnect"]["max_ms"].as<int>();
    cfg.timing.rc.multiplier = root["timing"]["reconnect"]["multiplier"].as<double>();
    cfg.timing.write_timeout_ms = root["timing"]["write_timeout_ms"].as<int>();
    cfg.timing.sampling_ms      = root["timing"]["sampling_ms"].as<int>();
    
    cfg.structs.stat_root = root["structs"]["stat_root"].as<std::string>();
    cfg.structs.mod_root = root["structs"]["mod_root"].as<std::string>();
    cfg.structs.sensing_root = root["structs"]["sensing_root"].as<std::string>();
    cfg.structs.cleaning_root = root["structs"]["cleaning_root"].as<std::string>();
    cfg.structs.workcell_status = root["structs"]["Workcell"].as<std::string>();
    cfg.structs.spot_M1_root = root["structs"]["spot_M1_root"].as<std::string>();
    cfg.structs.spot_M2_root = root["structs"]["spot_M2_root"].as<std::string>();

if (root["structs"]["spatter_classes"]) {
    for (const auto& spot : root["structs"]["spatter_classes"]) {
        std::string current_spot_path = make_weld_node(cfg.structs.spot_M1_root, spot.as<std::string>());
        cfg.structs.spatter1_vec.push_back(current_spot_path);  

        if (root["structs"]["spatter_size"]) {
            for (const auto& spot_size : root["structs"]["spatter_size"]) {
                cfg.structs.spatter_size.push_back(
                    make_weld_node(current_spot_path, spot_size.as<std::string>())
                );
            }
        }
    } 
}


    return cfg;

}

