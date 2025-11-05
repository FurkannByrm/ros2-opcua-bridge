#include "backend/config.hpp"
#include "yaml-cpp/yaml.h"


UaConfig ConfigLoader::load_file(const std::string& yaml_path){

    UaConfig cfg;
    YAML::Node root = YAML::LoadFile(yaml_path);

    cfg.endpoint = root["endpoint"].as<std::string>();
    cfg.nodes.speed    = root["nodes"]["speed"].as<std::string>();
    cfg.nodes.mode     = root["nodes"]["mode"].as<std::string>();
    cfg.nodes.command  = root["nodes"]["command"].as<std::string>();
    cfg.structs.mod_root = root["structs"]["mod_root"].as<std::string>();
    cfg.structs.stat_root = root["structs"]["stat_root"].as<std::string>();
    cfg.structs.discover = root["structs"]["discover"].as<bool>();
    
    cfg.timing.rc.initial_ms = root["timing"]["reconnect"]["initial_ms"].as<int>();
    cfg.timing.rc.max_ms     = root["timing"]["reconnect"]["max_ms"].as<int>();
    cfg.timing.rc.multiplier = root["timing"]["reconnect"]["multiplier"].as<double>();

    cfg.timing.write_timeout_ms = root["timing"]["write_timeout_ms"].as<int>();
    cfg.timing.sampling_ms      = root["timing"]["sampling_ms"].as<int>();


    for(YAML::const_iterator it = root["structs"]["mod_fields"].begin(); it!=root["structs"]["mod_fields"].end(); ++it){
        cfg.structs.mod_fields.emplace_back(it->as<std::string>());
    }    

    for( const auto&  stat : root["structs"]["stat_fields"]){
        cfg.structs.stat_fields.emplace_back(stat.as<std::string>());
    }    
    return cfg;

}

