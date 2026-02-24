#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

struct ReconnectCfg { 

    int initial_ms, max_ms; 
    double multiplier; 
};

struct TimingCfg { 

    int sampling_ms, write_timeout_ms;
    ReconnectCfg rc; 

};

struct NodesCfg {
    std::string status, mode, speed, command, slider1_go, slider2_go;
};

struct StructsCfg{
    std::string mod_root, stat_root, sensing_root, cleaning_root, workcell_status;
    bool discover;
    std::vector<std::string> mod_fields;
    std::vector<std::string> stat_fields;
};

struct UaConfig{
    std::string endpoint;
    int ns_index;
    NodesCfg nodes;
    StructsCfg structs;
    TimingCfg timing;
};


class ConfigLoader{

    public:
    static UaConfig load_file(const std::string& yaml_path);


};




#endif //CONFIG_HPP