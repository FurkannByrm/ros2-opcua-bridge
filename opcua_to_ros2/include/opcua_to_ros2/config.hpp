#ifndef CONFIG_HPP
#define CONFIG_HPP

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

struct StructsCfg{
    std::string mod_root, stat_root, sensing_root, cleaning_root, workcell_status, spot_M1_root, spot_M2_root;
    std::vector<std::string> spatter1_vec;
    std::vector<std::string> spatter2_vec;
    std::vector<std::string> spatter_size;
};

struct UaConfig{
    std::string endpoint;
    int ns_index;
    StructsCfg structs;
    TimingCfg timing;
};

class ConfigLoader{

    public:
    static UaConfig load_file(const std::string& yaml_path);

};




#endif //CONFIG_HPP
