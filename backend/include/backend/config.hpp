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
    std::string status, mode, speed, command;
};

struct StructsCfg{
    std::string mod_root, stat_root;
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
// namespace YAML_parser {
    
//     struct Config{
    
//         std::string endpoint;
//         std::unordered_map<std::string, std::string> nodes;
//         std::unordered_map<std::string, std::string> mod_root;
//         int namespaceIndex{0};
//     };
    
//     class Parser{
    
//         public:

//         Parser();
//         void parse();

//         void getConfig()const{
//         std::cout<<"Endpoint : "<<conf_.endpoint<<"\n";
//         std::cout<<"NamespaceIndex : "<<conf_.namespaceIndex<<"\n";
//         for(const auto& [node, value] : conf_.nodes){
//         std::cout<<"[Node] : "<<node<<" [value] : "<<value<<"\n";
//         }
//     }

//         private:

//         Config conf_;
//         YAML::Node node_;
//         const std::string& path_;
// };

// }






#endif //CONFIG_HPP