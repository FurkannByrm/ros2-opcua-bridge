#include "backend/opcua_client.hpp"
#include "backend/config.hpp"
#include <iostream>
#include <csignal>
#include <thread>
#include <memory>


int main() {
    
    UaConfig cfg = ConfigLoader::load_file("/home/cengo/ros_ws/src/backend/config/opcua.yaml");
    
    UaClient ua;
    ua.connect(cfg);
    ua.start();

    ua.subscribe_int16(cfg.nodes.speed, [](int16_t v){});
    ua.subscribe_bool (cfg.structs.mod_root, [](bool b){ });


    ua.write_int16(cfg.nodes.speed, 1200);
    ua.write_bool(cfg.structs.mod_fields[1], true);
    
    ua.stop();      
    ua.disconnect();
}