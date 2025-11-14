#include "backend/opcua_client.hpp"
#include "backend/config.hpp"
#include "backend/naming.hpp"
#include <csignal>
#include <iostream>
#include <atomic>
#include <thread>

std::atomic<bool> g_running{true};

void signal_handler(int) {
    std::cout << "\n[INFO] SIGINT received, shutting down..." << std::endl;
    g_running = false;
}

int main() {
    std::signal(SIGINT, signal_handler);

    UaConfig cfg = ConfigLoader::load_file("/home/cengo/ros_ws/src/backend/config/opcua.yaml");
    std::cout << "[INFO] Loaded config. Endpoint: " << cfg.endpoint << std::endl;

    UaClient ua;
    if (!ua.connect(cfg)) {
        std::cerr << "[ERROR] UA connect() failed" << std::endl;
        return 1;
    }
    ua.start();

    for (int i = 0; i < 50 && !ua.is_connected(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!ua.is_connected()) {
        std::cerr << "[ERROR] Failed to connect to OPC UA Server!" << std::endl;
        ua.stop();
        return 1;
    }

    std::cout << "[INFO] Connected successfully to OPC UA Server" << std::endl;


    ua.subscribe_int16(cfg.nodes.speed, [](int16_t v) {
        std::cout << "[SUB] SPEED updated: " << v << std::endl;
    });

    ua.subscribe_bool(make_child_node(cfg.structs.stat_root, "SYSTEM_READY"), [](bool v) {
        std::cout << "[SUB] STAT.SYSTEM_READY = " << std::boolalpha << v << std::endl;
    });

    ua.subscribe_bool(make_child_node(cfg.structs.mod_root, "COBOT"), [](bool v) {
        std::cout << "[SUB] MOD.COBOT = " << std::boolalpha << v << std::endl;
    });


    std::this_thread::sleep_for(std::chrono::seconds(1));
    ua.enqueue_write_int16(cfg.nodes.speed, 1200);
    std::cout << "[CMD] SPEED -> 1200" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    ua.enqueue_write_bool(make_child_node(cfg.structs.mod_root, "COBOT"), true);
    std::cout << "[CMD] MOD.COBOT -> TRUE" << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    ua.enqueue_write_bool(make_child_node(cfg.structs.mod_root, "COBOT"), false);
    std::cout << "[CMD] MOD.COBOT -> FALSE" << std::endl;


    std::cout << "\n[INFO] Running... Press Ctrl+C to exit\n";
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[INFO] Stopping UaClient..." << std::endl;
    ua.stop();
    ua.disconnect();

    std::cout << "[INFO] Graceful shutdown complete." << std::endl;
    return 0;
}
