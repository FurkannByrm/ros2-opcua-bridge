#ifndef UA_CLIENT_HPP
#define UA_CLIENT_HPP

#include <open62541/client_highlevel.h>
#include <open62541/client_subscriptions.h>
#include <open62541/client_config_default.h>

#include "backend/config.hpp"

#include <functional>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <string>

class UaClient {
public:
  UaClient() = default;
  ~UaClient() = default;

  bool connect(const UaConfig& cfg);
  void disconnect();
  bool is_connected() const { return connected_; }

  void start();   
  void stop();

  bool write_int16(const std::string& node_str, int16_t value);
  bool write_bool (const std::string& node_str, bool value);


  using IntCb  = std::function<void(int16_t)>;
  using BoolCb = std::function<void(bool)>;


  bool subscribe_int16(const std::string& node_str, IntCb cb);
  bool subscribe_bool (const std::string& node_str, BoolCb cb);


  std::mutex sub_mtx_;
  std::unordered_map<UA_UInt32, IntCb>  int_cbs_;
  std::unordered_map<UA_UInt32, BoolCb> bool_cbs_;

private:
  
  void worker_loop();
  bool try_connect_once();
  void rebind_all();

  std::mutex ua_mtx_;
  UA_Client* client_{nullptr};
  UaConfig   cfg_;

  std::thread worker_;
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};

  struct IntSub {
    std::string node;
    UA_UInt32   sid;
    UA_UInt32   mid;
    IntCb       cb;
  };
  struct BoolSub {
    std::string node;
    UA_UInt32   sid;
    UA_UInt32   mid;
    BoolCb      cb;
  };

  std::vector<IntSub>  subs_int_;
  std::vector<BoolSub> subs_bool_;
};

#endif // UA_CLIENT_HPP
