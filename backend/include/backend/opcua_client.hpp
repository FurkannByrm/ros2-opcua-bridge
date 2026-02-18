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
#include <memory>
#include <future>
#include <deque>

class UaClient {

  public:
  
  UaClient() = default;
  ~UaClient() = default;

  bool connect(const UaConfig& cfg);
  void disconnect();
  bool is_connected() const { return connected_; }

  void start();
  void stop();

  void enqueue_write_bool (const std::string& node, bool v);
  void enqueue_write_int16(const std::string& node, int16_t v);
  void enqueue_write_double(const std::string& node, double v);



  using IntCb  = std::function<void(int16_t)>;
  using BoolCb = std::function<void(bool)>;
  using DoubleCb= std::function<void(double)>;

  void subscribe_int16(const std::string& node, IntCb cb);
  void subscribe_bool (const std::string& node, BoolCb cb);
  void subscribe_double(const std::string& node, DoubleCb cb);

  std::mutex sub_mtx_;
  std::unordered_map<UA_UInt32, IntCb>  int_cbs_;
  std::unordered_map<UA_UInt32, BoolCb> bool_cbs_;
  std::unordered_map<UA_UInt32, DoubleCb> double_cbs_;

private:

  void worker_loop();
  bool try_connect_once();
  void rebind_all();

  bool write_int16_worker(const std::string& node_str, int16_t value);
  bool write_bool_worker (const std::string& node_str, bool value);
  bool write_double_worker(const std::string& node_str, double value);

  bool sub_int16_worker  (const std::string& node_str, IntCb cb);
  bool sub_bool_worker   (const std::string& node_str, BoolCb cb);
  bool sub_double_worker (const std::string& node_str, DoubleCb cb);

  struct UaCommand {
    enum class Type { WriteBool, WriteInt16, WriteDouble, SubBool, SubInt16, SubDouble, Stop } type;
    std::string node;
    bool bval{false};
    int16_t ival{0};
    double dval{0.0};
    std::shared_ptr<std::promise<UA_StatusCode>> prom;
  };
  std::deque<UaCommand> cmd_q_;
  std::mutex ua_mtx_, cmd_mtx_;
  std::condition_variable cmd_cv_;

  UA_Client* client_{nullptr};
  UaConfig   cfg_;
  size_t cmd_q_max_ = 1024;

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
  struct DoubleSub{
    std::string node;
    UA_UInt32   sid;
    UA_UInt32   mid;
    DoubleCb    cb;
  };


  std::vector<IntSub>  desired_subs_int_;
  std::vector<BoolSub> desired_subs_bool_;
  std::vector<DoubleSub> desired_subs_double_;
  
  std::vector<IntSub>  subs_int_;
  std::vector<BoolSub> subs_bool_;
  std::vector<DoubleSub> subs_double_;
};

#endif // UA_CLIENT_HPP
