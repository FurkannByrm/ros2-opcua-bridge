#include "backend/opcua_client.hpp"



namespace {

   void handler_int(UA_Client*, UA_UInt32, void* subContext,
                          UA_UInt32 monId, void*, UA_DataValue* value) {
    auto self = static_cast<UaClient*>(subContext);
    if(!value || !value->hasValue || value->value.type != &UA_TYPES[UA_TYPES_INT16]) return;
    const int16_t v = *static_cast<int16_t*>(value->value.data);
    std::lock_guard<std::mutex> lk(self->sub_mtx_);
    auto it = self->int_cbs_.find(monId);
    if(it != self->int_cbs_.end() && it->second) it->second(v);
  }
  
  void handler_bool(UA_Client*, UA_UInt32, void* subContext,
                           UA_UInt32 monId, void*, UA_DataValue* value) {
    auto self = static_cast<UaClient*>(subContext);
    if(!value || !value->hasValue || value->value.type != &UA_TYPES[UA_TYPES_BOOLEAN]) return;
    const bool v = *static_cast<UA_Boolean*>(value->value.data) != 0;
    std::lock_guard<std::mutex> lk(self->sub_mtx_);
    auto it = self->bool_cbs_.find(monId);
    if(it != self->bool_cbs_.end() && it->second) it->second(v);
  }
  

  
}

bool UaClient::connect(const UaConfig& conf) {
  cfg_ = conf;
  connected_ = false;
  return true;
}

void UaClient::disconnect() {
  std::lock_guard<std::mutex> lk(ua_mtx_);
  if(!client_) { connected_ = false; return; }
  {
    std::lock_guard<std::mutex> lk2(sub_mtx_);
    int_cbs_.clear();
    bool_cbs_.clear();
    subs_int_.clear();
    subs_bool_.clear();
  }
  UA_Client_disconnect(client_);
  UA_Client_delete(client_);
  client_ = nullptr;
  connected_ = false;
}

void UaClient::start() {
  if(running_) return;
  running_ = true;
  worker_ = std::thread(&UaClient::worker_loop, this);
}

void UaClient::stop() {
  if(!running_) return;
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    cmd_q_.push_front({UaCommand::Type::Stop, "", false, 0, nullptr});
  }
  cmd_cv_.notify_one();
  if(worker_.joinable()) worker_.join();
  running_ = false;
}

bool UaClient::try_connect_once() {
  std::lock_guard<std::mutex> lk(ua_mtx_);
  if(client_) { UA_Client_disconnect(client_); UA_Client_delete(client_); client_ = nullptr; }
  client_ = UA_Client_new();
  UA_ClientConfig_setDefault(UA_Client_getConfig(client_));
  UA_StatusCode st = UA_Client_connect(client_, cfg_.endpoint.c_str());
  if(st == UA_STATUSCODE_GOOD) { connected_ = true; return true; }
  UA_Client_delete(client_); client_ = nullptr; connected_ = false;
  return false;
}

void UaClient::worker_loop() {
  int backoff = cfg_.timing.rc.initial_ms;
  const int backoff_max = cfg_.timing.rc.max_ms;
  const double mul = cfg_.timing.rc.multiplier;
  auto next_iterate = std::chrono::steady_clock::now();

  while(running_) {
    if(!connected_) {
      if(try_connect_once()) {
        rebind_all();
        backoff = cfg_.timing.rc.initial_ms;
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(backoff));
        backoff = std::min(static_cast<int>(backoff * mul), backoff_max);
        continue;
      }
    }

    UaCommand cmd;
    bool has_cmd = false;
    {
      std::unique_lock<std::mutex> lk(cmd_mtx_);
      cmd_cv_.wait_for(lk, std::chrono::milliseconds(50), [&]{
        return !cmd_q_.empty() || !running_;
      });
      if(!running_) break;
      if(!cmd_q_.empty()) {
        cmd = std::move(cmd_q_.front());
        cmd_q_.pop_front();
        has_cmd = true;
      }
    }

    if(has_cmd) {
      if(cmd.type == UaCommand::Type::Stop) break;
      if(!connected_) continue;
      switch(cmd.type) {
        case UaCommand::Type::WriteBool:  write_bool_worker(cmd.node, cmd.bval); break;
        case UaCommand::Type::WriteInt16: write_int16_worker(cmd.node, cmd.ival); break;
        case UaCommand::Type::SubBool:
          for(auto& s : desired_subs_bool_)
            if(s.node == cmd.node) sub_bool_worker(cmd.node, s.cb);
          break;
        case UaCommand::Type::SubInt16:
          for(auto& s : desired_subs_int_)
            if(s.node == cmd.node) sub_int16_worker(cmd.node, s.cb);
          break;
        default: break;
      }
      if(cmd.prom) cmd.prom->set_value(UA_STATUSCODE_GOOD);
    }

    // --- iterate ---
    if(connected_ && std::chrono::steady_clock::now() >= next_iterate) {
      std::lock_guard<std::mutex> lk(ua_mtx_);
      if(client_) {
        UA_StatusCode rc = UA_Client_run_iterate(client_, 0);
        if(rc != UA_STATUSCODE_GOOD) {
          connected_ = false;
          UA_Client_disconnect(client_);
          UA_Client_delete(client_);
          client_ = nullptr;
        }
      } else connected_ = false;
      next_iterate = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    }
  }

  std::lock_guard<std::mutex> lk(cmd_mtx_);
  while(!cmd_q_.empty()) cmd_q_.pop_front();
}

void UaClient::enqueue_write_bool(const std::string& node, bool v) {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  if(cmd_q_.size() >= cmd_q_max_) cmd_q_.pop_front();
  cmd_q_.push_back({UaCommand::Type::WriteBool, node, v, 0, nullptr});
  cmd_cv_.notify_one();
}

void UaClient::enqueue_write_int16(const std::string& node, int16_t v) {
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  if(cmd_q_.size() >= cmd_q_max_) cmd_q_.pop_front();
  cmd_q_.push_back({UaCommand::Type::WriteInt16, node, false, v, nullptr});
  cmd_cv_.notify_one();
}

void UaClient::subscribe_bool(const std::string& node, BoolCb cb) {
  {
    std::lock_guard<std::mutex> lk(sub_mtx_);
    desired_subs_bool_.push_back({node, 0, 0, cb});
  }
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  cmd_q_.push_back({UaCommand::Type::SubBool, node, false, 0, nullptr});
  cmd_cv_.notify_one();
}

void UaClient::subscribe_int16(const std::string& node, IntCb cb) {
  {
    std::lock_guard<std::mutex> lk(sub_mtx_);
    desired_subs_int_.push_back({node, 0, 0, cb});
  }
  std::lock_guard<std::mutex> lk(cmd_mtx_);
  cmd_q_.push_back({UaCommand::Type::SubInt16, node, false, 0, nullptr});
  cmd_cv_.notify_one();
}

//Worker Helpers
bool UaClient::write_int16_worker(const std::string& node_str, int16_t value) {
  if(!client_) return false;
  UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));
  UA_Variant var;
  UA_Variant_init(&var);
  UA_Variant_setScalarCopy(&var, &value, &UA_TYPES[UA_TYPES_INT16]);
  UA_StatusCode rc = UA_Client_writeValueAttribute(client_, id, &var);
  UA_Variant_clear(&var);
  return rc == UA_STATUSCODE_GOOD;
}

bool UaClient::write_bool_worker(const std::string& node_str, bool value) {
  if(!client_) return false;
  UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));
  UA_Boolean uaValue = value;
  UA_Variant var;
  UA_Variant_init(&var);
  UA_Variant_setScalarCopy(&var, &uaValue, &UA_TYPES[UA_TYPES_BOOLEAN]);
  UA_StatusCode rc = UA_Client_writeValueAttribute(client_, id, &var);
  UA_Variant_clear(&var);
  return rc == UA_STATUSCODE_GOOD;
}

// bool UaClient::sub_bool_worker(const std::string& node_str, BoolCb cb) {
//   if(!client_) return false;
//   UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));
//   UA_CreateSubscriptionRequest sreq = UA_CreateSubscriptionRequest_default();
//   sreq.requestedPublishingInterval = cfg_.timing.sampling_ms;
//   UA_CreateSubscriptionResponse sres = UA_Client_Subscriptions_create(client_, sreq, this, nullptr, nullptr);
//   if(sres.responseHeader.serviceResult != UA_STATUSCODE_GOOD) return false;
//   UA_MonitoredItemCreateRequest mreq = UA_MonitoredItemCreateRequest_default(id);
//   UA_MonitoredItemCreateResult mres = UA_Client_MonitoredItems_createDataChange(
//       client_, sres.subscriptionId, UA_TIMESTAMPSTORETURN_BOTH, mreq,
//       this, handler_bool, nullptr);
//   if(mres.statusCode != UA_STATUSCODE_GOOD) return false;
//   std::lock_guard<std::mutex> lk(sub_mtx_);
//   bool_cbs_[mres.monitoredItemId] = cb;
//   subs_bool_.push_back({node_str, sres.subscriptionId, mres.monitoredItemId, cb});
//   return true;
// }

// bool UaClient::sub_int16_worker(const std::string& node_str, IntCb cb) {
//   if(!client_) return false;
//   UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));
//   UA_CreateSubscriptionRequest sreq = UA_CreateSubscriptionRequest_default();
//   sreq.requestedPublishingInterval = cfg_.timing.sampling_ms;
//   UA_CreateSubscriptionResponse sres = UA_Client_Subscriptions_create(client_, sreq, this, nullptr, nullptr);
//   if(sres.responseHeader.serviceResult != UA_STATUSCODE_GOOD) return false;
//   UA_MonitoredItemCreateRequest mreq = UA_MonitoredItemCreateRequest_default(id);
//   UA_MonitoredItemCreateResult mres = UA_Client_MonitoredItems_createDataChange(
//       client_, sres.subscriptionId, UA_TIMESTAMPSTORETURN_BOTH, mreq,
//       this, handler_int, nullptr);
//   if(mres.statusCode != UA_STATUSCODE_GOOD) return false;
//   std::lock_guard<std::mutex> lk(sub_mtx_);
//   int_cbs_[mres.monitoredItemId] = cb;
//   subs_int_.push_back({node_str, sres.subscriptionId, mres.monitoredItemId, cb});
//   return true;
// }


bool UaClient::sub_bool_worker(const std::string& node_str, BoolCb cb) {
  if(!client_) return false;
  
  {
    std::lock_guard<std::mutex> lk(sub_mtx_);
    for(const auto& sub : subs_bool_) {
      if(sub.node == node_str) {
        return true;  
      }
    }
  }
  
  static UA_UInt32 shared_bool_subscription_id = 0;
  
  if (shared_bool_subscription_id == 0) {
    UA_CreateSubscriptionRequest sreq = UA_CreateSubscriptionRequest_default();
    sreq.requestedPublishingInterval = cfg_.timing.sampling_ms;
    UA_CreateSubscriptionResponse sres = UA_Client_Subscriptions_create(client_, sreq, this, nullptr, nullptr);
    if(sres.responseHeader.serviceResult != UA_STATUSCODE_GOOD) return false;
    shared_bool_subscription_id = sres.subscriptionId;
  }
  
  UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));
  UA_MonitoredItemCreateRequest mreq = UA_MonitoredItemCreateRequest_default(id);
  UA_MonitoredItemCreateResult mres = UA_Client_MonitoredItems_createDataChange(
      client_, shared_bool_subscription_id, UA_TIMESTAMPSTORETURN_BOTH, mreq,
      this, handler_bool, nullptr);
      
  if(mres.statusCode != UA_STATUSCODE_GOOD) {
    return false;
  }
    
  std::lock_guard<std::mutex> lk(sub_mtx_);
  bool_cbs_[mres.monitoredItemId] = cb;
  subs_bool_.push_back({node_str, shared_bool_subscription_id, mres.monitoredItemId, cb});
  return true;
}

bool UaClient::sub_int16_worker(const std::string& node_str, IntCb cb) {
  if(!client_) return false;
  
  {
    std::lock_guard<std::mutex> lk(sub_mtx_);
    for(const auto& sub : subs_int_) {
      if(sub.node == node_str) {
        return true;  
      }
    }
  }
  
  static UA_UInt32 shared_int_subscription_id = 0;
  
  if (shared_int_subscription_id == 0) {
    UA_CreateSubscriptionRequest sreq = UA_CreateSubscriptionRequest_default();
    sreq.requestedPublishingInterval = cfg_.timing.sampling_ms;
    UA_CreateSubscriptionResponse sres = UA_Client_Subscriptions_create(client_, sreq, this, nullptr, nullptr);
    if(sres.responseHeader.serviceResult != UA_STATUSCODE_GOOD) return false;
    shared_int_subscription_id = sres.subscriptionId;
  }
  
  UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));
  UA_MonitoredItemCreateRequest mreq = UA_MonitoredItemCreateRequest_default(id);
  UA_MonitoredItemCreateResult mres = UA_Client_MonitoredItems_createDataChange(
      client_, shared_int_subscription_id, UA_TIMESTAMPSTORETURN_BOTH, mreq,
      this, handler_int, nullptr);
      
  if(mres.statusCode != UA_STATUSCODE_GOOD) {
       return false;
  }
    
  std::lock_guard<std::mutex> lk(sub_mtx_);
  int_cbs_[mres.monitoredItemId] = cb;
  subs_int_.push_back({node_str, shared_int_subscription_id, mres.monitoredItemId, cb});
  return true;
}

// void UaClient::rebind_all() {
//   std::vector<IntSub> ints;
//   std::vector<BoolSub> bools;
//   {
//     std::lock_guard<std::mutex> lk(sub_mtx_);
//     ints = desired_subs_int_;
//     bools = desired_subs_bool_;
//   }
//   {
//     std::lock_guard<std::mutex> lk(cmd_mtx_);
//     for(auto& s : ints)
//       cmd_q_.push_back({UaCommand::Type::SubInt16, s.node, false, 0, nullptr});
//     for(auto& s : bools)
//       cmd_q_.push_back({UaCommand::Type::SubBool, s.node, false, 0, nullptr});
//   }
//   cmd_cv_.notify_one();
// }
void UaClient::rebind_all() {
  std::vector<IntSub> ints;
  std::vector<BoolSub> bools;
  {
    std::lock_guard<std::mutex> lk(sub_mtx_);
    ints = desired_subs_int_;
    bools = desired_subs_bool_;
    subs_int_.clear();
    subs_bool_.clear();
    int_cbs_.clear();
    bool_cbs_.clear();
  }
  {
    std::lock_guard<std::mutex> lk(cmd_mtx_);
    for(auto& s : ints)
      cmd_q_.push_back({UaCommand::Type::SubInt16, s.node, false, 0, nullptr});
    for(auto& s : bools)
      cmd_q_.push_back({UaCommand::Type::SubBool, s.node, false, 0, nullptr});
  }
  cmd_cv_.notify_one();
}
