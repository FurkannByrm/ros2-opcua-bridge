#include "backend/opcua_client.hpp"

static void handler_int(UA_Client* client,
                        UA_UInt32 subId,
                        void* subContext,
                        UA_UInt32 monId,
                        void* monContext,
                        UA_DataValue* value) {
  auto self = static_cast<UaClient*>(subContext);
  if(!value || !value->hasValue || value->value.type != &UA_TYPES[UA_TYPES_INT16]) return;
  const int16_t v = *static_cast<int16_t*>(value->value.data);

  std::lock_guard<std::mutex> lk(self->sub_mtx_);
  auto it = self->int_cbs_.find(monId);
  if(it != self->int_cbs_.end() && it->second) {
    it->second(v);
  }
}

static void handler_bool(UA_Client* client,
                        UA_UInt32 subId,
                        void* subContext,
                        UA_UInt32 monId,
                        void* monContext,
                        UA_DataValue* value) {
  auto self = static_cast<UaClient*>(subContext);
  if(!value || !value->hasValue || value->value.type != &UA_TYPES[UA_TYPES_BOOLEAN]) return;
  const bool v = *static_cast<UA_Boolean*>(value->value.data) != 0;

  std::lock_guard<std::mutex> lk(self->sub_mtx_);
  auto it = self->bool_cbs_.find(monId);
  if(it != self->bool_cbs_.end() && it->second) {
    it->second(v);
  }
}


bool UaClient::connect(const UaConfig& conf){

    cfg_ = conf;
    client_ = UA_Client_new();
    UA_ClientConfig_setDefault(UA_Client_getConfig(client_));
    UA_StatusCode status = UA_Client_connect(client_, cfg_.endpoint.c_str());
    return status == UA_STATUSCODE_GOOD;

}

void UaClient::disconnect(){
  if (client_)
    {
        UA_Client_disconnect(client_);
        UA_Client_delete(client_);
        client_ = nullptr;
    }
}


bool UaClient::try_connect(){
    client_ = UA_Client_new();
    UA_ClientConfig_setDefault(UA_Client_getConfig(client_));
    UA_StatusCode rc = UA_Client_connect(client_,cfg_.endpoint.c_str());
    if(rc != UA_STATUSCODE_GOOD){
        UA_Client_delete(client_);
        client_ = nullptr;
        return false;
    }
    return true;
}

void UaClient::start(){
    running_ = true;
    worker_ = std::thread(&UaClient::worker_loop, this);
}

void UaClient::stop(){
    running_ = false;
    if(worker_.joinable()) worker_.join();
}

void UaClient::worker_loop(){
    int backoff = cfg_.timing.rc.initial_ms;
    while(running_){
        if(!connected_){
             if(try_connect()) {
            connected_ = true;
            rebind_all();
            backoff = cfg_.timing.rc.initial_ms;
        }else{
            std::this_thread::sleep_for(std::chrono::milliseconds(backoff));
            backoff = std::min(int(backoff * cfg_.timing.rc.multiplier),cfg_.timing.rc.max_ms);
            continue;
        }
    }

    UA_StatusCode rc = UA_Client_run_iterate(client_,50);
    if(rc!= UA_STATUSCODE_GOOD){
        connected_ = false;
        UA_Client_disconnect(client_);
      }
    }
}


void UaClient::rebind_all(){
    std::vector<IntSub> int_subs_copy;
    std::vector<BoolSub> bool_subs_copy;
    
    {
        std::lock_guard<std::mutex> lk(sub_mtx_);
        int_subs_copy = subs_int_;
        bool_subs_copy = subs_bool_;
        // Eski callback'leri temizle
        int_cbs_.clear();
        bool_cbs_.clear();
        subs_int_.clear();
        subs_bool_.clear();
    }
    
    for(const auto& sub : int_subs_copy){
        subscribe_int16(sub.node, sub.cb);
    }

    for(const auto& sub : bool_subs_copy){
        subscribe_bool(sub.node, sub.cb);
    }

}

bool UaClient::write_int16(const std::string& node_str, int16_t value){
    if(!client_) return false;
    
    UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));
    UA_Variant var;
    UA_Variant_init(&var);
    UA_Variant_setScalarCopy(&var, &value, &UA_TYPES[UA_TYPES_INT16]);
     
    UA_StatusCode rc = UA_Client_writeValueAttribute(client_, id, &var);
    UA_Variant_clear(&var);
    return rc == UA_STATUSCODE_GOOD;
}


bool UaClient::write_bool(const std::string& node_str, bool value){
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


bool UaClient::subscribe_bool(const std::string& node_str, BoolCb cb){
    if(!client_) return false;
    UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));

    UA_CreateSubscriptionRequest sreq = UA_CreateSubscriptionRequest_default();
    sreq.requestedPublishingInterval = static_cast<UA_Double>(cfg_.timing.sampling_ms);

    UA_CreateSubscriptionResponse sres =
    UA_Client_Subscriptions_create(client_, sreq, this, nullptr, nullptr);
    if(sres.responseHeader.serviceResult != UA_STATUSCODE_GOOD) return false;

    UA_MonitoredItemCreateRequest mreq = UA_MonitoredItemCreateRequest_default(id);
    mreq.requestedParameters.samplingInterval = static_cast<UA_Double>(cfg_.timing.sampling_ms);
    mreq.requestedParameters.queueSize = 10;

    UA_MonitoredItemCreateResult mres = UA_Client_MonitoredItems_createDataChange(
    client_,
    sres.subscriptionId,
    UA_TIMESTAMPSTORETURN_BOTH,
    mreq,                    // & yok
    this,                    // context
    handler_bool,             // callback
    nullptr                  // deleteCallback
  );

   if(mres.statusCode != UA_STATUSCODE_GOOD) {
    return false;
  }

  UA_UInt32 monId = mres.monitoredItemId;

  std::lock_guard<std::mutex> lk(sub_mtx_);
  bool_cbs_[monId] = std::move(cb);
  
  subs_bool_.push_back({node_str, sres.subscriptionId, monId, cb});
  
  return true;

}


bool UaClient::subscribe_int16(const std::string& node_str, IntCb cb) {
  if(!client_) return false;

  UA_NodeId id = UA_NODEID_STRING(cfg_.ns_index, const_cast<char*>(node_str.c_str()));

  UA_CreateSubscriptionRequest sreq = UA_CreateSubscriptionRequest_default();
  sreq.requestedPublishingInterval = static_cast<UA_Double>(cfg_.timing.sampling_ms);
  UA_CreateSubscriptionResponse sres =
      UA_Client_Subscriptions_create(client_, sreq, this, nullptr, nullptr);
  if(sres.responseHeader.serviceResult != UA_STATUSCODE_GOOD) return false;

  UA_MonitoredItemCreateRequest mreq = UA_MonitoredItemCreateRequest_default(id);
  mreq.requestedParameters.samplingInterval = static_cast<UA_Double>(cfg_.timing.sampling_ms);
  mreq.requestedParameters.queueSize = 10;

  UA_MonitoredItemCreateResult mres = UA_Client_MonitoredItems_createDataChange(
      client_,
      sres.subscriptionId,
      UA_TIMESTAMPSTORETURN_BOTH,
      mreq,                    
      this,                    
      handler_int,             
      nullptr                  
  );

  if(mres.statusCode != UA_STATUSCODE_GOOD) {
    return false;
  }

  UA_UInt32 monId = mres.monitoredItemId;

  std::lock_guard<std::mutex> lk(sub_mtx_);
  int_cbs_[monId] = std::move(cb);
  
  subs_int_.push_back({node_str, sres.subscriptionId, monId, cb});
  
  return true;
}
