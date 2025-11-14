                 ┌──────────────────────────────┐
                 │            GUI / ROS         │
                 │ (buton, service, callback)   │
                 └──────────────┬───────────────┘
                                │
                                │ enqueue_write_x()
                                ▼
                    ┌────────────────────────────┐
                    │        UaClient (main)     │
                    │----------------------------│
                    │   cmd_q_  ←  mutex + CV    │◄────────────┐
                    │   connected_               │             │
                    │   worker_thread            │             │
                    └────────────┬───────────────┘             │
                                 │                             │
                                 ▼                             │
         ┌──────────────────────────────────────────────────────────┐
         │                    Worker Thread (loop)                  │
         │----------------------------------------------------------│
         │  while(running_):                                        │
         │    if(!connected_) try_connect_once()                    │
         │        │                                                 │
         │        ├── success → rebind_all()                        │
         │        └── fail → sleep(backoff)                         │
         │                                                          │
         │    ┌──────────────────────────────────────────────┐      │
         │    │    Queue control                             │      │
         │    │   - cmd_q_ varsa pop_front()                 │      │
         │    │   - WriteBool / WriteInt16                   │      │
         │    │   - UA_Client_writeValueAttribute()          │      │
         │    │   - Promise set_value(status) (for sync )    │      │
         │    └──────────────────────────────────────────────┘      │
         │                                                          │
         │    ┌──────────────────────────────────────────────┐      │
         │    │    Event Loop                                │      │
         │    │   UA_Client_run_iterate(client_, 50)         │      │
         │    │   → Subscription callback (read)             │      │
         │    │     handler_int / handler_bool               │      │
         │    │     callback ROS topic publisher             │      │
         │    └──────────────────────────────────────────────┘      │
         │                                                          │
         │    - connection error → connected_ = false → retry       │
         └──────────────────────────────────────────────────────────┘

