# Magician ROS 2 — Endüstriyel Robot Hücresi Kontrol Sistemi

Endüstriyel bir robot hücresini (PLC, sensing/cleaning cobotları, slider mekanizmaları) ROS 2 üzerinden yönetmek için geliştirilmiş bir otomasyon yazılım sistemidir. OPC UA protokolü ile PLC'ye bağlanır, Qt5 tabanlı operatör arayüzü sunar ve BehaviorTree tabanlı robot orkestrasyon mantığı içerir.

> Kurulum adımları için bkz. [INSTALLATION.md](INSTALLATION.md)

---

## İçindekiler

- [Sistem Mimarisi](#sistem-mimarisi)
- [Paketler](#paketler)
  - [backend — OPC UA ↔ ROS 2 Köprüsü](#backend--opc-ua--ros-2-köprüsü)
  - [gui_app — Operatör Arayüzü](#gui_app--operatör-arayüzü)
  - [demonstrator_tree — BehaviorTree Orkestrasyon](#demonstrator_tree--behaviortree-orkestrasyon)
- [Sistemi Çalıştırma](#sistemi-çalıştırma)
- [ROS 2 Arayüzleri](#ros-2-arayüzleri)
- [OPC UA Adres Alanı](#opc-ua-adres-alanı)
- [Kod Yapısı](#kod-yapısı)
- [Hata Ayıklama](#hata-ayıklama)

---

## Sistem Mimarisi

Sistem üç ROS 2 paketinden oluşur ve katmanlı bir mimari izler:

```
                        ┌───────────────────────────────────────────────────────────┐
                        │                  Operatör Katmanı                        │
                        │                                                           │
                        │  ┌─────────────────────────────────────────────────────┐  │
                        │  │              gui_app  (Qt5 GUI)                     │  │
                        │  │  Hız ayarı · Mod değiştirme · Robot kontrolleri    │  │
                        │  │  Slider konumları · Gerçek zamanlı durum izleme    │  │
                        │  └──────────────────────┬──────────────────────────────┘  │
                        │                         │ ROS 2 Service Çağrıları         │
                        │                         │ ROS 2 Topic Abonelikleri        │
                        └─────────────────────────┼────────────────────────────────┘
                                                  │
                        ┌─────────────────────────┼────────────────────────────────┐
                        │              Haberleşme Katmanı                          │
                        │                         │                                │
                        │  ┌──────────────────────▼──────────────────────────────┐  │
                        │  │           backend  (opc_bridge düğümü)             │  │
                        │  │                                                     │  │
                        │  │  ┌─────────────┐   ┌────────────┐   ┌───────────┐  │  │
                        │  │  │ UaClient    │   │ RosBridge  │   │ Config    │  │  │
                        │  │  │ (open62541) │◄─►│ (rclcpp)   │   │ (YAML)   │  │  │
                        │  │  └──────┬──────┘   └────────────┘   └───────────┘  │  │
                        │  │         │                                           │  │
                        │  └─────────┼───────────────────────────────────────────┘  │
                        │            │ OPC UA (TCP)                                 │
                        └────────────┼─────────────────────────────────────────────┘
                                     │
                        ┌────────────┼─────────────────────────────────────────────┐
                        │  Saha      │  Katmanı                                    │
                        │            ▼                                              │
                        │  ┌──────────────────┐                                    │
                        │  │   Siemens PLC     │ ← Gerçek üretim ortamı            │
                        │  │  (veya test_server│   veya localhost simülasyonu       │
                        │  │   simülasyonu)    │                                    │
                        │  └──────────────────┘                                    │
                        └──────────────────────────────────────────────────────────┘

                        ┌──────────────────────────────────────────────────────────┐
                        │               Orkestrasyon Katmanı                       │
                        │                                                          │
                        │  ┌────────────────────────────────────────────────────┐   │
                        │  │         demonstrator_tree  (BehaviorTree.CPP)      │   │
                        │  │                                                    │   │
                        │  │  Sequence                                          │   │
                        │  │  ├── Fallback                                      │   │
                        │  │  │   ├── IsRobotAtHome  → joint_states kontrol     │   │
                        │  │  │   └── CallHoming     → homing servis çağrısı    │   │
                        │  │  └── CallOpcUI          → PLC'ye safe-transfer     │   │
                        │  └────────────────────────────────────────────────────┘   │
                        │         │                              │                  │
                        │         │ /xbotcore/joint_states       │ /ros2_comm/      │
                        │         │ /xbotcore/homing/switch*     │ safetransfer_set │
                        │         ▼                              ▼                  │
                        │  ┌──────────────┐           ┌──────────────────┐          │
                        │  │ Robot        │           │ backend          │          │
                        │  │ Kontrolcüler │           │ (opc_bridge)     │          │
                        │  │ (xbotcore)   │           │                  │          │
                        │  └──────────────┘           └──────────────────┘          │
                        └──────────────────────────────────────────────────────────┘
```

### Veri Akışı

```
PLC ──(OPC UA subscription)──► UaClient ──(callback)──► RosBridge ──(publish)──► ROS 2 Topics ──► GUI / BT
GUI ──(service call)──► RosBridge ──(enqueue_write)──► UaClient ──(OPC UA write)──► PLC
BT  ──(service call)──► RosBridge ──(enqueue_write)──► UaClient ──(OPC UA write)──► PLC
```

---

## Paketler

### backend — OPC UA ↔ ROS 2 Köprüsü

Sistemin çekirdek paketidir. PLC ile OPC UA protokolü üzerinden çift yönlü haberleşme sağlar ve tüm verileri ROS 2 topic/service olarak sunar.

#### Temel Bileşenler

| Sınıf | Dosya | Görev |
|---|---|---|
| `UaClient` | `opcua_client.hpp/cpp` | open62541 istemcisi. Bağlantı, subscription, yazma işlemleri. Ayrı worker thread'de çalışır. Mutex korumalı komut kuyruğu. |
| `RosBridge` | `ros_bridge.hpp/cpp` | ROS 2 düğümü. Publisher ve service tanımları. `UaClient`'tan gelen verileri topic olarak yayınlar, servis çağrılarını `UaClient`'a iletir. |
| `ConfigLoader` | `config.hpp/cpp` | YAML dosyalarından (`opcua.yaml` / `opcua_test.yaml`) konfigürasyonu okur. |
| `test_server` | `test/test_server.cpp` | PLC'yi simüle eden OPC UA sunucusu. Geliştirme ortamında donanımsız test imkanı sağlar. Yazma işlemlerini renkli terminal çıktısı olarak gösterir. |

#### Özellikler

- **Çift yönlü OPC UA haberleşme** — open62541 C kütüphanesi ile endüstriyel PLC'lere bağlanır
- **Otomatik yeniden bağlanma** — Yapılandırılabilir exponential backoff ile bağlantı kesintilerini otomatik yönetir
- **Çoklu iş parçacığı** — Worker thread, mutex korumalı asenkron komut kuyruğu (`std::deque<UaCommand>`)
- **YAML konfigürasyon** — Endpoint, namespace, node yolları, zamanlama parametreleri dışarıdan ayarlanabilir
- **Özel servis tipleri** — `SetInt16.srv` (hız), `SetFloat32.srv` (slider konumu)

#### Konfigürasyon Dosyaları

`backend/config/` altında iki yapılandırma dosyası bulunur:

| Dosya | Kullanım |
|---|---|
| `opcua.yaml` | **Üretim** — Gerçek PLC endpoint'i (`opc.tcp://192.168.1.1:4840`) |
| `opcua_test.yaml` | **Geliştirme** — Yerel test sunucusu (`opc.tcp://localhost:4840`) |

```yaml
endpoint: "opc.tcp://192.168.1.1:4840"
namespace_index: 3

nodes:
  status:   '"ROS2_COMM"."STATUS"'        # Int16
  mode:     '"ROS2_COMM"."MODE"'           # Int16
  command:  '"ROS2_COMM"."COMMAND"'        # Int16
  speed:    '"ROS2_COMM"."SPEED"'          # Int16
  slider1:  '"ROS2_COMM"."GO_TO_POS_1"'   # Double
  slider2:  '"ROS2_COMM"."GO_TO_POS_2"'   # Double

structs:
  mod_root:      '"ROS2_COMM"."MOD"'
  stat_root:     '"ROS2_COMM"."STAT"'
  sensing_root:  '"ROS2_COMM"."STAT"."Robot_Sensing_Status"'
  cleaning_root: '"ROS2_COMM"."STAT"."Robot_Cleaning_Status"'
  Workcell:      '"ROS2_COMM"."MOD"."Workcell_Status"'

timing:
  sampling_ms: 50               # Subscription örnekleme periyodu
  write_timeout_ms: 200         # Yazma zaman aşımı
  reconnect:
    initial_ms: 500             # İlk yeniden bağlantı bekleme süresi
    max_ms: 10000               # Maksimum bekleme süresi
    multiplier: 2.0             # Exponential backoff çarpanı
```

---

### gui_app — Operatör Arayüzü

Qt5 Widgets tabanlı operatör kontrol paneli. ROS 2 servis çağrıları ile `backend` üzerinden PLC'ye komut gönderir, topic abonelikleri ile gerçek zamanlı durum izler.

#### Özellikler

- **Hız kontrolü** — 0–2000 aralığında hız ayarı (`SetInt16` servisi)
- **Mod değiştirme** — COBOT, STARTUP, MAINTENANCE vb. mod toggle butonları
- **Sensing robot kontrolleri** — Safe-transfer, sensing-active, touch-sensing, finished, slide-command, running
- **Cleaning robot kontrolleri** — Safe-transfer, cleaning-active, finished, slide-command, running
- **Slider pozisyon kontrolleri** — İki slider için konum belirleme (`SetFloat32`) ve hareket başlatma
- **Gerçek zamanlı durum** — Tüm durumlar ROS 2 topic'lerinden anlık güncellenir

#### Teknik Detay

- `MainWindow` sınıfı hem Qt5 QMainWindow hem de dahili bir `rclcpp::Node` barındırır
- `QTimer` ile ROS 2 executor periyodik olarak döndürülür (UI thread'i bloklanmaz)
- Toggle butonları aktif/pasif duruma göre renk değiştirir

---

### demonstrator_tree — BehaviorTree Orkestrasyon

BehaviorTree.CPP kütüphanesi ile robotların güvenli pozisyonda olup olmadığını kontrol eder, gerekirse homing işlemi başlatır ve PLC'ye safe-transfer sinyali gönderir.

#### BehaviorTree Yapısı

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="magician_sequence">
      <Fallback>
        <IsRobotAtHome   name="check_home_pos"/>
        <CallHoming      name="call_homing_service"/>
      </Fallback>
      <CallOpcUI         name="call_opcua_service"/>
    </Sequence>
  </BehaviorTree>
</root>
```

**Akış mantığı:**
1. `IsRobotAtHome` — Her iki robotun (`sensing_cobot`, `cleaning_cobot`) joint pozisyonlarını kontrol eder (±0.01 rad tolerans)
2. `CallHoming` — Home pozisyonunda olmayan robotlar için homing servisini çağırır (paralel async çağrı desteği, 5 sn timeout)
3. `CallOpcUI` — Tüm robotlar home'da ise PLC'ye safe-transfer sinyali gönderir

#### BT Düğüm Sınıfları

| Sınıf | Görev |
|---|---|
| `MagicianSubNode` | `/xbotcore/joint_states` topic'ine abone olur. Anlık eklem açılarını `parameters.yaml`'daki home pozisyonları ile karşılaştırır. |
| `MagicianClientNode` | Homing servislerini (`/xbotcore/homing/switch1`, `switch2`) çağırır. Her iki robot için paralel async çağrı yapabilir. |
| `MagicianOpcUA` | `/ros2_comm/sensing/safetransfer_set` ve `/ros2_comm/cleaning/safetransfer_set` servislerini çağırarak PLC'ye safe-transfer durumunu bildirir. |

#### Konfigürasyon

`demonstrator_tree/config/parameters.yaml`:

```yaml
cobot1:
  robot_name: "sensing_cobot"
  sensing_joint_states: "/xbotcore/joint_states"
  sensing_service: "/xbotcore/homing/switch1"
  home_position: [0.0, 0.605, 1.571, 0.0, 0.995, 0.0]

cobot2:
  robot_name: "cleaning_cobot"
  cleaning_joint_states: "/xbotcore/joint_states"
  cleaning_service: "/xbotcore/homing/switch2"
  home_position: [0.0, 0.605, 1.571, 0.0, 0.995, 0.0]
```

#### Dış Bağımlılıklar (xbotcore tarafı)

| Arayüz | Tip | Yön |
|---|---|---|
| `/xbotcore/joint_states` | `sensor_msgs/JointState` | ← Abone olunur |
| `/xbotcore/homing/switch1` | `std_srvs/SetBool` | → Çağrılır |
| `/xbotcore/homing/switch2` | `std_srvs/SetBool` | → Çağrılır |

---

## Sistemi Çalıştırma

### Üretim Ortamı (Gerçek PLC)

PLC endpoint'ini `backend/config/opcua.yaml` dosyasında ayarlayın, ardından:

```bash
# Launch ile (opc_bridge + GUI otomatik başlar)
ros2 launch backend system.launch.py

# veya ayrı terminallerde:
ros2 run backend opc_bridge                   # OPC UA köprüsü
ros2 run gui_app gui_node                     # Qt5 GUI
ros2 run demonstrator_tree demo               # BehaviorTree
```

### Test Ortamı (PLC olmadan)

```bash
# Sadece backend testi (test_server + opc_bridge)
ros2 launch backend test_system.launch.py

# Tam sistem testi (test_server + opc_bridge + GUI)
ros2 launch backend full_test_system.launch.py
```

veya manuel olarak:

```bash
ros2 run backend test_server                                          # OPC UA simülasyon sunucusu (localhost:4840)
ros2 run backend opc_bridge --ros-args -p config:=opcua_test.yaml     # Köprü → localhost
ros2 run gui_app gui_node                                             # GUI
```

Test sunucusu her yazma işlemini renkli terminal çıktısı ile gösterir — servis çağrılarını gerçek zamanlı izleyebilirsiniz.

### BehaviorTree Testi

```bash
# Mock homing servisi (ayrı terminalde)
ros2 run demonstrator_tree service_server.py

# BehaviorTree düğümü
ros2 run demonstrator_tree demo
```

---

## ROS 2 Arayüzleri

### Yayınlanan Topic'ler (backend → subscribers)

| Topic | Tip | Açıklama |
|---|---|---|
| `/ros2_comm/speed` | `std_msgs/Int16` | Güncel hız değeri |
| `/ros2_comm/mod/cobot` | `std_msgs/Bool` | COBOT modu durumu |
| `/ros2_comm/sensing/home_st` | `std_msgs/Bool` | Sensing robot home (safe-transfer) durumu |
| `/ros2_comm/sensing/finished` | `std_msgs/Bool` | Sensing tamamlandı |
| `/ros2_comm/sensing/touch_finished` | `std_msgs/Bool` | Touch-sensing tamamlandı |
| `/ros2_comm/sensing/sensing_active` | `std_msgs/Bool` | Sensing aktif |
| `/ros2_comm/sensing/touch_active` | `std_msgs/Bool` | Touch-sensing aktif |
| `/ros2_comm/sensing/slide_command` | `std_msgs/Bool` | Sensing slide komutu |
| `/ros2_comm/sensing/running` | `std_msgs/Bool` | Sensing çalışıyor |
| `/ros2_comm/cleaning/home_st` | `std_msgs/Bool` | Cleaning robot home (safe-transfer) durumu |
| `/ros2_comm/cleaning/finished` | `std_msgs/Bool` | Cleaning tamamlandı |
| `/ros2_comm/cleaning/cleaning_active` | `std_msgs/Bool` | Cleaning aktif |
| `/ros2_comm/cleaning/slide_command` | `std_msgs/Bool` | Cleaning slide komutu |
| `/ros2_comm/cleaning/running` | `std_msgs/Bool` | Cleaning çalışıyor |

### Servisler (clients → backend → PLC)

| Servis | Tip | Açıklama |
|---|---|---|
| `/ros2_comm/speed_set` | `backend/SetInt16` | Hız değeri ayarla |
| `/ros2_comm/mod/cobot_set` | `std_srvs/SetBool` | COBOT modunu aç/kapat |
| `/ros2_comm/sensing/safetransfer_set` | `std_srvs/SetBool` | Sensing safe-transfer |
| `/ros2_comm/sensing/finished_set` | `std_srvs/SetBool` | Sensing finished |
| `/ros2_comm/sensing/touch_finished_set` | `std_srvs/SetBool` | Touch-sensing finished |
| `/ros2_comm/sensing/active_set` | `std_srvs/SetBool` | Sensing active |
| `/ros2_comm/sensing/touch_active_set` | `std_srvs/SetBool` | Touch-sensing active |
| `/ros2_comm/sensing/slide_command_set` | `std_srvs/SetBool` | Sensing slide command |
| `/ros2_comm/sensing/running` | `std_srvs/SetBool` | Sensing running |
| `/ros2_comm/cleaning/safetransfer_set` | `std_srvs/SetBool` | Cleaning safe-transfer |
| `/ros2_comm/cleaning/cleaning_finished_set` | `std_srvs/SetBool` | Cleaning finished |
| `/ros2_comm/cleaning/cleaning_active_set` | `std_srvs/SetBool` | Cleaning active |
| `/ros2_comm/cleaning/slide_command_set` | `std_srvs/SetBool` | Cleaning slide command |
| `/ros2_comm/cleaning/running_set` | `std_srvs/SetBool` | Cleaning running |
| `/ros2_comm/slider1/go_pos` | `std_srvs/SetBool` | Slider 1 harekete başla |
| `/ros2_comm/slider1/set_pos` | `backend/SetFloat32` | Slider 1 hedef konum ayarla |
| `/ros2_comm/slider2/go_pos` | `std_srvs/SetBool` | Slider 2 harekete başla |
| `/ros2_comm/slider2/set_pos` | `backend/SetFloat32` | Slider 2 hedef konum ayarla |

### Özel Servis Tanımları

```
# srv/SetInt16.srv          # srv/SetFloat32.srv
int16 data                  float32 data
---                         ---
bool success                bool success
string message              string message
```

---

## OPC UA Adres Alanı

Test sunucusu ve gerçek PLC aynı node ağacını `ns=3` altında sunar:

```
Objects/
└── ROS2_COMM
    ├── STATUS                                    Int16
    ├── MODE                                      Int16
    ├── COMMAND                                   Int16
    ├── SPEED                                     Int16
    ├── GO_TO_POS_1                               Double
    ├── GO_TO_POS_2                               Double
    │
    ├── MOD/
    │   ├── STARTUP                               Bool
    │   ├── CALIBRATION                           Bool
    │   ├── LEARNING                              Bool
    │   ├── MAINTENANCE                           Bool
    │   ├── EMERGENCY                             Bool
    │   ├── COBOT                                 Bool
    │   ├── FULLY_AUTOMATIC                       Bool
    │   ├── SHUTDOWN_MODE                         Bool
    │   └── Workcell_Status/
    │       ├── Slider_1_actual position-linear   Double
    │       └── Slider_2_actual position-linear   Double
    │
    └── STAT/
        ├── STARTUP                               Bool
        ├── CALIBRATION                           Bool
        ├── LEARNING                              Bool
        ├── MAINTENANCE                           Bool
        ├── EMERGENCY                             Bool
        ├── COBOT                                 Bool
        ├── Robot_Sensing_Status/
        │   ├── robothome_safetransfer            Bool
        │   ├── sensing-finised                   Bool
        │   ├── touchsensing-finished             Bool
        │   ├── sensing-active                    Bool
        │   ├── touchsensing-active               Bool
        │   ├── slide command                     Bool
        │   └── running                           Bool
        └── Robot_Cleaning_Status/
            ├── robothome_safetransfer            Bool
            ├── cleaning-finished                 Bool
            ├── cleaning-active                   Bool
            ├── slide command                     Bool
            └── running                           Bool
```

---

## Kod Yapısı

```
magician_ws/src/
│
├── backend/                           # OPC UA ↔ ROS 2 köprü paketi
│   ├── include/backend/
│   │   ├── config.hpp                 # UaConfig, TimingCfg, NodesCfg yapıları
│   │   ├── opcua_client.hpp           # UaClient sınıfı (connect, subscribe, write)
│   │   ├── ros_bridge.hpp             # RosBridge sınıfı (publisher + service tanımları)
│   │   └── naming.hpp                 # OPC UA node-ID yol yardımcısı
│   ├── src/
│   │   ├── main.cpp                   # Giriş noktası (config parametresi, executor)
│   │   ├── opcua_client.cpp           # Worker döngüsü, reconnect, subscription yönetimi
│   │   ├── ros_bridge.cpp             # Publisher ve service kurulumu
│   │   └── config.cpp                 # YAML ayrıştırma (yaml-cpp)
│   ├── test/
│   │   └── test_server.cpp            # PLC simülasyon sunucusu
│   ├── srv/
│   │   ├── SetInt16.srv               # Özel int16 servis tanımı
│   │   └── SetFloat32.srv             # Özel float32 servis tanımı
│   ├── config/
│   │   ├── opcua.yaml                 # Üretim konfigürasyonu (gerçek PLC)
│   │   └── opcua_test.yaml            # Test konfigürasyonu (localhost)
│   └── launch/
│       ├── system.launch.py           # Üretim: opc_bridge + GUI
│       ├── test_system.launch.py      # Test: test_server + opc_bridge
│       └── full_test_system.launch.py # Test: test_server + opc_bridge + GUI
│
├── gui_app/                           # Qt5 operatör arayüzü paketi
│   ├── include/gui_app/
│   │   └── mainwindow.hpp             # MainWindow sınıfı (Qt5 + ROS 2)
│   ├── src/
│   │   ├── main.cpp                   # Qt Application + ROS 2 init
│   │   └── mainwindow.cpp            # UI oluşturma, servis çağrıları, topic abonelikleri
│   └── png/                           # Logo ve görsel varlıklar
│
├── demonstrator_tree/                 # BehaviorTree orkestrasyon paketi
│   ├── include/demonstrator_tree/
│   │   ├── behavior_node.hpp          # MagicianSubNode, MagicianClientNode, MagicianOpcUA
│   │   └── parameters_parser.hpp      # CobotConfig YAML ayrıştırıcı
│   ├── src/
│   │   ├── main.cpp                   # BT fabrika kurulumu + executor
│   │   ├── behavior_node.cpp          # BT düğüm implementasyonları
│   │   └── parameters_parser.cpp      # YAML → CobotConfig dönüşümü
│   ├── config/
│   │   ├── bt_tree.xml                # BehaviorTree XML tanımı
│   │   └── parameters.yaml            # Robot home pozisyonları ve servis isimleri
│   └── test/
│       └── service_server.py          # Mock homing servisi (test için)
│
├── README.md                          # ← Bu dosya
└── INSTALLATION.md                    # Kurulum kılavuzu
```

---

## Hata Ayıklama

```bash
# Topic ve servis listesi
ros2 topic list | grep ros2_comm
ros2 service list | grep ros2_comm

# Anlık değer izleme
ros2 topic echo /ros2_comm/speed
ros2 topic echo /ros2_comm/mod/cobot
ros2 topic echo /ros2_comm/sensing/home_st

# Manuel servis çağrıları
ros2 service call /ros2_comm/speed_set backend/srv/SetInt16 "{data: 500}"
ros2 service call /ros2_comm/mod/cobot_set std_srvs/srv/SetBool "{data: true}"
ros2 service call /ros2_comm/sensing/active_set std_srvs/srv/SetBool "{data: true}"
ros2 service call /ros2_comm/slider1/set_pos backend/srv/SetFloat32 "{data: 150.0}"
ros2 service call /ros2_comm/slider1/go_pos std_srvs/srv/SetBool "{data: true}"

# BehaviorTree için joint state gözlemi
ros2 topic echo /xbotcore/joint_states
```

---

## Gereksinimler

| Bileşen | Versiyon |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| C++ | 17 |
| open62541 | ≥ 1.4 |
| Qt5 | 5.x (qtbase5-dev) |
| BehaviorTree.CPP | ros-humble-behaviortree-cpp |
| yaml-cpp | libyaml-cpp-dev |

Detaylı kurulum adımları için bkz. [INSTALLATION.md](INSTALLATION.md)

---

## Geliştirici

frknbyrm05@gmail.com
