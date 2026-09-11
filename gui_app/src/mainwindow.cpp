#include "gui_app/mainwindow.hpp"
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QStatusBar>
#include <QScrollArea>
#include <QPixmap>
#include <QFrame>
#include <qpushbutton.h>
#include <ament_index_cpp/get_package_share_directory.hpp>


MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), slider_qos_(rclcpp::QoS(20)), common_qos_(rclcpp::QoS(20)), sensing_and_cleaning_qos_(rclcpp::QoS(20)) {
  // ── Global stylesheet ──────────────────────────────────────────────
  setStyleSheet(
    "QMainWindow { background-color: #1e2124; }"
    "QScrollArea { border: none; background-color: #1e2124; }"
    "QWidget { background-color: #1e2124; color: #c8cdd6; }"
    "QGroupBox {"
    "  font-size: 11px; font-weight: bold; letter-spacing: 1.5px;"
    "  color: #7a8494; border: 1px solid #32363e;"
    "  margin-top: 18px; padding: 12px 8px 8px 8px;"
    "  background-color: #23272b; border-radius: 2px; }"
    "QGroupBox::title {"
    "  subcontrol-origin: margin; left: 10px; top: 2px; padding: 0 4px; }"
    "QLabel { color: #8a909c; font-size: 12px; background: transparent; }"
    "QLineEdit {"
    "  background-color: #191b1f; border: 1px solid #32363e;"
    "  color: #c8cdd6; padding: 6px 10px; font-size: 12px; border-radius: 2px; }"
    "QLineEdit:focus { border-color: #5088b8; color: #dde2ea; }"
    "QStatusBar {"
    "  background-color: #191b1f; color: #7a8494; font-size: 11px;"
    "  border-top: 1px solid #32363e; }"
  );

  auto *scrollArea = new QScrollArea(this);
  auto *central = new QWidget();
  auto *mainLayout = new QVBoxLayout(central);
  mainLayout->setSpacing(0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  // ── Header bar ────────────────────────────────────────────────────
  auto *headerWidget = new QWidget(central);
  headerWidget->setFixedHeight(64);
  headerWidget->setStyleSheet(
    "QWidget { background-color: #191b1f; border-bottom: 1px solid #32363e; }"
    "QLabel { background: transparent; }"
  );
  auto *headerLayout = new QHBoxLayout(headerWidget);
  headerLayout->setContentsMargins(18, 0, 18, 0);
  headerLayout->setSpacing(14);
    std::string package_path = ament_index_cpp::get_package_share_directory("gui_app");
    QString image_path = QString::fromStdString(package_path + "/png/images.png");
    QPixmap logo(image_path);

  if (!logo.isNull()) {
    auto *logoLabel = new QLabel(headerWidget);
    logoLabel->setPixmap(logo.scaledToHeight(50, Qt::SmoothTransformation));
    headerLayout->addWidget(logoLabel);
  }

  auto *titleLabel = new QLabel("PLC CONTROL SYSTEM", headerWidget);
  titleLabel->setStyleSheet(
    "QLabel { color: #d0d6e0; font-size: 14px; font-weight: bold; letter-spacing: 2px; }");
  headerLayout->addWidget(titleLabel);
  headerLayout->addStretch();

  auto *subtitleLabel = new QLabel("ROS2  ·  OPC-UA BRIDGE", headerWidget);
  subtitleLabel->setStyleSheet(
    "QLabel { color: #5a6270; font-size: 11px; letter-spacing: 1.5px; }");
  headerLayout->addWidget(subtitleLabel);
  mainLayout->addWidget(headerWidget);

  // ── Main content: two-column layout ──────────────────────────────
  auto *contentWidget = new QWidget(central);
  auto *contentLayout = new QHBoxLayout(contentWidget);
  contentLayout->setSpacing(8);
  contentLayout->setContentsMargins(10, 8, 10, 8);

  // ── Left panel ────────────────────────────────────────────────────
  auto *leftPanel = new QWidget(contentWidget);
  leftPanel->setStyleSheet("QWidget { background-color: #161719; }");
  auto *leftLayout = new QVBoxLayout(leftPanel);
  leftLayout->setSpacing(8);
  leftLayout->setContentsMargins(0, 0, 0, 0);

 auto *slidersGroup = new QGroupBox("SLIDER POSITIONS", leftPanel);
auto *slidersLayout = new QGridLayout(slidersGroup);

slidersLayout->setSpacing(5);
slidersLayout->setColumnStretch(1, 1);

slider1Pos_ = new QLineEdit(slidersGroup);
slider1Pos_->setPlaceholderText("position");
slider1Pos_->setReadOnly(true);
slider1Pos_->setAlignment(Qt::AlignCenter);

slider2Pos_ = new QLineEdit(slidersGroup);
slider2Pos_->setPlaceholderText("position");
slider2Pos_->setReadOnly(true);
slider2Pos_->setAlignment(Qt::AlignCenter);

const QString positionStyle =
    "QLineEdit {"
    "  background-color: #1c293b;"
    "  color: #90bede;"
    "  border: 1px solid #2e4460;"
    "  padding: 6px 10px;"
    "  font-size: 12px;"
    "  font-weight: bold;"
    "  border-radius: 2px;"
    "}";

slider1Pos_->setStyleSheet(positionStyle);
slider2Pos_->setStyleSheet(positionStyle);

slidersLayout->addWidget(
    new QLabel("Sensing Slider", slidersGroup), 0, 0);

slidersLayout->addWidget(
    slider1Pos_, 0, 1);

slidersLayout->addWidget(
    new QLabel("Cleaning Slider", slidersGroup), 1, 0);

slidersLayout->addWidget(
    slider2Pos_, 1, 1);

leftLayout->addWidget(slidersGroup); 

  // System Mode
  auto *cobotGroup = new QGroupBox("SYSTEM MODE", leftPanel);
  auto *cobotLayout = new QVBoxLayout(cobotGroup);
  cobotLayout->setSpacing(4);
  btnCobotModeToggle_    = createToggleButton("COBOT MODE");
  btnAutomaticModeToggle_ = createToggleButton("FULL AUTOMATIC");
  cobotLayout->addWidget(btnCobotModeToggle_);
  cobotLayout->addWidget(btnAutomaticModeToggle_);
  leftLayout->addWidget(cobotGroup);
  leftLayout->addStretch();
  contentLayout->addWidget(leftPanel, 4);

  // ── Right panel ───────────────────────────────────────────────────
  auto *rightPanel = new QWidget(contentWidget);
  rightPanel->setStyleSheet("QWidget { background-color: #161719; }");
  auto *rightLayout = new QVBoxLayout(rightPanel);
  rightLayout->setSpacing(8);
  rightLayout->setContentsMargins(0, 0, 0, 0);

  // Sensing Robot
  auto *sensingGroup = new QGroupBox("SENSING ROBOT", rightPanel);
  auto *sensingLayout = new QGridLayout(sensingGroup);
  sensingLayout->setSpacing(4);
  btnSensingCarbodyLocatedSt_    = createToggleButton("Carbody Located");
  btnSensingSafeTransferToggle_  = createToggleButton("Robot Home");
  btnSensingFinishedToggle_      = createToggleButton("Sensing Finished");
  btnSensingTouchFinishedToggle_ = createToggleButton("Touch Finished");
  btnSensingActiveToggle_        = createToggleButton("Sensing Active");
  btnSensingTouchActiveToggle_   = createToggleButton("Touch Active");
  btnSensingRunningToggle_       = createToggleButton("Running");
  btnSensingPos2Toggle_          = createToggleButton("Position 2"); 
  btnSensingPos3Toggle_          = createToggleButton("Position 3");
  btnSensingPos4Toggle_          = createToggleButton("Position 4");
  btnSensingPos5Toggle_          = createToggleButton("Position 5");
  sensingLayout->addWidget(btnSensingSafeTransferToggle_,    0, 0);
  sensingLayout->addWidget(btnSensingFinishedToggle_,        0, 1);
  sensingLayout->addWidget(btnSensingTouchFinishedToggle_,   1, 0);
  sensingLayout->addWidget(btnSensingActiveToggle_,          1, 1);
  sensingLayout->addWidget(btnSensingTouchActiveToggle_,     2, 0); 
  sensingLayout->addWidget(btnSensingCarbodyLocatedSt_,      2, 1);
  sensingLayout->addWidget(btnSensingRunningToggle_,         3, 0);
  sensingLayout->addWidget(btnSensingPos2Toggle_,            4, 0);
  sensingLayout->addWidget(btnSensingPos3Toggle_,            4, 1);
  sensingLayout->addWidget(btnSensingPos4Toggle_,            5, 0);
  sensingLayout->addWidget(btnSensingPos5Toggle_,            5, 1);
  rightLayout->addWidget(sensingGroup);

  // Cleaning Robot
  auto *cleaningGroup = new QGroupBox("CLEANING ROBOT", rightPanel);
  auto *cleaningLayout = new QGridLayout(cleaningGroup);
  cleaningLayout->setSpacing(4);
  btnCleaningCarbodyLocatedSt_   = createToggleButton("Carbody Located");
  btnCleaningSafeTransferToggle_ = createToggleButton("Robot Home");
  btnCleaningFinishedToggle_     = createToggleButton("Cleaning Finished");
  btnCleaningActiveToggle_       = createToggleButton("Cleaning Active");
  btnCleaningRunningToggle_      = createToggleButton("Running");
  btnCleaningPos2Toggle_          = createToggleButton("Position 2"); 
  btnCleaningPos3Toggle_          = createToggleButton("Position 3");
  btnCleaningPos4Toggle_          = createToggleButton("Position 4");
  btnCleaningPos5Toggle_          = createToggleButton("Position 5");
  cleaningLayout->addWidget(btnCleaningSafeTransferToggle_,  0, 0);
  cleaningLayout->addWidget(btnCleaningFinishedToggle_,      0, 1);
  cleaningLayout->addWidget(btnCleaningActiveToggle_,        1, 0);
  cleaningLayout->addWidget(btnCleaningCarbodyLocatedSt_,    1, 1);
  cleaningLayout->addWidget(btnCleaningRunningToggle_,       2, 0);
  cleaningLayout->addWidget(btnCleaningPos2Toggle_,          3, 0); 
  cleaningLayout->addWidget(btnCleaningPos3Toggle_,          3, 1);
  cleaningLayout->addWidget(btnCleaningPos4Toggle_,          4, 0);
  cleaningLayout->addWidget(btnCleaningPos5Toggle_,          4, 1);
  rightLayout->addWidget(cleaningGroup);
  rightLayout->addStretch();
  contentLayout->addWidget(rightPanel, 6);

  mainLayout->addWidget(contentWidget, 1);

  scrollArea->setWidget(central);
  scrollArea->setWidgetResizable(true);
  setCentralWidget(scrollArea);
  setWindowTitle("Magician — PLC Control");
  resize(1080, 680);

  setup_ros();


/* SENSING  */

  connect(btnCobotModeToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCobotModeToggle_, checked);
    call_service(cli_mod_cobot_, checked);
  });
 
  connect(btnAutomaticModeToggle_, &QPushButton::toggled, [this](bool checked){
    updateToggleButtonStyle(btnAutomaticModeToggle_,checked);
    call_service(cli_mod_automatic_, checked);
  });

  connect(btnSensingCarbodyLocatedSt_,&QPushButton::toggled,[this](bool checked){
    updateToggleButtonStyle(btnSensingCarbodyLocatedSt_, checked);
    call_service(cli_sensing_carbody_located_st_,checked);        
  });

  connect(btnSensingSafeTransferToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingSafeTransferToggle_, checked);
    call_service(cli_sensing_safetransfer_, checked);
  });
  
  connect(btnSensingFinishedToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingFinishedToggle_, checked);
    call_service(cli_sensing_finished_, checked);
  });
  
  connect(btnSensingTouchFinishedToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingTouchFinishedToggle_, checked);
    call_service(cli_sensing_touch_finished_, checked);
  });
  
  connect(btnSensingActiveToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingActiveToggle_, checked);
    call_service(cli_sensing_active_, checked);
  });
  
  connect(btnSensingTouchActiveToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingTouchActiveToggle_, checked);
    call_service(cli_sensing_touch_active_, checked);
  });
    
  connect(btnSensingRunningToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingRunningToggle_, checked);
    call_service(cli_sensing_running_, checked);
  });
 

  connect(btnSensingPos2Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingPos2Toggle_, checked);
    call_service(cli_sensing_pos2_st_, checked);
  });


  connect(btnSensingPos3Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingPos3Toggle_, checked);
    call_service(cli_sensing_pos3_st_, checked);
  });


  connect(btnSensingPos4Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingPos4Toggle_, checked);
    call_service(cli_sensing_pos4_st_, checked);
  });


  connect(btnSensingPos5Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingPos5Toggle_, checked);
    call_service(cli_sensing_pos5_st_, checked);
  });

/* CLEANING  */

  connect(btnCleaningCarbodyLocatedSt_, &QPushButton::toggled, [this](bool checked){
    updateToggleButtonStyle(btnCleaningCarbodyLocatedSt_, checked);
    call_service(cli_cleaning_carbody_located_st_, checked);
  });
  connect(btnCleaningSafeTransferToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningSafeTransferToggle_, checked);
    call_service(cli_cleaning_safetransfer_, checked);
  });
  
  connect(btnCleaningFinishedToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningFinishedToggle_, checked);
    call_service(cli_cleaning_finished_, checked);
  });
  
  connect(btnCleaningActiveToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningActiveToggle_, checked);
    call_service(cli_cleaning_active_, checked);
  });
   
  connect(btnCleaningRunningToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningRunningToggle_, checked);
    call_service(cli_cleaning_running_, checked);
  });


  connect(btnCleaningPos2Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningPos2Toggle_, checked);
    call_service(cli_cleaning_pos2_st_, checked);
  });


  connect(btnCleaningPos3Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningPos3Toggle_, checked);
    call_service(cli_cleaning_pos3_st_, checked);
  });


  connect(btnCleaningPos4Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningPos4Toggle_, checked);
    call_service(cli_cleaning_pos4_st_, checked);
  });

  connect(btnCleaningPos5Toggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningPos5Toggle_, checked);
    call_service(cli_cleaning_pos5_st_, checked);
  });


}

MainWindow::~MainWindow() {}

QPushButton* MainWindow::createToggleButton(const QString& label) {
  auto *btn = new QPushButton(label + ":  OFF", this);
  btn->setCheckable(true);
  btn->setChecked(false);
  btn->setMinimumHeight(34);
  updateToggleButtonStyle(btn, false);
  return btn;
}

void MainWindow::updateToggleButtonStyle(QPushButton* btn, bool state) {
  QString baseLabel = btn->text().left(btn->text().lastIndexOf(":"));
  if (state) {
    btn->setText(baseLabel + ":  ON");
    btn->setStyleSheet(
      "QPushButton { background-color: #1a3028; color: #5ed68e; font-weight: bold;"
      "  border: 1px solid #2a4e3a; border-left: 3px solid #3ec06a;"
      "  font-size: 12px; padding: 7px 12px; border-radius: 2px; text-align: left; }"
      "QPushButton:hover { background-color: #1e3a30; color: #78e8a4; }"
    );
  } else {
    btn->setText(baseLabel + ":  OFF");
    btn->setStyleSheet(
      "QPushButton { background-color: #23272b; color: #7a8494; font-weight: bold;"
      "  border: 1px solid #32363e; border-left: 3px solid #3e4450;"
      "  font-size: 12px; padding: 7px 12px; border-radius: 2px; text-align: left; }"
      "QPushButton:hover { background-color: #292e34; color: #9aa2b0; }"
    );
  }
}
void MainWindow::setup_ros() {
  node_ = std::make_shared<rclcpp::Node>("gui_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  
  cli_mod_cobot_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/mod/cobot_mode_set");
  cli_mod_automatic_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/mod/full_automatic_mode_set");

  cli_sensing_carbody_located_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/carbody_located_set");
  cli_sensing_safetransfer_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/safetransfer_set");
  cli_sensing_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/finished_set");
  cli_sensing_touch_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_finished_set");
  cli_sensing_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/active_set");
  cli_sensing_touch_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_active_set");
  cli_sensing_running_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/running");

 cli_sensing_pos2_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/pos2_set"); 
 cli_sensing_pos3_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/pos3_set");
 cli_sensing_pos4_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/pos4_set");
 cli_sensing_pos5_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/pos5_set");






  cli_cleaning_carbody_located_st_= node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/carbody_located_set");
  cli_cleaning_safetransfer_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/safetransfer_set");
  cli_cleaning_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_finished_set");
  cli_cleaning_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_active_set");
  cli_cleaning_running_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/running_set");

  cli_cleaning_pos2_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/pos2_set");
  cli_cleaning_pos3_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/pos3_set");
  cli_cleaning_pos4_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/pos4_set");
  cli_cleaning_pos5_st_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/pos5_set");
  
  sub_cobot_mode_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/mod/cobot", common_qos_.reliable(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCobotModeToggle_->blockSignals(true);
        btnCobotModeToggle_->setChecked(state);
        updateToggleButtonStyle(btnCobotModeToggle_, state);
        btnCobotModeToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  sub_automatic_mode_ = node_->create_subscription<std_msgs::msg::Bool>("/ros2_comm/mod/automatic",common_qos_.reliable(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this,[this,state=msg->data](){
        btnAutomaticModeToggle_->blockSignals(true);
        btnAutomaticModeToggle_->setChecked(state);
        updateToggleButtonStyle(btnAutomaticModeToggle_,state);
        btnAutomaticModeToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
 
    
  sub_sensing_carbody_located_st_ = node_->create_subscription<std_msgs::msg::Bool>("/ros2_comm/sensing/carbody_located_status",sensing_and_cleaning_qos_.best_effort(),
          [this](const std_msgs::msg::Bool::SharedPtr msg){
          QMetaObject::invokeMethod(this,[this,state=msg->data](){
           btnSensingCarbodyLocatedSt_->blockSignals(true);
           btnSensingCarbodyLocatedSt_->setChecked(state);
           updateToggleButtonStyle(btnSensingCarbodyLocatedSt_,state);
           btnSensingCarbodyLocatedSt_->blockSignals(false);
           },Qt::QueuedConnection);
          });

  sub_sensing_safetransfer_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/home_st", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingSafeTransferToggle_->blockSignals(true);
        btnSensingSafeTransferToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingSafeTransferToggle_, state);
        btnSensingSafeTransferToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/finished", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingFinishedToggle_->blockSignals(true);
        btnSensingFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingFinishedToggle_, state);
        btnSensingFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_touch_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/touch_finished", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingTouchFinishedToggle_->blockSignals(true);
        btnSensingTouchFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingTouchFinishedToggle_, state);
        btnSensingTouchFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/sensing_active", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingActiveToggle_->blockSignals(true);
        btnSensingActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingActiveToggle_, state);
        btnSensingActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_touch_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/touch_active", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingTouchActiveToggle_->blockSignals(true);
        btnSensingTouchActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingTouchActiveToggle_, state);
        btnSensingTouchActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  
  sub_sensing_running_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/running",sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingRunningToggle_->blockSignals(true);
        btnSensingRunningToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingRunningToggle_, state);
        btnSensingRunningToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

    
    sub_sensing_pos2_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/pos2_status",sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingPos2Toggle_->blockSignals(true);
        btnSensingPos2Toggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingPos2Toggle_, state);
        btnSensingPos2Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

    

    sub_sensing_pos3_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/pos3_status",sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingPos3Toggle_->blockSignals(true);
        btnSensingPos3Toggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingPos3Toggle_, state);
        btnSensingPos3Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

    
    sub_sensing_pos4_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/pos4_status",sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingPos4Toggle_->blockSignals(true);
        btnSensingPos4Toggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingPos4Toggle_, state);
        btnSensingPos4Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
    


    sub_sensing_pos5_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/pos5_status",sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingPos5Toggle_->blockSignals(true);
        btnSensingPos5Toggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingPos5Toggle_, state);
        btnSensingPos5Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

 
sub_sensing_slider_ =
    node_->create_subscription<std_msgs::msg::Float32>(
        "/ros2_comm/sensing/slider_actual_pos",
        slider_qos_.best_effort(),
        [this](const std_msgs::msg::Float32::SharedPtr msg)
        {
            QMetaObject::invokeMethod(
                this,
                [this, position = msg->data]()
                {
                    slider1Pos_->setText(
                        QString::number(position, 'f', 2)
                    );
                },
                Qt::QueuedConnection);
        });

sub_cleaning_carbody_located_st_ = node_->create_subscription<std_msgs::msg::Bool>("/ros2_comm/cleaning/carbody_located_status",sensing_and_cleaning_qos_.best_effort(),
        [this](const std_msgs::msg::Bool::SharedPtr msg){
        QMetaObject::invokeMethod(this,[this,state=msg->data](){
                btnCleaningCarbodyLocatedSt_->blockSignals(true);
                btnCleaningCarbodyLocatedSt_->setChecked(state);
                updateToggleButtonStyle(btnCleaningCarbodyLocatedSt_,state);
                btnCleaningCarbodyLocatedSt_->blockSignals(false);
                },Qt::QueuedConnection);
        });

sub_cleaning_slider_ =
    node_->create_subscription<std_msgs::msg::Float32>(
        "/ros2_comm/cleaning/slider_actual_pos",
        slider_qos_.best_effort(),
        [this](const std_msgs::msg::Float32::SharedPtr msg)
        {
            QMetaObject::invokeMethod(
                this,
                [this, position = msg->data]()
                {
                    slider2Pos_->setText(
                        QString::number(position, 'f', 2)
                    );
                },
                Qt::QueuedConnection);
        });





  sub_cleaning_safetransfer_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/home_st", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningSafeTransferToggle_->blockSignals(true);
        btnCleaningSafeTransferToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningSafeTransferToggle_, state);
        btnCleaningSafeTransferToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/finished", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningFinishedToggle_->blockSignals(true);
        btnCleaningFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningFinishedToggle_, state);
        btnCleaningFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/cleaning_active", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningActiveToggle_->blockSignals(true);
        btnCleaningActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningActiveToggle_, state);
        btnCleaningActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  
  sub_cleaning_running_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/running", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningRunningToggle_->blockSignals(true);
        btnCleaningRunningToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningRunningToggle_, state);
        btnCleaningRunningToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

    sub_cleaning_pos2_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/pos2_status", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningPos2Toggle_->blockSignals(true);
        btnCleaningPos2Toggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningPos2Toggle_, state);
        btnCleaningPos2Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

    
    sub_cleaning_pos3_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/pos3_status", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningPos3Toggle_->blockSignals(true);
        btnCleaningPos3Toggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningPos3Toggle_, state);
        btnCleaningPos3Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });


    sub_cleaning_pos4_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/pos4_status", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningPos4Toggle_->blockSignals(true);
        btnCleaningPos4Toggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningPos4Toggle_, state);
        btnCleaningPos4Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });


    sub_cleaning_pos5_st_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/pos5_status", sensing_and_cleaning_qos_.best_effort(),
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningPos5Toggle_->blockSignals(true);
        btnCleaningPos5Toggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningPos5Toggle_, state);
        btnCleaningPos5Toggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });


  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, [this](){
    executor_->spin_some();
  });
  ros_timer_->start(20); // 50Hz
}

void MainWindow::call_service(rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client, bool value) {
  if(!client->wait_for_service(std::chrono::seconds(1))) {
    statusBar()->showMessage(" Service not available!", 2000);
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = value;
  client->async_send_request(req);
}
