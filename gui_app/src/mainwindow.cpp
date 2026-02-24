#include "gui_app/mainwindow.hpp"
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QStatusBar>
#include <QScrollArea>
#include <QPixmap>
#include <QFrame>

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
  auto *scrollArea = new QScrollArea(this);
  auto *central = new QWidget();
  auto *mainLayout = new QVBoxLayout(central);
  mainLayout->setSpacing(10);
  mainLayout->setContentsMargins(15, 15, 15, 15);
  
  auto *headerFrame = new QFrame(central);
  headerFrame->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
  headerFrame->setStyleSheet("QFrame { background-color: #2c3e50; border-radius: 10px; padding: 10px; }");
  auto *headerLayout = new QVBoxLayout(headerFrame);
  
  auto *logoLabel = new QLabel(headerFrame);
  QPixmap logo("/home/furkan/magician_ws/src/gui_app/png/magician_logo_full.png"); // -->  CHANGE IT !!
  if (!logo.isNull()) {
    logoLabel->setPixmap(logo.scaledToHeight(150, Qt::SmoothTransformation));
    logoLabel->setAlignment(Qt::AlignCenter);
    headerLayout->addWidget(logoLabel);
  } else {
    auto *placeholderLabel = new QLabel("MAGICIAN", headerFrame);
    placeholderLabel->setAlignment(Qt::AlignCenter);
    placeholderLabel->setStyleSheet("QLabel { color: white; font-size: 48px; font-weight: bold; }");
    headerLayout->addWidget(placeholderLabel);
  }
  
  auto *titleLabel = new QLabel("PLC Control System", headerFrame);
  titleLabel->setAlignment(Qt::AlignCenter);
  titleLabel->setStyleSheet("QLabel { color: white; font-size: 24px; font-weight: bold; margin: 10px; }");
  headerLayout->addWidget(titleLabel);
  
  auto *subtitleLabel = new QLabel("ROS2 OPC-UA Bridge Interface", headerFrame);
  subtitleLabel->setAlignment(Qt::AlignCenter);
  subtitleLabel->setStyleSheet("QLabel { color: #ecf0f1; font-size: 14px; margin-bottom: 10px; }");
  headerLayout->addWidget(subtitleLabel);
  
  mainLayout->addWidget(headerFrame);
  
  // Speed Control Group
  auto *speedGroup = new QGroupBox("⚡ Speed Control", central);
  speedGroup->setStyleSheet(
    "QGroupBox { font-weight: bold; font-size: 14px; border: 2px solid #3498db; "
    "border-radius: 8px; margin-top: 10px; padding-top: 15px; background-color: #ecf0f1; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
  );
  auto *speedLayout = new QHBoxLayout(speedGroup);
  auto *speedEdit = new QLineEdit(speedGroup);
  speedEdit->setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; font-size: 12px; }");
  auto *btnSpeedSet = new QPushButton("Set Speed", speedGroup);
  btnSpeedSet->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; padding: 8px 15px; "
    "border-radius: 5px; font-weight: bold; }"
    "QPushButton:hover { background-color: #2980b9; }"
    "QPushButton:pressed { background-color: #1c638e; }"
  );
  speedEdit->setPlaceholderText("Enter speed value");
  speedLayout->addWidget(new QLabel("Speed:", speedGroup));
  speedLayout->addWidget(speedEdit);
  speedLayout->addWidget(btnSpeedSet);
  mainLayout->addWidget(speedGroup);
  



  auto *slider1Group = new QGroupBox("slider1 Control", central);
  slider1Group->setStyleSheet(
    "QGroupBox { font-weight: bold; font-size: 14px; border: 2px solid #3498db; "
    "border-radius: 8px; margin-top: 10px; padding-top: 15px; background-color: #ecf0f1; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
  );
  auto *slider1Layout = new QHBoxLayout(slider1Group);
  auto *slider1Edit = new QLineEdit(slider1Group);
  slider1Edit->setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; font-size: 12px; }");
  auto *btnslider1Set = new QPushButton("Set Position", slider1Group);
  btnslider1Set->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; padding: 8px 15px; "
    "border-radius: 5px; font-weight: bold; }"
    "QPushButton:hover { background-color: #2980b9; }"
    "QPushButton:pressed { background-color: #1c638e; }"
  );
  auto *btnslider1Go = new QPushButton("Go Position", slider1Group);
  btnslider1Go->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; padding: 8px 15px; "
    "border-radius: 5px; font-weight: bold; }"
    "QPushButton:hover { background-color: #2980b9; }"
    "QPushButton:pressed { background-color: #1c638e; }"
  );
  slider1Edit->setPlaceholderText("Enter target positon");
  slider1Layout->addWidget(new QLabel("Slider1:", slider1Group));
  slider1Layout->addWidget(slider1Edit);
  slider1Layout->addWidget(btnslider1Set);
  slider1Layout->addWidget(btnslider1Go);
  mainLayout->addWidget(slider1Group);


  auto *slider2Group = new QGroupBox("slider2 Control", central);
  slider2Group->setStyleSheet(
    "QGroupBox { font-weight: bold; font-size: 14px; border: 2px solid #3498db; "
    "border-radius: 8px; margin-top: 10px; padding-top: 15px; background-color: #ecf0f1; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
  );
  auto *slider2Layout = new QHBoxLayout(slider2Group);
  auto *slider2Edit = new QLineEdit(slider2Group);
  slider2Edit->setStyleSheet("QLineEdit { padding: 8px; border: 2px solid #bdc3c7; border-radius: 5px; font-size: 12px; }");
  auto *btnslider2Set = new QPushButton("Set Position", slider2Group);
  btnslider2Set->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; padding: 8px 15px; "
    "border-radius: 5px; font-weight: bold; }"
    "QPushButton:hover { background-color: #2980b9; }"
    "QPushButton:pressed { background-color: #1c638e; }"
  );
  auto *btnslider2Go = new QPushButton("Go Position", slider2Group);
  btnslider2Go->setStyleSheet(
    "QPushButton { background-color: #3498db; color: white; padding: 8px 15px; "
    "border-radius: 5px; font-weight: bold; }"
    "QPushButton:hover { background-color: #2980b9; }"
    "QPushButton:pressed { background-color: #1c638e; }"
  );
  slider2Edit->setPlaceholderText("Enter target position");
  slider2Layout->addWidget(new QLabel("Slider2:", slider2Group));
  slider2Layout->addWidget(slider2Edit);
  slider2Layout->addWidget(btnslider2Set);
  slider2Layout->addWidget(btnslider2Go);
  mainLayout->addWidget(slider2Group);




  auto *cobotGroup = new QGroupBox("🤖 COBOT Control", central);
  cobotGroup->setStyleSheet(
    "QGroupBox { font-weight: bold; font-size: 14px; border: 2px solid #9b59b6; "
    "border-radius: 8px; margin-top: 10px; padding-top: 15px; background-color: #ecf0f1; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
  );
  auto *cobotLayout = new QVBoxLayout(cobotGroup);
  btnCobotToggle_ = createToggleButton("COBOT");
  cobotLayout->addWidget(btnCobotToggle_);
  mainLayout->addWidget(cobotGroup);
  
  auto *sensingGroup = new QGroupBox("🔍 Sensing Robot Controls", central);
  sensingGroup->setStyleSheet(
    "QGroupBox { font-weight: bold; font-size: 14px; border: 2px solid #16a085; "
    "border-radius: 8px; margin-top: 10px; padding-top: 15px; background-color: #ecf0f1; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
  );
  auto *sensingLayout = new QVBoxLayout(sensingGroup);
  
  btnSensingSafeTransferToggle_ = createToggleButton("Safe Transfer");
  btnSensingFinishedToggle_ = createToggleButton("Sensing Finished");
  btnSensingTouchFinishedToggle_ = createToggleButton("Touch Sensing Finished");
  btnSensingActiveToggle_ = createToggleButton("Sensing Active");
  btnSensingTouchActiveToggle_ = createToggleButton("Touch Sensing Active");
  btnSensingSlideCommandToggle_ = createToggleButton("Slide Command");
  btnSensingRunningToggle_ = createToggleButton("Running");
  
  sensingLayout->addWidget(btnSensingSafeTransferToggle_);
  sensingLayout->addWidget(btnSensingFinishedToggle_);
  sensingLayout->addWidget(btnSensingTouchFinishedToggle_);
  sensingLayout->addWidget(btnSensingActiveToggle_);
  sensingLayout->addWidget(btnSensingTouchActiveToggle_);
  sensingLayout->addWidget(btnSensingSlideCommandToggle_);
  sensingLayout->addWidget(btnSensingRunningToggle_);
  mainLayout->addWidget(sensingGroup);
  
  auto *cleaningGroup = new QGroupBox("🧹 Cleaning Robot Controls", central);
  cleaningGroup->setStyleSheet(
    "QGroupBox { font-weight: bold; font-size: 14px; border: 2px solid #e67e22; "
    "border-radius: 8px; margin-top: 10px; padding-top: 15px; background-color: #ecf0f1; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }"
  );
  auto *cleaningLayout = new QVBoxLayout(cleaningGroup);
  
  btnCleaningSafeTransferToggle_ = createToggleButton("Safe Transfer");
  btnCleaningFinishedToggle_ = createToggleButton("Cleaning Finished");
  btnCleaningActiveToggle_ = createToggleButton("Cleaning Active");
  btnCleaningSlideCommandToggle_ = createToggleButton("Slide Command");
  btnCleaningRunningToggle_ = createToggleButton("Running");
  
  cleaningLayout->addWidget(btnCleaningSafeTransferToggle_);
  cleaningLayout->addWidget(btnCleaningFinishedToggle_);
  cleaningLayout->addWidget(btnCleaningActiveToggle_);
  cleaningLayout->addWidget(btnCleaningSlideCommandToggle_);
  cleaningLayout->addWidget(btnCleaningRunningToggle_);
  mainLayout->addWidget(cleaningGroup);
  
  auto *footerLabel = new QLabel("© 2025 Magician - Industrial Automation Solutions", central);
  footerLabel->setAlignment(Qt::AlignCenter);
  footerLabel->setStyleSheet("QLabel { color: #7f8c8d; font-size: 10px; margin-top: 10px; }");
  mainLayout->addWidget(footerLabel);
  
  scrollArea->setWidget(central);
  scrollArea->setWidgetResizable(true);
  scrollArea->setStyleSheet("QScrollArea { border: none; background-color: #bdc3c7; }");
  setCentralWidget(scrollArea);
  setWindowTitle("Magician PLC Control System");
  resize(600, 900);
  
  setStyleSheet("QMainWindow { background-color: #bdc3c7; }");

  setup_ros();

  connect(btnSpeedSet, &QPushButton::clicked, [this, speedEdit]{
    bool ok=false; int v = speedEdit->text().toInt(&ok);
    if(ok) call_speed_set(v);
  });

  connect(btnslider1Set, &QPushButton::clicked, [this, slider1Edit]{
    bool ok=false; float v = slider1Edit->text().toFloat(&ok);
    if(ok) call_slider_set(cli_slider1_set_pos_,v);
  });
  
  connect(btnslider2Set, &QPushButton::clicked, [this, slider2Edit]{
      bool ok=false; float v = slider2Edit->text().toFloat(&ok);
      if(ok) call_slider_set(cli_slider2_set_pos_,v);
    });

  connect(btnslider1Go, &QPushButton::clicked, [this](bool checked){ 
    call_service(cli_slider1_go_pos_, checked);
  });

  connect(btnslider2Go, &QPushButton::clicked, [this](bool checked){ 
    call_service(cli_slider2_go_pos_, checked);
  });
  connect(btnCobotToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCobotToggle_, checked);
    call_service(cli_cobot_, checked);
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
  
  connect(btnSensingSlideCommandToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingSlideCommandToggle_, checked);
    call_service(cli_sensing_slide_command_, checked);
  });
  
  connect(btnSensingRunningToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnSensingRunningToggle_, checked);
    call_service(cli_sensing_running_, checked);
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
  
  connect(btnCleaningSlideCommandToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningSlideCommandToggle_, checked);
    call_service(cli_cleaning_slide_command_, checked);
  });
  
  connect(btnCleaningRunningToggle_, &QPushButton::toggled, [this](bool checked){ 
    updateToggleButtonStyle(btnCleaningRunningToggle_, checked);
    call_service(cli_cleaning_running_, checked);
  });
}

MainWindow::~MainWindow() {}

QPushButton* MainWindow::createToggleButton(const QString& label) {
  auto *btn = new QPushButton(label + ": OFF", this);
  btn->setCheckable(true);
  btn->setChecked(false);
  btn->setMinimumHeight(45);
  updateToggleButtonStyle(btn, false);
  return btn;
}

void MainWindow::updateToggleButtonStyle(QPushButton* btn, bool state) {
  QString baseLabel = btn->text().left(btn->text().lastIndexOf(":"));
  if (state) {
    btn->setText(baseLabel + ": ON");
    btn->setStyleSheet(
      "QPushButton { background-color: #27ae60; color: white; font-weight: bold; "
      "border-radius: 8px; font-size: 13px; padding: 10px; border: 2px solid #229954; }"
      "QPushButton:hover { background-color: #229954; }"
    );
  } else {
    btn->setText(baseLabel + ": OFF");
    btn->setStyleSheet(
      "QPushButton { background-color: #e74c3c; color: white; font-weight: bold; "
      "border-radius: 8px; font-size: 13px; padding: 10px; border: 2px solid #c0392b; }"
      "QPushButton:hover { background-color: #c0392b; }"
    );
  }
}
void MainWindow::setup_ros() {
  node_ = std::make_shared<rclcpp::Node>("gui_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);
  
  cli_speed_ = node_->create_client<backend::srv::SetInt16>("/ros2_comm/speed_set");
  cli_cobot_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/mod/cobot_set");
  
  cli_sensing_safetransfer_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/safetransfer_set");
  cli_sensing_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/finished_set");
  cli_sensing_touch_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_finished_set");
  cli_sensing_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/active_set");
  cli_sensing_touch_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/touch_active_set");
  cli_sensing_slide_command_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/slide_command_set");
  cli_sensing_running_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/sensing/running");
  
  cli_cleaning_safetransfer_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/safetransfer_set");
  cli_cleaning_finished_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_finished_set");
  cli_cleaning_active_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/cleaning_active_set");
  cli_cleaning_slide_command_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/slide_command_set");
  cli_cleaning_running_ = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/cleaning/running_set");

  cli_slider1_set_pos_ = node_->create_client<backend::srv::SetFloat32>("/ros2_comm/slider1/set_pos");
  cli_slider2_set_pos_ = node_->create_client<backend::srv::SetFloat32>("/ros2_comm/slider2/set_pos");
  cli_slider1_go_pos_  = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/slider1/go_pos");
  cli_slider2_go_pos_  = node_->create_client<std_srvs::srv::SetBool>("/ros2_comm/slider2/go_pos");



  sub_speed_ = node_->create_subscription<std_msgs::msg::Int16>(
    "/ros2_comm/speed", 10,
    [this](const std_msgs::msg::Int16::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, value=msg->data](){
        this->statusBar()->showMessage(QString("⚡ Current Speed: %1").arg(value), 3000);
        this->statusBar()->setStyleSheet("QStatusBar { background-color: #34495e; color: white; font-weight: bold; }");
      }, Qt::QueuedConnection);
    });
  
  sub_cobot_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/mod/cobot", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCobotToggle_->blockSignals(true);
        btnCobotToggle_->setChecked(state);
        updateToggleButtonStyle(btnCobotToggle_, state);
        btnCobotToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_safetransfer_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/home_st", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingSafeTransferToggle_->blockSignals(true);
        btnSensingSafeTransferToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingSafeTransferToggle_, state);
        btnSensingSafeTransferToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/finished", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingFinishedToggle_->blockSignals(true);
        btnSensingFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingFinishedToggle_, state);
        btnSensingFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_touch_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/touch_finished", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingTouchFinishedToggle_->blockSignals(true);
        btnSensingTouchFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingTouchFinishedToggle_, state);
        btnSensingTouchFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/sensing_active", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingActiveToggle_->blockSignals(true);
        btnSensingActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingActiveToggle_, state);
        btnSensingActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_touch_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/touch_active", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingTouchActiveToggle_->blockSignals(true);
        btnSensingTouchActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingTouchActiveToggle_, state);
        btnSensingTouchActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_slide_command_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/slide_command", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingSlideCommandToggle_->blockSignals(true);
        btnSensingSlideCommandToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingSlideCommandToggle_, state);
        btnSensingSlideCommandToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_sensing_running_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/sensing/running", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnSensingRunningToggle_->blockSignals(true);
        btnSensingRunningToggle_->setChecked(state);
        updateToggleButtonStyle(btnSensingRunningToggle_, state);
        btnSensingRunningToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_safetransfer_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/home_st", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningSafeTransferToggle_->blockSignals(true);
        btnCleaningSafeTransferToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningSafeTransferToggle_, state);
        btnCleaningSafeTransferToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_finished_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/finished", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningFinishedToggle_->blockSignals(true);
        btnCleaningFinishedToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningFinishedToggle_, state);
        btnCleaningFinishedToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_active_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/cleaning_active", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningActiveToggle_->blockSignals(true);
        btnCleaningActiveToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningActiveToggle_, state);
        btnCleaningActiveToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_slide_command_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/slide_command", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningSlideCommandToggle_->blockSignals(true);
        btnCleaningSlideCommandToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningSlideCommandToggle_, state);
        btnCleaningSlideCommandToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });
  
  sub_cleaning_running_ = node_->create_subscription<std_msgs::msg::Bool>(
    "/ros2_comm/cleaning/running", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg){
      QMetaObject::invokeMethod(this, [this, state=msg->data](){
        btnCleaningRunningToggle_->blockSignals(true);
        btnCleaningRunningToggle_->setChecked(state);
        updateToggleButtonStyle(btnCleaningRunningToggle_, state);
        btnCleaningRunningToggle_->blockSignals(false);
      }, Qt::QueuedConnection);
    });

  ros_timer_ = new QTimer(this);
  connect(ros_timer_, &QTimer::timeout, [this](){
    executor_->spin_some();
  });
  ros_timer_->start(20); // 50Hz
}
void MainWindow::call_speed_set(int value) {
  if(!cli_speed_->wait_for_service(std::chrono::seconds(1))) {
    statusBar()->showMessage("Speed service not available!", 3000);
    return;
  }
  auto req = std::make_shared<backend::srv::SetInt16::Request>();
  req->data = value;
  cli_speed_->async_send_request(req);
  statusBar()->showMessage(QString(" Speed set to: %1").arg(value), 3000);
}


void MainWindow::call_slider_set(rclcpp::Client<backend::srv::SetFloat32>::SharedPtr client, float value){
if(!client->wait_for_service(std::chrono::seconds(1))){
  statusBar()->showMessage("slider service not available!",3000);
  return;
}
  auto req = std::make_shared<backend::srv::SetFloat32::Request>();
  req->data = value;
  client->async_send_request(req);
  statusBar()->showMessage(QString("Slider set to %1").arg(value),3000);
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