#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "gui_app/mainwindow.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  

  QApplication app(argc, argv);

  // Ana pencereyi oluştur
  MainWindow w;
  w.show();

  // ROS2 executor'ı Qt event loop içinde çalıştırmak için timer kullan
  // Not: MainWindow içinde de timer var ama bu global bir yedek olabilir
  QTimer rosTimer;
  QObject::connect(&rosTimer, &QTimer::timeout, [](){
    rclcpp::spin_some(rclcpp::Node::make_shared("dummy_node"));
  });
  // rosTimer.start(100); // İsteğe bağlı, MainWindow içindeki yeterli

  // Qt event loop'u başlat
  int ret = app.exec();
  
  // Temizlik
  rclcpp::shutdown();
  return ret;
}