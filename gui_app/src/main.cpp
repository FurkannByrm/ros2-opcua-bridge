#include <QApplication>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "gui_app/mainwindow.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  

  QApplication app(argc, argv);

  MainWindow w;
  w.show();

  QTimer rosTimer;
  QObject::connect(&rosTimer, &QTimer::timeout, [](){
    rclcpp::spin_some(rclcpp::Node::make_shared("dummy_node"));
  });


  int ret = app.exec();
  
  rclcpp::shutdown();
  return ret;
}