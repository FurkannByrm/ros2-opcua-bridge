#include <QApplication>
#include "gui_app/mainwindow.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  

  QApplication app(argc, argv);

  MainWindow w;
  w.show();

  int ret = app.exec();
  
  rclcpp::shutdown();
  return ret;
}
