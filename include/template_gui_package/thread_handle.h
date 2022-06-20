#ifndef THREAD_HANDLE_H
#define THREAD_HANDLE_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include "/home/ilya/catkin_ws/src/khi_robot/khi_robot_control/include/khi_robot_driver.h"
#include <QString>
#include <QElapsedTimer>
#include <QTimer>
#include <QThread>
#include <string>
#include <QDebug>
#include <QKeyEvent>
#include <iostream>
#include <QScrollBar>
#include <QTcpSocket>

class thread_handle : public QObject
{
  Q_OBJECT
public:
  explicit thread_handle(QObject *parent = nullptr);
  ~thread_handle();

  void _set_variable(QString variable, double value);

  void _get_variable_value (QString variable);

signals:
  void emit_thread_loop();

public slots:
  void communicate_slot(QString variable, QString variable_2, double value, uint timer_msec);

  void spinOnce1();

  void check_sending(bool send);

  void write_to_txt();

  void _thread_loop();

private:
  QTimer *ros_timer2, *pila_timer;

  ros::ServiceClient client2;

  QElapsedTimer *elapsed_timer, *elapsed_timer_log;

  khi_robot_msgs::KhiRobotCmd srv2;

  QString *service_string2, *chopped;

  QString *recieved_value;

  ros::NodeHandlePtr nh2_;
};

#endif // THREAD_HANDLE_H
