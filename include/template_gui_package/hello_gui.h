/*
# MIT License

# Copyright (c) 2022 Kristopher Krasnosky

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
*/

#ifndef HELLO_GUI_H
#define HELLO_GUI_H

#include "thread_handle.h"

using namespace std;

namespace Ui {
class HelloGui;
}

class HelloGui : public QWidget
{
  Q_OBJECT

public:
  explicit HelloGui(QWidget *parent = nullptr);
  ~HelloGui();

  void service_call(uint start_signal, uint8_t number_of_bits, uint16_t number_to_send);

  void get_signal(uint start_signal);

  void set_signals(uint8_t number, uint8_t number_of_bits, uint16_t start_signal);

  void get_bits(uint16_t start_bit, uint8_t number_of_bits);

  void set_variable(QString variable, double value);

  void get_variable_value (QString variable);

  void do_smth(int a);

  void sendData(const QByteArray &ba);



  Ui::HelloGui *ui;

signals:
  void signal_service_ended();

  void emit_button(QString variable, QString variable_2, double value, uint timer);

  void emit_sending_signal(bool send);

  void write_to_txt_signal();


public slots:
  void spinOnce();

  void HandleStateChange(QAbstractSocket::SocketState socketState);

  void recieved_msg_handler();

private slots:
  //void keyPressEvent(QKeyEvent *);
  void service_call();

  void on_hi_button_clicked();

  void update_timer();

  void on_pushButton_clicked();

  void on_pushButton_3_clicked();

  void on_pushButton_2_clicked();

  void on_pushButton_4_clicked();

  void on_pushButton_5_clicked();

  void on_pushButton_6_clicked();

  void on_pushButton_7_clicked();

  void on_pushButton_8_clicked();

private:
  QTimer *ros_timer;

  QTimer *timer;

  QElapsedTimer *elapsed_timer;

  ros::ServiceClient client;

  khi_robot_msgs::KhiRobotCmd srv;

  QString *service_string;

  ros::NodeHandlePtr nh_;

  thread_handle *th1;

  QThread *thread1; 

  QTcpSocket *tcpClient;
};

#endif // HELLO_GUI_H

