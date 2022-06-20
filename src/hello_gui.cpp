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


#include "hello_gui.h"
#include "ui_hello_gui.h"
#include "thread_handle.h"


enum movement_state{
  _moving,
  _stopped
};

static int movement_state = _stopped;

//static const int16_t half_16 = 32767;

static uint8_t function_invertor = 1;

static uint8_t sent_packet_counter = 0;

static QByteArray recieved;

HelloGui::HelloGui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::HelloGui)
{
  qDebug() << "SETUP...";
  ui->setupUi(this);

  tcpClient = new QTcpSocket (this);

  bool success = connect(tcpClient, SIGNAL(stateChanged(QAbstractSocket::SocketState)),
                         this, SLOT(HandleStateChange(QAbstractSocket::SocketState)));
  Q_ASSERT(success);

  success = connect(tcpClient, SIGNAL(readyRead()),
                    this, SLOT(recieved_msg_handler()));
  Q_ASSERT(success);

  tcpClient->connectToHost(QString("192.168.1.228"), static_cast<uint16_t>(23));
  tcpClient->flush();


  nh_.reset(new ros::NodeHandle("~"));
  elapsed_timer = new QElapsedTimer;
//   setup the timer that will signal ros stuff to happen
  ros_timer = new QTimer(this);
  success = connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  Q_ASSERT(success);

  ros_timer->start(50);  // set the rate to 100ms  You can change this if you want to increase/decrease update rate

//  timer = new QTimer(this);
//  connect(timer, SIGNAL(timeout()), this, SLOT(update_timer()));

  thread1 = new QThread;
  th1 = new thread_handle;

  th1->moveToThread(thread1);

  success = connect(this, SIGNAL(emit_button(QString, QString, double, uint)),
                    th1, SLOT(communicate_slot(QString, QString, double, uint)));
  Q_ASSERT(success);

  success = connect(this, SIGNAL(emit_sending_signal(bool)),
                    th1, SLOT(check_sending(bool)));
  Q_ASSERT(success);

  success = connect(this, SIGNAL(write_to_txt_signal()),
                    th1, SLOT(write_to_txt()));
  Q_ASSERT(success);

  client  = nh_->serviceClient<khi_robot_msgs::KhiRobotCmd>("/khi_robot_command_service");

  qDebug() << "SETUP FINISHED";
}

HelloGui::~HelloGui(){
  delete ui;
  delete ros_timer;
}


void HelloGui::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else
      QApplication::quit();
}


void HelloGui::service_call(uint start_signal, uint8_t number_of_bits, uint16_t number_to_send){
  //elapsed_timer->restart();
  service_string = new QString;
  //number_to_send += half_16;
  srv.request.type = "as";
  service_string->append(QString("BITS %1, %2 = %3")
                         .arg(QString::number(start_signal))
                         .arg(QString::number(number_of_bits))
                         .arg(QString::number(number_to_send)));

  srv.request.cmd = service_string->toStdString();
  cout << service_string->toStdString() <<endl;
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
  /*else{
    ROS_INFO("CMD_REturn: %d" ,static_cast<int>(srv.response.driver_ret));
  }*/
  //ROS_INFO("Time: %d", static_cast<int>(elapsed_timer->elapsed()));
  free(service_string);
}


void HelloGui::get_signal(uint start_signal){
  service_string = new QString;
  srv.request.type = "driver";

  service_string->append(QString("get_signal %1").arg(QString::number(start_signal)));
  srv.request.cmd = service_string->toStdString();
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
  cout << "Service reply: " << srv.response.driver_ret << endl;
  cout << "Signal: " << start_signal << " = " << srv.response.cmd_ret <<endl;
  free(service_string);
}


void HelloGui::get_bits(uint16_t start_bit, uint8_t number_of_bits){
  service_string = new QString;
  srv.request.type = "as";
  service_string->append(QString("BITS %1, %2").arg(QString::number(start_bit)).arg(QString::number(number_of_bits)));
  srv.request.cmd = service_string->toStdString();
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
  cout << "Service reply: " << srv.response.cmd_ret << endl;
  //cout << service_string2->toStdString() <<endl;
  free(service_string);
}


void HelloGui::set_signals(uint8_t number, uint8_t number_of_bits, uint16_t start_signal){
  service_string = new QString;
  srv.request.type = "driver";
  service_string->append(QString("set_signal "));
  const unsigned int mask = 1U << (number_of_bits - 1);
  uint8_t copy_number = number;
  for (int i = 0; i < number_of_bits; i++) {
    if (i != number_of_bits - 1 && (copy_number & mask)){
      service_string->append(QString("%1,").arg(QString::number(start_signal + i)));
    }
    else if (i != number_of_bits - 1 && !(copy_number & mask)) {
      service_string->append(QString("%1,").arg(QString::number(-(start_signal + i))));
    }
    else  if (i == number_of_bits - 1 && (copy_number & mask))
      service_string->append(QString("%1").arg(QString::number(start_signal + i)));
    else
      service_string->append(QString("%1").arg(QString::number(-(start_signal + i))));
    copy_number <<= 1;
  }
  srv.request.cmd = service_string->toStdString();
  cout << service_string->toStdString() <<endl;
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
  cout << "Service reply: " << srv.response.driver_ret << endl;
  free(service_string);
}


void HelloGui::set_variable(QString variable, double value){
  service_string = new QString;
  srv.request.type = "as";
  service_string->append(QString("%1 = %2")
                         .arg(variable)
                         .arg(QString::number(value)));
  //ui->lcdNumber->display(value);
  srv.request.cmd = service_string->toStdString();
  cout << service_string->toStdString() <<endl;
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
  cout << "Service reply: " << srv.response.driver_ret << endl;
  free(service_string);
}


void HelloGui::get_variable_value(QString variable){
  service_string = new QString;
  srv.request.type = "as";
  service_string->append((QString("print %1").arg(variable)));
  srv.request.cmd = service_string->toStdString();
  cout << service_string->toStdString() <<endl;
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
  ui->lcdNumber->display(QString::fromStdString(srv.response.cmd_ret).toDouble());
  cout << "Service reply: " << srv.response.cmd_ret << endl;
  free(service_string);
}

void HelloGui::sendData(const QByteArray &ba){
  tcpClient->write(ba.constData(), ba.count());
}

void HelloGui::service_call(){
//  if(!ui->radioButton->isChecked()) return;

  elapsed_timer->restart();

  if(function_invertor)
    set_variable(ui->lineEdit->text() ,(static_cast<double>(ui->spinBox_3->value()) / 100));
  else
    set_variable(ui->lineEdit->text() , -(static_cast<double>(ui->spinBox_3->value()) / 100));

  if (sent_packet_counter == 10){
    sent_packet_counter = 0;
    function_invertor = ! function_invertor;
  }
  sent_packet_counter++;
  get_variable_value(ui->lineEdit_2->text());
  ROS_INFO("Time: %d", static_cast<int>(elapsed_timer->elapsed()));
  emit signal_service_ended();
}

void HelloGui::on_hi_button_clicked(){
  service_string = new QString;
  if (movement_state == _stopped){
    service_string -> append(QString("set_signal %1, %2").arg(QString::number(2001)).arg(QString::number(2002)));
    srv.request.cmd = service_string->toStdString();
    movement_state = _moving;
  }
  else{
    service_string -> append(QString("set_signal %1").arg(QString::number(-2001)));
    srv.request.cmd = service_string->toStdString();
    movement_state = _stopped;
  }
  srv.request.type = "driver";
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
  else{
    ROS_INFO("CMD_REturn: %d" ,static_cast<int>(srv.response.driver_ret));
  }
  free(service_string);
}


void HelloGui::on_pushButton_clicked(){
  elapsed_timer->restart();
  //get_signal(static_cast<uint>(ui->hi_num->value()));
  get_bits(static_cast<uint16_t>(ui->hi_num->value()), 16);
  ROS_INFO("Time get_signal: %d", static_cast<int>(elapsed_timer->elapsed()));
}



void HelloGui::update_timer(){
//  if(!ui->radioButton->isChecked()) return;

  elapsed_timer->restart();

  if(function_invertor)
    set_variable(ui->lineEdit->text() ,(static_cast<double>(ui->spinBox_3->value()) / 100));
  else
    set_variable(ui->lineEdit->text() , -(static_cast<double>(ui->spinBox_3->value()) / 100));

  if (sent_packet_counter == 10){
    sent_packet_counter = 0;
    function_invertor = ! function_invertor;
  }
  sent_packet_counter++;
  get_variable_value(ui->lineEdit_2->text());
  ROS_INFO("Time: %d", static_cast<int>(elapsed_timer->elapsed()));


}

void HelloGui::on_pushButton_3_clicked(){
    elapsed_timer->restart();
    set_signals(static_cast<uint8_t>(ui->spinBox->value()), 8, 2003);
    ROS_INFO("Time set_signals: %d", static_cast<int>(elapsed_timer->elapsed()));
}

void HelloGui::on_pushButton_2_clicked(){
  elapsed_timer->restart();
  service_call(static_cast<uint>(ui->spinBox_5->value()),
               static_cast<uint8_t>(ui->spinBox_4->value()),
               static_cast<uint16_t>(ui->spinBox_2->value()));
  ROS_INFO("Time BITS: %d", static_cast<int>(elapsed_timer->elapsed()));
}

void HelloGui::on_pushButton_4_clicked(){
    elapsed_timer->start();
    cout << static_cast<double>(ui->spinBox_3->value()) / 100 << endl;
    set_variable(ui->lineEdit->text() ,(static_cast<double>(ui->spinBox_3->value()) / 100));

    ROS_INFO("Time set_variable: %d", static_cast<int>(elapsed_timer->elapsed()));
}

void HelloGui::on_pushButton_5_clicked(){
  elapsed_timer->restart();
  get_variable_value(ui->lineEdit_2->text());
  ROS_INFO("Time get_variable_value: %d", static_cast<int>(elapsed_timer->elapsed()));
}

void HelloGui::on_pushButton_6_clicked(){
  if(!thread1->isRunning()){
    thread1->start();
    ui->checkBox_2->setChecked(true);
    ui->checkBox->setChecked(true);
    emit emit_button(ui->lineEdit->text(), ui->lineEdit_2->text(),
                    (static_cast<double>(ui->spinBox_3->value()) / 100),
                     static_cast<uint>(ui->spinBox_6->value()));
    emit_sending_signal(ui->checkBox->isChecked());
    ui->pushButton->setDisabled(true);
    ui->pushButton_2->setDisabled(true);
    ui->pushButton_3->setDisabled(true);
    ui->pushButton_4->setDisabled(true);
    ui->pushButton_5->setDisabled(true);
    ui->hi_button->setDisabled(true);
    return;
  }
  thread1->terminate();
  ui->checkBox_2->setChecked(false);
  ui->checkBox->setChecked(false);
  ui->pushButton->setDisabled(false);
  ui->pushButton_2->setDisabled(false);
  ui->pushButton_3->setDisabled(false);
  ui->pushButton_4->setDisabled(false);
  ui->pushButton_5->setDisabled(false);
  ui->hi_button->setDisabled(false);
}

void HelloGui::on_pushButton_7_clicked(){
  if(!thread1->isRunning()){
    thread1->start();
    emit write_to_txt_signal();
  }
}

void HelloGui::HandleStateChange(QAbstractSocket::SocketState socketState){
  qDebug() << socketState;
  if(socketState == QAbstractSocket::ConnectedState)
    tcpClient->write("as\n");
}

void HelloGui::recieved_msg_handler(){
  recieved = tcpClient->readAll();

  ui->plainTextEdit_2->insertPlainText(recieved);
  ui->plainTextEdit_2->insertPlainText("\n");
  ui->plainTextEdit_2->verticalScrollBar()->setValue(INT_MAX);
}

void HelloGui::on_pushButton_8_clicked(){
    QString text = ui->lineEdit_3->text();
    text.append("\n");
    sendData(text.toLatin1());
}
