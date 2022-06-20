#include "hello_gui.h"

static bool isSending = false;
static QString variable_to_send, variable_to_get, chopped;
static double value_to_send;
static uint timer_value_msec;
static uint8_t function_invertor = 1;
//static uint counter = 0;

thread_handle::thread_handle(QObject *parent) : QObject(parent){
  qDebug() << "Thread Handler SETUP...";
  nh2_.reset(new ros::NodeHandle("~"));
  ros_timer2 = new QTimer(this);
  elapsed_timer = new QElapsedTimer;
  elapsed_timer->restart();
  elapsed_timer_log = new QElapsedTimer;
  bool success = connect(ros_timer2, SIGNAL(timeout()), this, SLOT(spinOnce1()));
  Q_ASSERT(success);
  success = connect(this, SIGNAL(emit_thread_loop()), this, SLOT(_thread_loop()));
  Q_ASSERT(success);
  client2  = nh2_->serviceClient<khi_robot_msgs::KhiRobotCmd>("/khi_robot_command_service");
  recieved_value = new QString;
  qDebug() << "Thread Handler SETUP ENDED";
}

void thread_handle::communicate_slot(QString variable, QString variable_2, double value, uint timer_msec){
//  _set_variable(variable, value);
  variable_to_send = variable;
  variable_to_get = variable_2;
  value_to_send = value;
  timer_value_msec = timer_msec;
  qDebug()<< "Recieved variables and its value, timer:" << variable_to_send << variable_to_get
          <<  value_to_send << timer_msec;
  elapsed_timer_log->start();
}

void thread_handle::check_sending(bool send){
  isSending = send;
  qDebug() << "isSending value changed" << isSending;
  if(isSending) emit emit_thread_loop();
}

void thread_handle::write_to_txt(){
    qDebug()<< "wrtie to txt began...";
    QFile file("log_file_test.txt");
//    cout << recieved_value->toStdString() <<endl;
    file.open(QIODevice::WriteOnly);
    file.write(recieved_value->toUtf8());
    file.close();
}

void thread_handle::_thread_loop(){
  if (elapsed_timer->elapsed() >= timer_value_msec){
    function_invertor = ! function_invertor;
    elapsed_timer->restart();
  }
  if(function_invertor)
    _set_variable(variable_to_send, value_to_send);
  else
    _set_variable(variable_to_send, -value_to_send);
  _get_variable_value(variable_to_get);
//  qDebug() <<"Time: "<<static_cast<int>(elapsed_timer->elapsed());
  if(isSending) _thread_loop();
}


void thread_handle::spinOnce1(){
  if(ros::ok()){
    ros::spinOnce();
  }
//  else
//     thread_handle::~thread_handle();
}

thread_handle::~thread_handle(){
  delete ros_timer2;
}

void thread_handle::_set_variable(QString variable, double value){
  service_string2 = new QString;
  srv2.request.type = "as";
  service_string2->append(QString("%1 = %2")
                         .arg(variable)
                         .arg(QString::number(value)));
  srv2.request.cmd = service_string2->toStdString();
//  cout << service_string2->toStdString() <<endl;
  if(!client2.call(srv2)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
    return;
  }
//  cout << "Service reply: " << srv2.response.driver_ret << endl;

  free(service_string2);
}

void thread_handle::_get_variable_value(QString variable){
  service_string2 = new QString;
  chopped  = new QString;
  srv2.request.type = "as";
  service_string2->append((QString("print %1").arg(variable)));
  srv2.request.cmd = service_string2->toStdString();
//  cout << service_string2->toStdString() <<endl;
  if(!client2.call(srv2)){
    ROS_ERROR("Failed to call service /khi_robot_command_service");
  }
//  cout << "Service reply: " << srv2.response.cmd_ret << endl;
  chopped->append(QString::fromStdString(srv2.response.cmd_ret));
  chopped->chop(2);
  recieved_value->append(QString("%1,%2\n").arg(*chopped).arg(QString::number(elapsed_timer_log->elapsed())));
  free(service_string2);
}


