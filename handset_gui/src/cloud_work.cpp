#include "../include/handset_gui/cloud_work.hpp"
#include <QTime>
#include <iostream>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <string>
#include <QStringListModel>
#include <QFuture>
#include <QtConcurrentRun>

#include <boost/lexical_cast.hpp>

namespace handset_gui {

using namespace std;

Cloud_Work::Cloud_Work(int argc, char **argv, QMutex *nmutex):init_argc(argc),
  init_argv(argv),mutex(nmutex)
{
  QFuture<void> future = QtConcurrent::run(this, &Cloud_Work::init);
}

Cloud_Work::~Cloud_Work(){
  if(ros::isStarted()){
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

void Cloud_Work::init(){

}

}
