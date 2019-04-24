#ifndef QTIMAGENODE_H
#define QTIMAGENODE_H

#endif // QTIMAGENODE_H

#include <string>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/NavSatFix.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <qobject.h>
#include <QThread>
#include <QMutex>

#include "mavros_msgs/GlobalPositionTarget.h"
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamSetRequest.h>
#include <mavros_msgs/ParamSetResponse.h>
#include <mavros_msgs/ParamValue.h>

#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <math.h>

namespace handset_gui {

class Cloud_Work : public QThread
{
  Q_OBJECT
public:
  Cloud_Work(int argc, char** argv, QMutex*);
  virtual ~Cloud_Work();
  void init();
  void run_class();

  QMutex* mutex;

private:
  int init_argc;
  char** init_argv;

};

}
