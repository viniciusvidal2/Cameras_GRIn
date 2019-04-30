#ifndef QTIMAGENODE_H
#define QTIMAGENODE_H

#endif // QTIMAGENODE_H

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"

#include <image_geometry/pinhole_camera_model.h>
#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
#include <yaml-cpp/yaml.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ctime>

#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamSetRequest.h>
#include <mavros_msgs/ParamSetResponse.h>
#include <mavros_msgs/ParamValue.h>

#include <string>
#include <ros/ros.h>
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
#include <qobject.h>
#include <QThread>
#include <QMutex>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace pcl;
using namespace message_filters;
using namespace nav_msgs;
using namespace cv;

namespace handset_gui {


class Cloud_Work : public QThread
{
  Q_OBJECT
public:
  Cloud_Work(int argc, char** argv, QMutex*);
  virtual ~Cloud_Work();
  void init();
//  void run_class();
  void set_inicio_acumulacao(bool flag);
  void set_primeira_vez(bool flag);
  void set_tempo_aquisicao(float t);

  QMutex* mutex;



private:
  /// Definicoes ///
  typedef PointXYZRGB PointT;
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2, Odometry> syncPolicy;
  typedef Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync;

  /// Procedimentos internos ///
  void filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size);
  void passthrough(PointCloud<PointT>::Ptr cloud, std::string field, float min, float max);
  Eigen::Matrix4f icp(const PointCloud<PointT>::Ptr src, const PointCloud<PointT>::Ptr tgt, PointCloud<PointT>::Ptr final, Eigen::Matrix4f T);
  void callback_acumulacao(const sensor_msgs::ImageConstPtr &msg_image, const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const OdometryConstPtr &msg_odom);
  void registra_global_icp(PointCloud<PointT>::Ptr parcial, Eigen::Quaternion<float> rot, Eigen::Vector3f offset);
  Eigen::Matrix4f qt2T(Eigen::Quaternion<float> rot, Eigen::Vector3f offset);
  void salva_dados_parciais(PointCloud<PointT>::Ptr cloud, Eigen::Quaternion<float> rot, Eigen::Vector3f offset, sensor_msgs::ImageConstPtr &imagem);

  /// Variaveis ///
  int init_argc;
  char** init_argv;
  // Para iniciar a acumulacao, eh finalizada por tempo depois
  bool realizar_acumulacao;
  // Analisa se primeira vez ou nao para controlar acumulacao
  bool primeira_vez;
  // Tempo global de inicio de cada aquisicao e por quanto tempo ficara aquisitando
  ros::Time t_inicio_aquisicao;
  float t_aquisicao;
  // Nuvem acumulada total
  PointCloud<PointT>::Ptr acumulada_global;
  // Nuvem acumulada parcial atual e anterior, para cada intervalo de aquisicao
  PointCloud<PointT>::Ptr acumulada_parcial;
  PointCloud<PointT>::Ptr acumulada_parcial_anterior;
  // Matriz de transformacao para a iteracao anterior
  Eigen::Matrix4f T_anterior;
  // Listener assincrono para tf entre zed e astra
  tf::TransformListener *p_listener;
  // Publicadores para nuvem parcial e total;
  ros::Publisher pub_total;
  ros::Publisher pub_parcial;
  // Pasta onde sera guardado o projeto para o MART e tudo o mais
  std::string pasta;
};

}
