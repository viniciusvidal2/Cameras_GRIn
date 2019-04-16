#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread/thread.hpp>
#include <iostream>
#include <string>
#include <numeric>
#include <ros/ros.h>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

/// Namespaces
using namespace pcl;

using namespace pcl::visualization;

using namespace std;
using namespace message_filters;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace cv;

/// Definitions
typedef PointXYZRGBNormal PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, Odometry, Odometry> syncPolicy;

// Variaveis globais
PointCloud<PointT>::Ptr caminho_zed;
PointCloud<PointT>::Ptr caminho_odo;

int cont = 0, iteracoes = 10000;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void visualizar_nuvem(){
  boost::shared_ptr<PCLVisualizer> vis_placa (new PCLVisualizer("caminhos"));
  vis_placa->addPointCloud<PointT>(caminho_zed, "caminho_pix");
  vis_placa->addPointCloud<PointT>(caminho_odo, "caminho_stereo");
  vis_placa->spin();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void atualizar_nuvem(const OdometryConstPtr& odom, string nome){
  // Salvar cada odometria na nuvem
  PointT point;
  tf::Quaternion q_atual;
  tf::Matrix3x3 m;
  double roll, pitch, yaw;

  q_atual.setX((double)odom->pose.pose.orientation.x);
  q_atual.setY((double)odom->pose.pose.orientation.y);
  q_atual.setZ((double)odom->pose.pose.orientation.z);
  q_atual.setW((double)odom->pose.pose.orientation.w);
//  q_atual.x() = (double)odom->pose.pose.orientation.x;
//  q_atual.y() = (double)odom->pose.pose.orientation.y;
//  q_atual.z() = (double)odom->pose.pose.orientation.z;
//  q_atual.w() = (double)odom->pose.pose.orientation.w;
  m.setRotation(q_atual);
  m.getRPY(roll, pitch, yaw);

  point.x        = odom->pose.pose.position.x;
  point.y        = odom->pose.pose.position.y;
  point.z        = odom->pose.pose.position.z;
  point.normal_x = roll;
  point.normal_y = pitch;
  point.normal_z = yaw;

  // Acumular na nuvem certa
  if(nome == "pix"){
    point.r = 0.0f; point.g = 250.0f; point.b = 0.0f; // Verde para referencia
    caminho_zed->push_back(point);
  } else {
    point.r = 0.0f; point.g = 0.0f; point.b = 250.0f; // Azul para o testado
    caminho_odo->push_back(point);
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void salvar_nuvens(){
  ROS_INFO("Salvando o arquivo com a odometria para ser guardado...");
  time_t t = time(0);
  struct tm * now = localtime( & t );
  std::string month, day, hour, minutes;
  month   = boost::lexical_cast<std::string>(now->tm_mon );
  day     = boost::lexical_cast<std::string>(now->tm_mday);
  hour    = boost::lexical_cast<std::string>(now->tm_hour);
  minutes = boost::lexical_cast<std::string>(now->tm_min );
  string date = "_" + month + "_" + day + "_" + hour + "h_" + minutes + "m";
  string home = getenv("HOME");
  string filename1 = (home+"/Desktop/stereo_"+date+".ply").c_str();
  string filename2 = (home+"/Desktop/placa_"+date+".ply").c_str();
  // Salvando com o nome diferenciado
  if(!io::savePLYFileASCII(filename1, *caminho_odo))
    cout << "\n\nSalvo stereo na pasta caminhos com o nome stereo_"+date+".ply" << endl;
  if(!io::savePLYFileASCII(filename2, *caminho_zed))
    cout << "\n\nSalvo placa  na pasta caminhos com o nome placa_"+date+".ply" << endl;

  ros::shutdown();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void odoms_callback(const sensor_msgs::ImageConstPtr& msg_im,
                    const OdometryConstPtr& msg_pix, const OdometryConstPtr& msg_odo){
  if(cont < iteracoes){
    cout << "\nOdometria salva para o ponto " << cont << "." << endl;
    atualizar_nuvem(msg_pix, "pix");
    atualizar_nuvem(msg_odo, "stereo");
    cont++;
  } else {
    salvar_nuvens();
    visualizar_nuvem();

    ros::shutdown();
  }

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sync_odom");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~");

  // Inicia nuvens
  caminho_zed = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  caminho_odo = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  // Subscriber para a imagem instantanea e odometrias
  message_filters::Subscriber<sensor_msgs::Image> subima(nh, "/duo3d/left/image_rect"   , 100);
  message_filters::Subscriber<Odometry>           subpix(nh, "/stereo_odometry"                , 100);
  message_filters::Subscriber<Odometry>           subodo(nh, "/stereo_odometer/odometry", 100);

  // Sincroniza as leituras dos topicos
  Synchronizer<syncPolicy> sync(syncPolicy(100), subima, subpix, subodo);
  sync.registerCallback(boost::bind(&odoms_callback, _1, _2, _3));

  while(ros::ok()){
    ros::spinOnce();
  }
//  if(!ros::ok())
  salvar_nuvens();
  visualizar_nuvem();

  return 0;
}
