#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

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
#include <geometry_msgs/PoseStamped.h>

using namespace nav_msgs;
using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace message_filters;

typedef PointXYZRGBNormal PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, Odometry> syncPolicy;

// Variaveis globais
PointCloud<PointT>::Ptr caminho_viso;
PointCloud<PointT>::Ptr caminho_kitti;
tf::TransformListener *kitti_tf;

double distancia_kitti = 0, distancia_viso = 0, ref_kitti_x, ref_kitti_y, ref_viso_x, ref_viso_y;
bool primeira_vez_kitti = true, primeira_vez_viso = true;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void visualizar_nuvem(){
  boost::shared_ptr<PCLVisualizer> vis_placa (new PCLVisualizer("caminhos"));
  vis_placa->addPointCloud<PointT>(caminho_viso, "caminho_viso");
  vis_placa->addPointCloud<PointT>(caminho_kitti, "caminho_stereo");
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
  m.setRotation(q_atual);
  m.getRPY(roll, pitch, yaw);

  point.x        = odom->pose.pose.position.x;
  point.y        = odom->pose.pose.position.z;
  point.z        = odom->pose.pose.position.y;
  point.normal_x = roll;
  point.normal_y = pitch;
  point.normal_z = yaw;

  // Acumular na nuvem viso
  point.r = 0.0f; point.g = 250.0f; point.b = 0.0f; // Verde para referencia
  caminho_viso->push_back(point);

  if(!primeira_vez_viso){
      distancia_viso = sqrt( pow(point.x-ref_viso_x, 2) + pow(point.y-ref_viso_y, 2) );
  } else {
      ref_viso_x = point.x; ref_viso_y = point.y;
      primeira_vez_viso = false;
  }
  ROS_INFO("DISTANCIA DA VISO: %.2f", distancia_viso);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void atualizar_nuvem2(const tf::StampedTransform t, string nome){
  // Salvar cada odometria na nuvem
  PointT point;

  tf::Vector3 v;
  v = t.getOrigin();

  point.x = v.getX(); point.y = v.getZ(); point.z = v.getY();

  if(!primeira_vez_kitti){
      distancia_kitti = sqrt( pow(point.x-ref_kitti_x, 2) + pow(point.y-ref_kitti_y, 2) );
  } else {
      ref_kitti_x = point.x; ref_kitti_y = point.y;
      primeira_vez_kitti = false;
  }
  ROS_INFO("DISTANCIA KITTI: %.2f", distancia_kitti);

  // Acumular na nuvem viso
  point.r = 0.0f; point.g = 0.0f; point.b = 250.0f; // Azul para referencia
  caminho_kitti->push_back(point);

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
  string filename1 = (home+"/Desktop/kitti_"+date+".ply").c_str();
  string filename2 = (home+"/Desktop/viso2_"+date+".ply").c_str();
  // Salvando com o nome diferenciado
  if(!io::savePLYFileASCII(filename1, *caminho_kitti))
    cout << "\n\nSalvo stereo na pasta caminhos com o nome kitti_"+date+".ply" << endl;
  if(!io::savePLYFileASCII(filename2, *caminho_viso))
    cout << "\n\nSalvo placa  na pasta caminhos com o nome viso2_"+date+".ply" << endl;

  ros::shutdown();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void visoCallback(const sensor_msgs::ImageConstPtr &msg_ima, const nav_msgs::OdometryConstPtr& msg_viso)
{
    tf::StampedTransform trans;
    try
    {
        kitti_tf->waitForTransform("world", "camera_left", msg_viso->header.stamp, ros::Duration(1.0));
        kitti_tf->lookupTransform( "world", "camera_left", msg_viso->header.stamp, trans);
    }
    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        ROS_WARN("Cannot accumulate");
        return;
    }

    // Atualizar as nuvens com as mensagens recebidas
    atualizar_nuvem(msg_viso, "viso");
    atualizar_nuvem2(trans, "stereo");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_tf");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    // Inicia nuvens
    caminho_viso  = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    caminho_kitti = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

    // Inicia tf listener
    kitti_tf = (tf::TransformListener*) new tf::TransformListener;

    string topico_viso, topico_imagem;
    n_.getParam("viso_topic" , topico_viso  );
    n_.getParam("image_topic", topico_imagem);

//    ros::Subscriber sub = nh.subscribe(topico_viso, 1, visoCallback);
    message_filters::Subscriber<sensor_msgs::Image> subima(nh, topico_imagem, 10);
    message_filters::Subscriber<Odometry>           subodo(nh, topico_viso  , 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), subima, subodo);
    sync.registerCallback(boost::bind(&visoCallback, _1, _2));

    while(ros::ok()){
      ros::spinOnce();
    }

    salvar_nuvens();
    visualizar_nuvem();

    return 0;
}
