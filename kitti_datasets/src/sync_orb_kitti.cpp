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

// Variaveis globais
PointCloud<PointT>::Ptr caminho_orb;
PointCloud<PointT>::Ptr caminho_kitti;
tf::TransformListener *kitti_tf;
tf::TransformListener *orb_tf;

double distancia_kitti = 0, distancia_orb = 0, ref_kitti_x, ref_kitti_y, ref_orb_x, ref_orb_y;
bool primeira_vez_kitti = true, primeira_vez_orb = true;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void visualizar_nuvem(){
    boost::shared_ptr<PCLVisualizer> vis_placa (new PCLVisualizer("caminhos"));
    vis_placa->addPointCloud<PointT>(caminho_orb, "caminho_orb");
    vis_placa->addPointCloud<PointT>(caminho_kitti, "caminho_stereo");
    vis_placa->spin();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void atualizar_nuvem(const tf::StampedTransform t, string nome){
    // Salvar cada odometria na nuvem
    PointT point;

    tf::Vector3 v;
    v = t.getOrigin();

    if(nome == "kitti"){


        point.x = v.getX(); point.y = v.getZ(); point.z = v.getY(); // Inverter os eixos para bater com a transformada

        if(!primeira_vez_kitti){
            distancia_kitti = sqrt( pow(point.x-ref_kitti_x, 2) + pow(point.y-ref_kitti_y, 2) );
        } else {
            ref_kitti_x = point.x; ref_kitti_y = point.y;
            primeira_vez_kitti = false;
        }
        ROS_INFO("DISTANCIA KITTI: %.2f", distancia_kitti);

        // Acumular na nuvem orb
        point.r = 0.0f; point.g = 250.0f; point.b = 0.0f; // Verde para referencia
        caminho_kitti->push_back(point);

    } else {

        point.x = v.getX(); point.y = v.getY(); point.z = v.getZ();

        if(!primeira_vez_orb){
            distancia_orb = sqrt( pow(point.x-ref_orb_x, 2) + pow(point.y-ref_orb_y, 2) );
        } else {
            ref_orb_x = point.x; ref_orb_y = point.y;
            primeira_vez_orb = false;
        }
        ROS_INFO("DISTANCIA ORB: %.2f", distancia_orb);

        // Acumular na nuvem orb

        point.r = 0.0f; point.g = 0.0f; point.b = 250.0f; // Azul para referencia
        caminho_orb->push_back(point);
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
    string filename1 = (home+"/Desktop/kitti_"+date+".ply").c_str();
    string filename2 = (home+"/Desktop/orb_"+date+".ply").c_str();
    // Salvando com o nome diferenciado
    if(!io::savePLYFileASCII(filename1, *caminho_kitti))
        cout << "\n\nSalvo stereo na pasta caminhos com o nome kitti_"+date+".ply" << endl;
    if(!io::savePLYFileASCII(filename2, *caminho_orb))
        cout << "\n\nSalvo placa  na pasta caminhos com o nome orb_"+date+".ply" << endl;

    ros::shutdown();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void orbCallback(const sensor_msgs::ImageConstPtr &msg_ima)
{
    ROS_INFO("ENTROU NO CALLBACK!");
    tf::StampedTransform trans_orb, trans_kitti;
    try
    {
        kitti_tf->waitForTransform("world", "camera_left", msg_ima->header.stamp, ros::Duration(1.0));
        kitti_tf->lookupTransform( "world", "camera_left", msg_ima->header.stamp, trans_kitti);
        orb_tf->waitForTransform("world", "base_link", msg_ima->header.stamp, ros::Duration(1.0));
        orb_tf->lookupTransform( "world", "base_link", msg_ima->header.stamp, trans_orb);
    }
    catch (tf::TransformException& ex){
        ROS_ERROR("%s",ex.what());
        ROS_WARN("Cannot accumulate");
        return;
    }

    // Atualizar as nuvens com as mensagens recebidas
    atualizar_nuvem(trans_orb  , "orb");
    atualizar_nuvem(trans_kitti, "kitti");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sync_orb_kitti");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    // Inicia nuvens
    caminho_orb   = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    caminho_kitti = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

    // Inicia tf listener
    kitti_tf = (tf::TransformListener*) new tf::TransformListener;
    orb_tf   = (tf::TransformListener*) new tf::TransformListener;

    string topico_imagem;
    n_.getParam("image_topic", topico_imagem);

    ros::Subscriber sub = nh.subscribe(topico_imagem, 1, orbCallback);

    ROS_INFO("COMECOU O NO BEM!");

    while(ros::ok()){
        ros::spinOnce();
    }

    salvar_nuvens();
    visualizar_nuvem();

    return 0;
}
