#include "../include/handset_gui/registra_nuvem.hpp"
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

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace handset_gui {
///////////////////////////////////////////////////////////////////////////////////////////
RegistraNuvem::RegistraNuvem(int argc, char** argv):init_argc(argc), init_argv(argv){
    QFuture<void> future = QtConcurrent::run(this, &RegistraNuvem::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
RegistraNuvem::~RegistraNuvem(){
    if(ros::isStarted())
        ros::shutdown();
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::init(){
    ros::init(init_argc,init_argv,"RegistraNuvem");
    if ( ! ros::master::check() )  {
        cout << "check ros master not good" << endl;
        return;
    }
    ros::start();
    ros::NodeHandle nh_;

    // Inicia as nuvens
    src       = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    src_temp  = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    tgt       = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    acumulada = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();

    // Inicia a transformaçao
    T = Eigen::Matrix4f::Identity();
    R = Eigen::Matrix3f::Identity();
    t << 0.0, 0.0, 0.0;

    // Publicadores (a principio)
    pub_srctemp   = nh_.advertise<sensor_msgs::PointCloud2>("\nuvem_fonte_temp"    , 1);
    pub_tgt       = nh_.advertise<sensor_msgs::PointCloud2>("\nuvem_alvo_temp"     , 1);
    pub_acumulada = nh_.advertise<sensor_msgs::PointCloud2>("\nuvem_acumulada_temp", 1);

    ros::Rate rate(2);
    while(ros::ok()){
        rate.sleep();
        publicar_nuvens();
    }

    ros::shutdown();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::criaMatriz(){
    T << R, t,
         0, 0, 0, 1;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::publicar_nuvens(){
    sensor_msgs::PointCloud2 src_temp_msg, tgt_msg, acumulada_msg;
    std::string frame = "registro";
    if(src_temp->size() > 0){
        toROSMsg(*src_temp, src_temp_msg);
        src_temp_msg.header.frame_id = frame;
        pub_srctemp.publish(src_temp_msg);
    }
    if(tgt->size() > 0){
        toROSMsg(*tgt, tgt_msg);
        tgt.header.frame_id = frame;
        pub_tgt.publish(tgt_msg);
    }
    if(acumulada_msg->size() > 0){
        toROSMsg(*acumulada, acumulada_msg);
        acumulada_msg.header.frame_id = frame;
        pub_acumulada.publish(acumulada_msg);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_alvo(string nome){
    arquivo_tgt = nome;
    loadPLYFile(arquivo_tgt, *tgt);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_fonte(string nome){
    arquivo_src = nome;
    loadPLYFile(arquivo_src, *src);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    for(int i=arquivo_src.length(); i>0; i--){
      if (arquivo_src[i] == '/'){
        pasta_src = arquivo_src.left(i);
        break;
      }
    }
    cout << "\n\n\nA pasta aqui selecionada: " << pasta_src << "\n\n\n";
    // A principio as nuvens sao iguais, depois havera modificacao
    *src_temp = *src;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_translacao(float tx, float ty, float tz){
    // Recebe em Centimetros, converte pra METROS
    t << tx/100.0, ty/100.0, tz/100.0;
    criaMatriz();

    src_temp->clear();
    transformPointCloudWithNormals(*src, *src_temp, T);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_rotacao(float rx, float ry, float rz){
    // Recebe em GRAUS, converte pra radianos
    rx = deg2rad(rx);
    ry = deg2rad(ry);
    rz = deg2rad(rz);

    R = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()) *
        Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()) *
        Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());
    criaMatriz();

    src_temp->clear();
    transformPointCloudWithNormals(*src, *src_temp, T);
}
} // fim do namespace handset_gui
