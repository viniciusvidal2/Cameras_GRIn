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

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace handset_gui {

using namespace std;
///////////////////////////////////////////////////////////////////////////////////////////
Cloud_Work::Cloud_Work(int argc, char **argv, QMutex *nmutex):init_argc(argc),
    init_argv(argv),mutex(nmutex)
{
    QFuture<void> future = QtConcurrent::run(this, &Cloud_Work::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
Cloud_Work::~Cloud_Work(){
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::init(){

    ros::init(init_argc,init_argv,"Cloud_Work");
    if ( ! ros::master::check() )  {
        cout << "check ros master not good" << endl;
        return;
    }
    ros::start();
    ros::NodeHandle nh_;

    // Inicia nuvens globais
    acumulada_parcial          = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    acumulada_parcial_anterior = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    acumulada_global           = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

    // Inicia publicadores de nuvens
    pub_parcial = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_parcial", 10);
    pub_total   = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_global" , 10);

    // Inicia subscriber de TF assincrono
    p_listener = (tf::TransformListener*) new tf::TransformListener;

    // Subscribers para sincronizar
    message_filters::Subscriber<sensor_msgs::Image      > sub_imagem(nh_, "topico_imagem", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_nuvem (nh_, "topico_nuvem" , 10);
    message_filters::Subscriber<Odometry                > sub_odom  (nh_, "topico_odom"  , 10);
    sync.reset(new Sync(syncPolicy(10), sub_imagem, sub_nuvem, sub_odom));
    sync->registerCallback(boost::bind(&Cloud_Work::callback_acumulacao, this, _1, _2, _3));

    ros::spin();

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::passthrough(PointCloud<PointT>::Ptr cloud, string field, float min, float max){
    PassThrough<PointT> ps;
    ps.setInputCloud(cloud);
    ps.setFilterFieldName(field);
    ps.setFilterLimits(min, max);

    ps.filter(*cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Cloud_Work::qt2T(Eigen::Quaternion<float> rot, Eigen::Vector3f offset){
    Eigen::Matrix4f T;
    Eigen::Matrix3f R = rot.matrix();//cout << "R " << R << endl;
    Eigen::MatrixXf t(3,1);
    t = offset;//cout << "T " << T << endl;
    T << R,t,
         0,0,0,1;
    cout << "\n\n Tira teima se esta funcionando transformacao:\n";
    cout << "RT " << T << endl;

    return T;
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Cloud_Work::icp(const PointCloud<PointT>::Ptr src,
                                const PointCloud<PointT>::Ptr tgt,
                                PointCloud<PointT>::Ptr final,
                                Eigen::Matrix4f T){
    ROS_INFO("Entrando no ICP");
    pcl::IterativeClosestPoint<PointT, PointT> registration;
    registration.setUseReciprocalCorrespondences(true);
    Eigen::Matrix4f T_icp = T;
    for(int i = 0; i < 1; i++)
    {
  //    registration.setMaxCorrespondenceDistance(0.1);
  //    registration.setMaxCorrespondenceDistance(0.0009);
      registration.setMaximumIterations(700);
      registration.setRANSACIterations(700);
      registration.setTransformationEpsilon(1*1e-9);
      registration.setEuclideanFitnessEpsilon(0.00000000001);
      registration.setInputCloud(src);
      registration.setInputTarget(tgt);
      registration.align(*final, T);
      T_icp = registration.getFinalTransformation()*T;
    }
    if(registration.hasConverged()){
        cout << "\nConvergiu: " << registration.hasConverged() << " score: " <<
                registration.getFitnessScore() << endl;
        transformPointCloud(*src, *src, T_icp);
    } else {
        cout << "\nICP nao convergiu, confiando somente na Odometria." << endl;
        transformPointCloud(*src, *src, T);
        T_icp = T;
    }
    return T_icp;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::registra_global_icp(PointCloud<PointT>::Ptr parcial, Eigen::Quaternion<float> rot, Eigen::Vector3f offset){
    // Se primeira vez, so acumula, senao registra com ICP com a
    // transformada relativa entregue pela ZED como aproximacao inicial
    if(!primeira_vez){

        ROS_INFO("NADA A ACUMULAR POR ENQUANTO!");

    } else {
        // Primeira nuvem
        *acumulada_global          += *parcial;
        *acumulada_parcial_anterior = *parcial;
        // Primeira transformacao vinda da odometria -> converter matriz 4x4
        T_anterior = qt2T(rot, offset);
        // Podemos dizer que nao e mais primeira vez, chaveia a variavel
        set_primeira_vez(false);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::callback_acumulacao(const sensor_msgs::ImageConstPtr &msg_image,
                                     const sensor_msgs::PointCloud2ConstPtr &msg_cloud,
                                     const OdometryConstPtr &msg_odom){

    ROS_INFO("Entramos no callback de acumulacao");
    // Se podemos iniciar a acumular (inicialmente botao da GUI setou essa flag)
    if(realizar_acumulacao){
        ROS_INFO("Gravando nuvens que chegam para acumular");
        Eigen::Quaternion<float> q;
        Eigen::Vector3f offset;
        // Vamos aquisitar e acumular enquanto nao acabar o tempo
        t_inicio_aquisicao = ros::Time::now();
        while( (ros::Time::now() - t_inicio_aquisicao).toSec() < t_aquisicao ){

            // Converte nuvem -> ja devia estar filtrada do outro no
            PointCloud<PointT>::Ptr nuvem_inst (new PointCloud<PointT>());
            fromROSMsg(*msg_cloud, *nuvem_inst);
            // Converte odometria
            q.x() = (float)msg_odom->pose.pose.orientation.x;
            q.y() = (float)msg_odom->pose.pose.orientation.y;
            q.z() = (float)msg_odom->pose.pose.orientation.z;
            q.w() = (float)msg_odom->pose.pose.orientation.w;
            offset << msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z;
            // Converte imagem -> somente na hora de salvar a principio essa merda

            // Escuta transformacao entre frames - a principio fixa
            tf::StampedTransform trans;
            try
            {
                p_listener->waitForTransform("camera_rgb_optical_frame", "zed_current_frame", msg_cloud->header.stamp, ros::Duration(3.0));
                p_listener->lookupTransform( "camera_rgb_optical_frame", "zed_current_frame", msg_cloud->header.stamp, trans);
            }
            catch (tf::TransformException& ex){
                return;
            }
            Eigen::Affine3d eigen_trf;
//            tf::transformTFToEigen(trans, eigen_trf); ARRUMAR AQUI!!!!
            // Transforma nuvem para o frame da ASTRA, camera_rgb_optical_frame, e para a odometria medida pela ZED no momento
            transformPointCloud<PointT>(*nuvem_inst, *nuvem_inst, eigen_trf);
            transformPointCloud<PointT>(*nuvem_inst, *nuvem_inst, offset, q);
            // Acumula na nuvem parcial, enquanto dentro do tempo, depois acumula ela de forma correta
            *acumulada_parcial += *nuvem_inst;

        } // fim do while -> contagem de tempo

        // Nao podemos mais acumular enquanto o botao nao chamar de novo
        set_inicio_acumulacao(false);

        // Acumular com a global, a depender se primeira vez ou nao
        registra_global_icp(acumulada_parcial, q, offset);

        // Salva os dados na pasta do projeto

    } // fim do if -> acumular ou nao

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::salva_dados_parciais(PointCloud<PointT>::Ptr cloud,
                                      Eigen::Quaternion<float> rot,
                                      Eigen::Vector3f offset,
                                      sensor_msgs::ImageConstPtr &imagem){

}
///////////////////////////////////////////////////////////////////////////////////////////
/// Sets
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::set_inicio_acumulacao(bool flag){
    realizar_acumulacao = flag;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::set_primeira_vez(bool flag){
    primeira_vez = flag;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::set_tempo_aquisicao(float t){
    t_aquisicao = t;
}

} // Fim do namespace handset_gui
