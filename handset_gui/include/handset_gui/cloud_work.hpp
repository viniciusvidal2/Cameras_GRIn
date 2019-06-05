#ifndef QTIMAGENODE_H
#define QTIMAGENODE_H

#endif // QTIMAGENODE_H

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
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
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/geometry/mesh_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/texture_mapping.h>

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
#include <iostream>
#include <ros/ros.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
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
using namespace pcl::io;
using namespace pcl::geometry;
using namespace message_filters;
using namespace nav_msgs;
using namespace cv;

namespace handset_gui {

class Cloud_Work : public QThread
{
    Q_OBJECT
public:
    /// Definicoes ///
    typedef PointXYZRGB PointT;
    typedef PointXYZRGBNormal PointTN;

    Cloud_Work(int argc, char** argv, QMutex*);
    virtual ~Cloud_Work();
    void init();
    void set_inicio_acumulacao(bool flag);
    void set_primeira_vez(bool flag);
    void set_n_nuvens_aquisicao(float t);
    void salvar_acumulada();
    void set_profundidade_max(float d);
    void reiniciar();
    Eigen::Matrix4f icp(const PointCloud<PointT>::Ptr src, const PointCloud<PointT>::Ptr tgt, Eigen::Matrix4f T);

    QMutex* mutex;



private:
    /// Definicoes ///
    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, Odometry> syncPolicy;
    typedef Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    /// Procedimentos internos ///
    void filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size);
    void filter_grid(PointCloud<PointXYZ>::Ptr cloud, float leaf_size); // Overload da funcao para usar no ICP
    void passthrough(PointCloud<PointT>::Ptr cloud, std::string field, float min, float max);
    void removeColorFromPoints(PointCloud<PointT>::Ptr in, PointCloud<PointXYZ>::Ptr out);
    void callback_acumulacao(const sensor_msgs::ImageConstPtr &msg_image, const sensor_msgs::ImageConstPtr &msg_zed_image, const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const OdometryConstPtr &msg_odom);
    void registra_global_icp(PointCloud<PointT>::Ptr parcial, Eigen::Quaternion<float> rot, Eigen::Vector3f offset);
    void salva_dados_parciais(PointCloud<PointT>::Ptr cloud, Eigen::Quaternion<float> rot, Eigen::Vector3f offset, const sensor_msgs::ImageConstPtr &imagem, const sensor_msgs::ImageConstPtr &imagem_zed);
    void salva_nvm_acumulada(std::string nome);
    void publica_nuvens();
    void calculateNormalsAndConcatenate(PointCloud<PointT>::Ptr cloud, PointCloud<PointTN>::Ptr cloud2, int K);
    void calculateNormalsAndConcatenate(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointNormal>::Ptr cloud2, int K); // Overload da funcao para usar no ICP
    void saveMesh(std::string nome);
    void triangulate();
    std::string escreve_linha_imagem(std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q);
    Eigen::Matrix4f qt2T(Eigen::Quaternion<float> rot, Eigen::Vector3f offset);
    Eigen::MatrixXf calcula_centro_camera(Eigen::Quaternion<float> q, Eigen::Vector3f offset);

    /// Variaveis ///
    int init_argc;
    char** init_argv;
    // Para iniciar a acumulacao, eh finalizada por tempo depois
    bool realizar_acumulacao;
    // Analisa se primeira vez ou nao para controlar acumulacao
    bool primeira_vez;
    // Nuvem acumulada total
    PointCloud<PointT>::Ptr acumulada_global;
    // Nuvem acumulada parcial atual e anterior, para cada intervalo de aquisicao
    PointCloud<PointT>::Ptr acumulada_parcial_frame_camera;
    PointCloud<PointT>::Ptr acumulada_parcial;
    PointCloud<PointT>::Ptr acumulada_parcial_anterior;
    PointCloud<PointT>::Ptr temp_nvm;
    // Matriz de transformacao para a aproximacao ICP
    Eigen::Matrix4f T_icp;
    // Transformacoes para calculo de movimento relativo e otimizacao do chute inicial do ICP
    Eigen::Matrix4f T_zed_anterior, T_zed_atual, T_zed_rel;
    Eigen::Matrix4f T_corrigida, T_chute_icp;
    // Publicadores para nuvem parcial e total;
    ros::Publisher pub_global;
    ros::Publisher pub_parcial;
    // Pasta onde sera guardado o projeto para o MART e tudo o mais
    std::string pasta;
    // Modelo da camera astra
    image_geometry::PinholeCameraModel astra_model;
    /// ARQUIVO NVM
    // Contador de imagens capturadas
    int contador_imagens;
    // Matriz de pontos
    Eigen::MatrixXd pontos_nvm;
    // Vetor com nomes de arquivos - arquivo nuvem acumulada
    std::vector<std::string> acumulada_imagens;
    /// ARQUIVO NVM - FIM
    // Mutex de acumulacao
    bool mutex_acumulacao;
    // Transformacao fixa entre o frame da astra e da zed
    Eigen::Vector3f offset_astra_zed;
    Eigen::Quaternion<float> rot_astra_zed;
    // Contador de nuvens repetidas capturadas
    int n_nuvens_instantaneas;
    // Profundidade do filtro obtida da GUI
    float profundidade_max;
    // Mesh final triangularizada
    PolygonMesh mesh_acumulada;
};

}
