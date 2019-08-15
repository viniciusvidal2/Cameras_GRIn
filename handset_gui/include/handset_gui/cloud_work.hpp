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
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
#include <yaml-cpp/yaml.h>
#include <unordered_set>

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
#include <pcl/filters/extract_indices.h>

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
#include <QString>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/connection.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/bag_player.h>

using namespace pcl::io;
using namespace pcl::geometry;
using namespace message_filters;
using namespace nav_msgs;
using namespace cv;
using namespace pcl;

namespace handset_gui {

class Cloud_Work : public QThread
{
    Q_OBJECT
public:
    /// Definicoes ///
    typedef PointXYZRGB PointC;
    typedef PointXYZRGBNormal PointCN;

    Cloud_Work(int argc, char** argv, QMutex*);
    virtual ~Cloud_Work();
    void init();
    void set_inicio_acumulacao(bool flag);
    void set_primeira_vez(bool flag);
    void set_n_nuvens_aquisicao(float t);
    void set_nome_bag(QString nome);
    void set_dados_online_offline(bool vejamos);
    void salvar_acumulada();
    void set_profundidade_max(float d);
    void reiniciar();
    Eigen::Matrix4f icp(const PointCloud<PointC>::Ptr src, const PointCloud<PointC>::Ptr tgt, Eigen::Matrix4f T);

    void cancela_listeners(bool eai); // A janela principal vai chamar dependendo de como estiver pra rodar, online ou nao

    QMutex* mutex;

private:

    typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, Odometry> syncPolicy;
    typedef Synchronizer<syncPolicy> Sync;
    boost::shared_ptr<Sync> sync;
    message_filters::Connection funcionou_porra;

    /// Procedimentos internos ///
    void filter_grid(PointCloud<PointC>::Ptr cloud, float leaf_size);
    void filter_grid(PointCloud<PointXYZ>::Ptr cloud, float leaf_size); // Overload da funcao para usar no ICP
    void filterForNan(PointCloud<PointC>::Ptr cloud, PointCloud<PointXYZ>::Ptr pixels);
    void passthrough(PointCloud<PointC>::Ptr cloud, PointCloud<PointXYZ>::Ptr foto, std::string field, float min, float max);
    void removeColorFromPoints(PointCloud<PointC>::Ptr in, PointCloud<PointXYZ>::Ptr out);
    void callback_acumulacao(const sensor_msgs::ImageConstPtr &msg_ast_image, const sensor_msgs::ImageConstPtr &msg_zed_image,
                             const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const sensor_msgs::PointCloud2ConstPtr &msg_pixels,
                             const OdometryConstPtr &msg_odom);
    void registra_global_icp(PointCloud<PointC>::Ptr parcial, Eigen::Quaternion<float> rot, Eigen::Vector3f offset);
    void salva_dados_parciais(PointCloud<PointC>::Ptr cloud, const sensor_msgs::ImageConstPtr &imagem_zed, const sensor_msgs::ImageConstPtr &imagem_ast, PointCloud<PointXYZ>::Ptr nuvem_pix_total, PointCloud<PointC>::Ptr nuvem_bat);
    void salva_nvm_acumulada(std::string nome);
    void publica_nuvens();
    void calcula_normais_com_pose_camera(PointCloud<PointCN>::Ptr acc, PointCloud<PointC> cloud, Eigen::MatrixXf C, int K);
    void comparaSift(cv_bridge::CvImagePtr astra, cv_bridge::CvImagePtr zed, PointCloud<PointC>::Ptr cloud);
    void resolveAstraPixeis(PointCloud<PointXYZ>::Ptr pixeis, PointCloud<PointC>::Ptr nuvem_total_bat, cv_bridge::CvImagePtr zed);
    void updateRTFromSolvePNP(std::vector<cv::Point2f> imagePoints, std::vector<cv::Point3f> objectPoints, cv_bridge::CvImagePtr zed);
    void printT(Eigen::Matrix4f T);
    std::string escreve_linha_imagem(float foco, std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q);
    Eigen::Matrix4f qt2T(Eigen::Quaternion<float> rot, Eigen::Vector3f offset);
    Eigen::MatrixXf calcula_centro_camera(Eigen::Quaternion<float> q, Eigen::Vector3f offset);

    void ProcessaOffline();

    /// Variaveis ///
    int init_argc;
    char** init_argv;
    // Para iniciar a acumulacao, eh finalizada por tempo depois
    bool realizar_acumulacao;
    // Analisa se primeira vez ou nao para controlar acumulacao
    bool primeira_vez;
    // Foco da ZED e da ASTRA global
    float fzed, fastra;
    // Nuvem acumulada total
    PointCloud<PointC>::Ptr acumulada_global;
    // Nuvem acumulada parcial atual e anterior, para cada intervalo de aquisicao
    PointCloud<PointC>::Ptr acumulada_parcial_frame_camera;
    PointCloud<PointC>::Ptr acumulada_parcial;
    PointCloud<PointC>::Ptr acumulada_parcial_anterior;
    PointCloud<PointC>::Ptr temp_nvm;
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
    // Caminho completo para a bag de dados
    QString caminho_bag;
    // Bool para controle de se vamos por bag ou online
    bool vamos_de_bag;
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
    Eigen::Matrix4f T_astra_zed;
    // Contador de nuvens repetidas capturadas
    int n_nuvens_instantaneas;
    // Profundidade do filtro obtida da GUI
    float profundidade_max;
    // Mesh final triangularizada
    PolygonMesh mesh_acumulada;
    // Struct para guardar cada nuvem com seu ponto de vista respectivo e calcular normais
    struct nuvem_pose{
        nuvem_pose() {}
        PointCloud<PointC> nuvem;
        Eigen::MatrixXf centro_camera; // Centro da camera no espaço
    };
    std::vector<nuvem_pose> np;
    // Keypoints filtrados globais
    std::vector<cv::KeyPoint> goodKeypointsLeft;  // astra
    std::vector<cv::KeyPoint> goodKeypointsRight; // zed
    // Matriz de pose da camera final, estimada pela correspondencia de pontos depth-astra-zed, Juliano
    Eigen::Matrix4f T_depth_astra_zed;
    // Estrutura para guardar os dados da camera interessantes aqui apos otimizados por bat
    struct camera{
        camera() {}
        float foco;
        Eigen::Matrix4f T;
    };
    // Variaveis para guardar os pontos 2D corretos e os 3D correspondentes
    std::vector<Point2f> imagePointsZed;
    std::vector<Point3f> objectPointsZed;
    // Funcao de otimizacao do bat
    camera bat(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, float foco_est, Eigen::Vector2f c_img, bool &valid);
    float  fob(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, Eigen::MatrixXf bat, Eigen::Vector2f c_est, Eigen::MatrixXf range);
};

}
