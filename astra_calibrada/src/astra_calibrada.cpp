/////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////            Grupo de Robótica Inteligente - GRIn                    ///////////////////
//////////////                      Projeto Virtual                               ///////////////////
//////////////               Luiz Augusto Zillmann da Silva                       ///////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////// Sincronizando nuvem Astra ///////////////////////////////////////////
///                                                                                               ///

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Diversos
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <ctime>
#include <fstream>
#include <iostream>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/io/ply_io.h>

// EIGEN
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

// Messages
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>


using namespace pcl;
using namespace std;
using namespace tf;
using namespace message_filters;
using namespace nav_msgs;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB       PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, Odometry> syncPolicy;

// Variaveis
PointCloud<PointT>::Ptr nuvem_colorida;
tf::TransformListener *p_listener;
ros::Publisher pub_cloud;
ros::Publisher pub_odom;
ros::Publisher pub_img;
ros::Publisher pub_zed;
// Matriz intrinseca K para Depth cam
Eigen::Matrix3f K1; //  MATLAB

float fxd, fyd, Cxd, Cyd;// = getIntrinsicsFromK(K, "D");

// Matriz intrinseca K para RGB cam
Eigen::Matrix3f K2;

float fxrgb, fyrgb, Cxrgb, Cyrgb;// = getIntrinsicsFromK(K, "D");

// Calculando Params. Extrisecos
Eigen::MatrixXf RT(3,4);

Eigen::MatrixXf P;

Eigen::Matrix3f F;

// Resolucao da nuvem
int resolucao;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Filtro para distâncias
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(pcl::PointCloud<PointT>::Ptr in, std::string field, float min, float max){
    pcl::PassThrough<PointT> ps;
    ps.setInputCloud(in);
    ps.setFilterFieldName(field);
    ps.setFilterLimits(min, max);

    ps.filter(*in);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Voxel Grid
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void voxelgrid_nuvem(PointCloud<PointT>::Ptr in, float lf){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(lf, lf, lf);
    grid.setInputCloud(in);
    grid.filter(*in);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Removendo Outliers
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
    StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(in);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(deviation);
    sor.filter(*in);
}

void callback(const sensor_msgs::ImageConstPtr& msg_rgb,
              const sensor_msgs::ImageConstPtr& msg_zed,
              const sensor_msgs::ImageConstPtr& msg_depth,
              const OdometryConstPtr& msg_odo)
{
    cv_bridge::CvImagePtr cv_ptr_d;
    cv_bridge::CvImagePtr cv_ptr_rgb;
    cv_ptr_d   = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    cv_ptr_rgb = cv_bridge::toCvCopy(msg_zed,   sensor_msgs::image_encodings::RGB8 );

    sensor_msgs::PointCloud2 msg_cor;

    int depthHeight = cv_ptr_d->image.rows;  //cv_image_d.rows;
    int depthWidth  = cv_ptr_d->image.cols;  //cv_image_d.cols;

    cv::Vec3b intensity;

    float x, y, z;
    PointT current_point;
    resolucao = resolucao > 0 ? resolucao : 2;
    for(int v = 0; v < depthHeight; v=v+resolucao){
        for(int u = 0; u < depthWidth; u=u+resolucao){
            z = cv_ptr_d->image.at<short int>(v, u);
            if(z!=0){ // eliminando pontos de prof. zero
                x = ((u - Cxd)*z)/fxd;
                y = ((v - Cyd)*z)/fyd;

                // Reprojetando...
                Eigen::MatrixXf X_(4,1);
                X_ << x,
                      y,
                      z,
                      1;
                Eigen::MatrixXf X = P*X_;

                X = X/X(2,0);

                // Adicionando o ponto sem conferencia da matriz fundamental
                if( floor(X(0,0)) >= 0 && floor(X(0,0)) < depthWidth && floor(X(1,0)) >= 0 && floor(X(1,0)) < depthHeight){
                    float s = 1000;
                    current_point.z = z/s;
                    current_point.x = x/s;
                    current_point.y = y/s;

                    intensity = cv_ptr_rgb->image.at<cv::Vec3b>(floor(X(1,0)), floor(X(0,0)));
                    current_point.r = intensity.val[0];
                    current_point.g = intensity.val[1];
                    current_point.b = intensity.val[2];

                    nuvem_colorida->push_back(current_point);
                }
            }
        } // fim do for u
    } // fim do for v

    nuvem_colorida->header.frame_id = "camera_rgb_optical_frame";

    // Mensagem Nuvem
    toROSMsg(*nuvem_colorida, msg_cor);
    msg_cor.header.frame_id = nuvem_colorida->header.frame_id;
    msg_cor.header.stamp = ros::Time::now();

    // Mensagem Odometria
    Odometry msg_odo2 = *msg_odo;
    msg_odo2.header.stamp = msg_cor.header.stamp;

    // Mensagem imagem para sincronizar a frente
    sensor_msgs::Image msg_rgb2 = *msg_rgb;
    msg_rgb2.header.stamp = msg_cor.header.stamp;

    // Mensagem de imagem da ZED tambem para sincronizar depois
    sensor_msgs::Image msg_zed2 = *msg_zed;
    msg_zed2.header.stamp = msg_cor.header.stamp;

    // Publicando tudo junto
    pub_odom.publish(msg_odo2);
    pub_cloud.publish(msg_cor);
    pub_img.publish(msg_rgb2);
    pub_zed.publish(msg_zed2);

    nuvem_colorida->clear();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nuvem_astra_calibrada");

    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    // Variáveis
    K1 <<  582.9937,   -2.8599,  308.9297, // CAMERA IR, PROFUNDIDADE
            0,  582.6690,  246.1634,
            0,         0,    1.0000;

    fxd = K1(0,0);
    fyd = K1(1,1);
    Cxd = K1(0,2);
    Cyd = K1(1,2);

//    K2 <<   525.1389,    1.4908,  324.1741, // CAMERA RGB
//            0,  521.6805,  244.8827,
//            0,         0,    1.0000;
    K2 <<   1475.0,      0,  963.9924, // CAMERA ZED
                 0, 1475.2,  588.7955,
                 0,      0,    1.0000;

    fxrgb = K2(0,0); //cout << fxrgb << endl;
    fyrgb = K2(1,1);
    Cxrgb = K2(0,2);
    Cyrgb = K2(1,2);

//    RT <<    0.999812767546507,  -0.013049436148855,  -0.014287829337980, -25.056528502229849, // MATLAB -25.056528502229849
//            0.012849751269106,   0.999819711122249,  -0.013979597409931, -10.308878899329242, // MATLAB -35.308878899329242
//            0.014467679265050,   0.013793384922440,   0.999800194433400,  -0.890397736650369;
    RT <<   0.999812767546507,  -0.008049436148855,  -0.006587829337980, 27.3628, //49.3628
            0.008149751269106,   0.999819711122249,  -0.002779597409931, 54.0082,
            0.006567679265050,  -0.002793384922440,   0.999800194433400,  -2.5417;

    P = K2*RT;

    F << 0.000000051968032,   0.000002923620641,  -0.000171378176749,
            -0.000001735294842,   0.000001158366503,   0.048294523803484,
            -0.000673889418073,  -0.044225087946452,  -0.498521482515482;

    nuvem_colorida = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

    // Resolucao para a nuvem vinda no parametro
    n_.getParam("resolucao", resolucao);

    pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/astra_projetada", 10);
    pub_odom  = nh.advertise<Odometry>                ("/odom2"          , 10);
    pub_img   = nh.advertise<sensor_msgs::Image>      ("/astra_rgb"      , 10);
    pub_zed   = nh.advertise<sensor_msgs::Image>      ("/zed2"           , 10);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub  (nh, "/camera/rgb/image_raw"     , 10);
    message_filters::Subscriber<sensor_msgs::Image> zed_sub  (nh, "/zed/left/image_rect_color", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw"   , 10);
    message_filters::Subscriber<Odometry>           subodo   (nh, "/zed/odom"                 , 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), rgb_sub, zed_sub, depth_sub, subodo);
    sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}
