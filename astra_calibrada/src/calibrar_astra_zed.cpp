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
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>


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
using namespace cv;

int cont = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB       PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy;

void callback(const sensor_msgs::ImageConstPtr& msg_astra,
              const sensor_msgs::ImageConstPtr& msg_zed){
    // Convertendo para objetos cv
    cv_bridge::CvImagePtr astra_cv, zed_cv;
    astra_cv = cv_bridge::toCvCopy(msg_astra, sensor_msgs::image_encodings::BGR8);
    zed_cv   = cv_bridge::toCvCopy(msg_zed  , sensor_msgs::image_encodings::BGR8);

    // Atualiza contador
    cont++;

    // Convertendo a imagem para varias resolucoes
    Mat zed_peq, zed_normal, astra_normal, astra_grande;
    zed_normal = zed_cv->image;
    astra_normal = astra_cv->image;
    cv::Size sz, sa;
    sz.height = zed_normal.rows  ; sz.width = zed_normal.cols;
    sa.height = astra_normal.rows; sa.width = astra_normal.cols;
    cv::resize(zed_normal  , zed_peq     , sa);
    cv::resize(astra_normal, astra_grande, sz);

    // Gravando as imagens na pasta certa
    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    char* home = getenv("HOME");

    ROS_INFO("Capturando fotos %d...", cont);

    cv::imwrite(std::string(home)+"/Desktop/calibrando/astranormal/"+std::to_string(cont)+".png", astra_normal, compression_params);
    cv::imwrite(std::string(home)+"/Desktop/calibrando/astragrande/"+std::to_string(cont)+".png", astra_grande, compression_params);
    cv::imwrite(std::string(home)+"/Desktop/calibrando/zedpeq/"+std::to_string(cont)+".png", zed_peq, compression_params);
    cv::imwrite(std::string(home)+"/Desktop/calibrando/zednormal/"+std::to_string(cont)+".png", zed_normal, compression_params);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrar_astra_zed");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgbsub   (nh, "/camera/rgb/image_raw"     , 10);
    message_filters::Subscriber<sensor_msgs::Image> zedsub   (nh, "/zed/left/image_rect_color", 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), rgbsub, zedsub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

//    ros::Rate r(1);
    while(ros::ok()){
        ros::spinOnce();
//        r.sleep();
    }

}
