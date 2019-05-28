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

bool primeira_vez = true;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis e definicoes globais
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Definicoes
typedef PointXYZRGB       PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> syncPolicy;

void callback(const sensor_msgs::ImageConstPtr& msg_img,
              const sensor_msgs::PointCloud2ConstPtr& cloud){
    cv_bridge::CvImagePtr cv_ptr_img;
    cv_ptr_img = cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8);

    if(primeira_vez){

        ROS_INFO("Gravando a nuvem de pontos e imagem colorida");

        cv::Mat image_gray;
        cv::cvtColor(cv_ptr_img->image, image_gray, cv::COLOR_BGR2GRAY);

        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        cv::imwrite("/home/grin/Desktop/calibrando/gray.png", image_gray, compression_params);
        cv::imwrite("/home/grin/Desktop/calibrando/rgb.png" , cv_ptr_img->image, compression_params);

        PointCloud<PointT>::Ptr xyzrgb (new PointCloud<PointT>());
        fromROSMsg(*cloud, *xyzrgb);
        vector<int> indices;
        removeNaNFromPointCloud(*xyzrgb, *xyzrgb, indices);

        PointCloud<PointXYZ>::Ptr xyz (new PointCloud<PointXYZ>());
        for(int i=0; i < xyzrgb->size(); i++){
            PointXYZ ponto;
            ponto.x = xyzrgb->points[i].x;
            ponto.y = xyzrgb->points[i].y;
            ponto.z = xyzrgb->points[i].z;

            xyz->push_back(ponto);
        }

        ROS_INFO("Gravando nuvem...");

        pcl::io::savePLYFileASCII("/home/grin/Desktop/calibrando/nuvem.ply", *xyz);
        pcl::io::savePLYFileASCII("/home/grin/Desktop/calibrando/nuvem_total.ply", *xyzrgb);

        primeira_vez = false;

        ROS_INFO("Nuvens gravadas, falta a imagem infravermelha.");

    }

}

void callback_ir(const sensor_msgs::ImageConstPtr& msg){

    if(!primeira_vez){

        ROS_INFO("Gravando imagem infravermelha...");

        cv_bridge::CvImagePtr cv_ptr_img;
        cv_ptr_img = cv_bridge::toCvCopy(msg);
        Mat im = cv_ptr_img->image.clone();
        im.convertTo(im, CV_8U, 1/256.0);

        vector<int> compression_params;
        compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
        cv::imwrite("/home/grin/Desktop/calibrando/ir.png", im, compression_params);

        ROS_INFO("Imagem infravermelha gravada, acabou");

        ros::shutdown();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrar_astra_online");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image>       img_sub  (nh, "/camera/rgb/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloudsub (nh, "/camera/depth/points" , 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), img_sub, cloudsub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::Subscriber irsub = nh.subscribe("/camera/ir/image", 100, callback_ir);

    while(ros::ok()){
        if(primeira_vez == false){

            img_sub.unsubscribe();
            cloudsub.unsubscribe();

        }
        ros::spinOnce();
    }

}
