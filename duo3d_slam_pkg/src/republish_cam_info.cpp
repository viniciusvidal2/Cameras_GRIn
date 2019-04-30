#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

using namespace std;

ros::Publisher pub_imagem;
ros::Publisher pub_cam_info;

sensor_msgs::CameraInfo mensagem_info;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    mensagem_info.header.stamp = msg->header.stamp;

    pub_imagem.publish(*msg);
    pub_cam_info.publish(mensagem_info);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "republish_cam_info");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~"); // Frescura da camera

  // Lendo os parametros passados
  string imagem_entrada, imagem_saida, cam_info_saida, nome_camera, arquivo_calibracao;
  n_.getParam("image_topic_in" , imagem_entrada    );
  n_.getParam("image_topic_out", imagem_saida      );
  n_.getParam("cam_info_out"   , cam_info_saida    );
  n_.getParam("camera_name"    , nome_camera       );
  n_.getParam("file"           , arquivo_calibracao);

  ros::Subscriber sub = nh.subscribe(imagem_entrada, 1, imageCallback);
  pub_imagem   = nh.advertise<sensor_msgs::Image     >(imagem_saida  , 10);
  pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>(cam_info_saida, 10);

  // Ler o arquivo e criar a mensagem constante a partir dele
  camera_info_manager::CameraInfoManager cam_info_man(n_, nome_camera, arquivo_calibracao);
  mensagem_info = cam_info_man.getCameraInfo();
  mensagem_info.header.frame_id = "duo3d/camera_frame"; // mesmo para as duas

  ros::spin();

  return 0;
}
