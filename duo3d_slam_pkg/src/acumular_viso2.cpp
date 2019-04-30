//Includes
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
#include <camera_info_manager/camera_info_manager.h>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/// Namespaces
using namespace pcl;
using namespace std;
using namespace message_filters;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace Eigen;
using namespace cv;

/// Definitions
typedef PointXYZRGB PointT;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicy;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Variaveis globais
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Quaternion<double> q_anterior, q_anterior_inverso, q_atual, q_relativo;
Eigen::Vector3d t_anterior, t_atual, t_relativo;
PointCloud<PointT>::Ptr cloud, cloud_tf;
image_geometry::PinholeCameraModel model_stereo;

PointCloud<PointT>::Ptr acumulada;

bool primeira_vez = true;

// Publicador
ros::Publisher acumulada_pub;

// Itens vindos do launch
std::string camera_calibration_yaml;
std::string cloud_topic_in, cloud_topic_out, odom_topic_in;
double overlap_rate;

// Overlap de 0 a 1 de quanto ficaria diferente
float overlap_0_a_1;

// Contagem de pontos projetados no frame de referencia
int n_pontos_fora_da_imagem_referencia = 0;

// Tamanho da imagem de entrada, nao precisa da imagem em si aqui
cv::Size image_size_stereo;

// Distancia onde ja se considera uma nova nuvem
float thresh_dist = 10.0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Funcoes para trabalho da nuvem
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_color(PointCloud<PointT>::Ptr cloud_in){

  // Try to clear white and blue points from sky, and green ones from the grass, or something close to it
  int rMax = 210;
  int rMin = 0;
  int gMax = 210;
  int gMin = 0;
  int bMax = 210;
  int bMin = 0;

  ConditionAnd<PointT>::Ptr color_cond (new ConditionAnd<PointT> ());
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("r", ComparisonOps::LT, rMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("r", ComparisonOps::GT, rMin)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("g", ComparisonOps::LT, gMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("g", ComparisonOps::GT, gMin)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("b", ComparisonOps::LT, bMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("b", ComparisonOps::GT, bMin)));

  // build the filter
  ConditionalRemoval<PointT> condrem (color_cond);
  condrem.setInputCloud (cloud_in);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud_in);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void passthrough(PointCloud<PointT>::Ptr in, std::string field, float min, float max){
  PassThrough<PointT> ps;
  ps.setInputCloud(in);
  ps.setFilterFieldName(field);
  ps.setFilterLimits(min, max);

  ps.filter(*in);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void remove_outlier(PointCloud<PointT>::Ptr in, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*in);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void filter_grid(PointCloud<PointT>::Ptr in, float lf){
  VoxelGrid<PointT> grid;
  grid.setInputCloud(in);
  grid.setLeafSize(lf, lf, lf);
  grid.filter(*in);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Iniciar o modelo da camera
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void inicia_modelo_camera(camera_info_manager::CameraInfoManager &cam){
  sensor_msgs::CameraInfo ci = cam.getCameraInfo();
  model_stereo.fromCameraInfo(ci);
  image_size_stereo = model_stereo.fullResolution(); // Armazena ja o tamanho da imagem pra nao ter erro ali nos testes
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Retornar transformacao relativa entre os instantes
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calcula_transformacao_relativa(){
  q_anterior_inverso = q_anterior.inverse();
  q_relativo = q_anterior_inverso*q_atual;
  t_relativo = q_anterior_inverso*(t_atual - t_anterior);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Acumulador de nuvens boas
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void acumular(PointCloud<PointT>::Ptr temp, Eigen::Quaternion<double> q, Eigen::Vector3d t){

  transformPointCloud<PointT>(*temp, *temp, t, q);

  *acumulada += *temp;
  ROS_INFO("Tamanho da nuvem acumulada: %d", acumulada->points.size());
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Callback para evitar repeticao da nuvem por projecao na imagem anterior
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop_closure_callback(const sensor_msgs::PointCloud2ConstPtr& msg_ptc,
                           const nav_msgs::OdometryConstPtr& msg_odo
                           ){
  // Ler nuvem de pontos
  fromROSMsg(*msg_ptc, *cloud);

  // Remover NaN
  vector<int> indicesNAN;
  removeNaNFromPointCloud(*cloud, *cloud, indicesNAN);

  // Vamos publicar a nuvem de saida? a principio nao, convencer do contrario com o overlap
  bool podemos_publicar = false;

  /// LOOP de transformacao e PROJECAO sobre a nuvem
  if(!primeira_vez){
    // Pega rotacao e translacao atual
    q_atual.x() = (double)msg_odo->pose.pose.orientation.x;
    q_atual.y() = (double)msg_odo->pose.pose.orientation.y;
    q_atual.z() = (double)msg_odo->pose.pose.orientation.z;
    q_atual.w() = (double)msg_odo->pose.pose.orientation.w;
    t_atual = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    // Calcular transformacao relativa entre os instantes
    calcula_transformacao_relativa();

    // Aqui ja checa o que foi deslocado, se passou do limite renova a pose da camera referencia e publica ao final
    if(t_relativo.norm() > thresh_dist){
      cout << "\nPassou a distancia: " << t_relativo.norm() << endl;

      podemos_publicar = true; // Podemos publicar qual seja a nuvem filtrada e as imagens

      // Armazena para a proxima iteracao a odometria de referencia
      q_anterior = q_atual;
      t_anterior = t_atual;
    }

    // Transformar camera para o ponto onde estava na iteracao anterior, e a nuvem para a iteracao de agora
    transformPointCloud<PointT>(*cloud, *cloud_tf, t_relativo, q_relativo);

    // Projeta cada ponto da nuvem na imagem e se cair fora adiciona na nuvem filtrada - OU REMOVE DA NUVEM
    cv::Point3d ponto3D;
    cv::Point2d pontoProjetado;
    n_pontos_fora_da_imagem_referencia = 0;

    if(podemos_publicar == false) {
      for(int i=0; i < cloud_tf->size(); i++){
        ponto3D.x = cloud_tf->points[i].x;
        ponto3D.y = cloud_tf->points[i].y;
        ponto3D.z = cloud_tf->points[i].z;

        pontoProjetado = model_stereo.project3dToPixel(ponto3D);
        // Se cair fora da imagem pode-se considerar ponto novo e armazena
        if(!(pontoProjetado.x > 0 && pontoProjetado.x < image_size_stereo.width &&
             pontoProjetado.y > 0 && pontoProjetado.y < image_size_stereo.height))
          n_pontos_fora_da_imagem_referencia++;
      } // Fim do for
    }

    cout << "Estamos com " << n_pontos_fora_da_imagem_referencia << " nao projetados dentre " << cloud_tf->size() << " totais." << endl;

    // Testar se reduzimos o suficiente o overlap desejado, se sim podemos atualizar a referencia e seguir com mensagens publicadas
    if( float(cloud->size() - n_pontos_fora_da_imagem_referencia) / float(cloud->size())  <  overlap_0_a_1 && podemos_publicar == false){

      cout << "\nQuantos pontos fora da imagem: " << n_pontos_fora_da_imagem_referencia << "\tTotal: " << cloud->size() << endl;
      cout << "Taxa de relacao: " << float(cloud->size() - n_pontos_fora_da_imagem_referencia) / float(cloud->size())  << endl;

      podemos_publicar = true; // Podemos publicar qual seja a nuvem filtrada e as imagens

      // Armazena para a proxima iteracao a odometria de referencia
      q_anterior = q_atual;
      t_anterior = t_atual;

    } // Fim do if de overlap

  } else {

    cout << "Primeira iteracao" << endl;

    // Segura a primeira odometria para referencia
    q_anterior.x() = (double)msg_odo->pose.pose.orientation.x;
    q_anterior.y() = (double)msg_odo->pose.pose.orientation.y;
    q_anterior.z() = (double)msg_odo->pose.pose.orientation.z;
    q_anterior.w() = (double)msg_odo->pose.pose.orientation.w;
    t_anterior = {msg_odo->pose.pose.position.x, msg_odo->pose.pose.position.y, msg_odo->pose.pose.position.z};

    q_atual = q_anterior;
    t_atual = t_anterior;

    primeira_vez = false;
    podemos_publicar = true;

  } // fim do if

  // Publica tudo aqui a depender da camera e do resultado da avaliacao do overlap
  if(podemos_publicar){

    // Cuidados com a nuvem antes de acumular
    passthrough(cloud, "z",   0, 30);
    passthrough(cloud, "x", -15, 15);
    passthrough(cloud, "y", -15, 15);

    remove_outlier(cloud, 7, 1);

    //filter_color(cloud);
    acumular(cloud, q_atual, t_atual);

    }

    // Aguardar a proxima possibilidade de acumular / publicar
    podemos_publicar = false;

  // Libera as nuvens
  cloud->clear(); cloud_tf->clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Publica continuamente nuvem acumulada
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void publicar_nuvem_atual(){

  if (acumulada->size() > 0){
    sensor_msgs::PointCloud2 msg_out;
    toROSMsg(*acumulada, msg_out);
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = acumulada->header.frame_id;

    ROS_INFO("PUBLICANDO nuvem acumulada com \t %d PONTOS.", acumulada->size());

    acumulada_pub.publish(msg_out);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main ///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv){
  ros::init(argc, argv, "acumular_viso2");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~"); // Frescura da camera

  ROS_INFO("Comecamos a calcular o overlap.");

  // Leitura de todos os parametros e topicos vindos do arquivo launch
  n_.getParam("camera_calibration_yaml", camera_calibration_yaml);
  camera_calibration_yaml = camera_calibration_yaml + std::string(".yaml");

  n_.getParam("cloud_topic_in" , cloud_topic_in );
  n_.getParam("cloud_topic_out", cloud_topic_out);

  n_.getParam("odom_topic_in"  , odom_topic_in  );

  n_.getParam("overlap_rate"   , overlap_rate   );

  // Porcentagem que a nuvem vai ter de diferenca para a referencia
  overlap_0_a_1 = overlap_rate/100.0;

  // Aloca as nuvens
  cloud      = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  cloud_tf   = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

  acumulada = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
  acumulada->header.frame_id = "odom";

  // Inicia o publicador
  acumulada_pub = nh.advertise<sensor_msgs::PointCloud2> (cloud_topic_out, 10);

  /// Inicia o modelo da camera com arquivo de calibracao
  camera_info_manager::CameraInfoManager cam_info(n_, "duo3d_left", camera_calibration_yaml);
  inicia_modelo_camera(cam_info);

  // Subscriber para a nuvem instantanea e odometria
  message_filters::Subscriber<sensor_msgs::PointCloud2> subptc(nh, cloud_topic_in, 100);
  message_filters::Subscriber<Odometry>                 subodo(nh, odom_topic_in , 100);

  // Sincroniza as leituras dos topicos
  Synchronizer<syncPolicy> sync(syncPolicy(100), subptc, subodo);
  sync.registerCallback(boost::bind(&loop_closure_callback, _1, _2));

  ros::Rate r(3);
  while (ros::ok()){
    publicar_nuvem_atual();
    r.sleep();
    ros::spinOnce();
  }

}
