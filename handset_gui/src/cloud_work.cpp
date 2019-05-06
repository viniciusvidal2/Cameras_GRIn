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
    acumulada_parcial              = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    acumulada_parcial_anterior     = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    acumulada_global               = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;
    acumulada_parcial_frame_camera = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

    // Inicia publicadores de nuvens
    pub_parcial = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_parcial", 10);
    pub_global  = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_global" , 10);

    // Inicia subscriber de TF assincrono
    p_listener = (tf::TransformListener*) new tf::TransformListener;

    // Subscribers para sincronizar
    message_filters::Subscriber<sensor_msgs::Image      > sub_imagem(nh_, "/astra_rgb"      , 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_nuvem (nh_, "/astra_projetada", 10);
    message_filters::Subscriber<Odometry                > sub_odom  (nh_, "/odom2"          , 10);
    sync.reset(new Sync(syncPolicy(10), sub_imagem, sub_nuvem, sub_odom));
    sync->registerCallback(boost::bind(&Cloud_Work::callback_acumulacao, this, _1, _2, _3));

    // Inicio do modelo da camera
    char* home;
    home = getenv("HOME");
    string file = std::string(home)+"/handsets_ws/src/Cameras_GRIn/astra_calibrada/calib/astra2.yaml";
    camera_info_manager::CameraInfoManager caminfo(nh_, "astra", file);
    astra_model.fromCameraInfo(caminfo.getCameraInfo());

    // Inicio do contador de imagens capturadas
    contador_imagens = 0;

    // Por garantia iniciar aqui tambem
    set_inicio_acumulacao(false);
    set_primeira_vez(true);

    // Rodar o no
    ros::Rate rate(2);
    while(ros::ok()){

        // Publica recorrentemente a nuvem atual
        this->publica_nuvens();

        ros::spinOnce();
        rate.sleep();
    }

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::publica_nuvens(){
    sensor_msgs::PointCloud2 parcial, global;
    // Arrumando frames
    acumulada_global->header.frame_id = acumulada_parcial->header.frame_id;
    parcial.header.frame_id = acumulada_parcial->header.frame_id;
    global.header.frame_id  = acumulada_parcial->header.frame_id;
    parcial.header.stamp = ros::Time::now();
    global.header.stamp = parcial.header.stamp;
    // Convertendo e Publicando
    toROSMsg(*acumulada_parcial, parcial);
    toROSMsg(*acumulada_global , global );
    pub_parcial.publish(parcial);
    pub_global.publish(global);
    ROS_INFO("Nuvens publicadas .....");
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
        // Reiniciando nuvens parciais
        acumulada_parcial->clear(); acumulada_parcial_frame_camera->clear();
        // Vamos aquisitar e acumular enquanto nao acabar o tempo
        ros::Time t_inicio_aquisicao = ros::Time::now();
        while( (ros::Time::now() - t_inicio_aquisicao).toSec() < t_aquisicao ){
            // Converte nuvem -> ja devia estar filtrada do outro no
            PointCloud<PointT>::Ptr nuvem_inst (new PointCloud<PointT>());
            fromROSMsg(*msg_cloud, *nuvem_inst);
            // Acumula na parcial do frame da camera, sem transformar, para projetar depois e salvar NVM
             acumulada_parcial_frame_camera->header.frame_id = msg_cloud->header.frame_id;
            *acumulada_parcial_frame_camera += *nuvem_inst;
            // Converte odometria
            q.x() = (float)msg_odom->pose.pose.orientation.x;
            q.y() = (float)msg_odom->pose.pose.orientation.y;
            q.z() = (float)msg_odom->pose.pose.orientation.z;
            q.w() = (float)msg_odom->pose.pose.orientation.w;
            offset << msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z;
            // Escuta transformacao entre frames - a principio fixa
            tf::StampedTransform trans;
            try
            {
                p_listener->waitForTransform("camera_rgb_optical_frame", "zed_current_frame", msg_cloud->header.stamp, ros::Duration(3.0));
                p_listener->lookupTransform( "camera_rgb_optical_frame", "zed_current_frame", msg_cloud->header.stamp, trans);
            }
            catch (tf::TransformException& ex){
                ROS_WARN("Nao conseguiu escutar transformada fixa e portanto nao vai dar bom.");
                return;
            }
            Eigen::Affine3d eigen_trf;
            tf::transformTFToEigen(trans, eigen_trf);
            // Transforma nuvem para o frame da ASTRA, camera_rgb_optical_frame, e para a odometria medida pela ZED no momento
            transformPointCloud<PointT>(*nuvem_inst, *nuvem_inst, eigen_trf);
            transformPointCloud<PointT>(*nuvem_inst, *nuvem_inst, offset, q);
            // Acumula na nuvem parcial, enquanto dentro do tempo, depois acumula ela de forma correta
            acumulada_parcial->header.frame_id = msg_cloud->header.frame_id;
            *acumulada_parcial += *nuvem_inst;
            cout << "Diferenca de tempo de publicacao: " << (ros::Time::now() - t_inicio_aquisicao).toSec() << endl;
            cout << "Tempo limite: " << t_aquisicao << endl;

        } // fim do while -> contagem de tempo

        // Nao podemos mais acumular enquanto o botao nao chamar de novo
        set_inicio_acumulacao(false);

        // Acumular com a global, a depender se primeira vez ou nao
        registra_global_icp(acumulada_parcial, q, offset);

        // Salva os dados na pasta do projeto -> PARCIAIS
        this->salva_dados_parciais(acumulada_parcial, q, offset, msg_image);

    } // fim do if -> acumular ou nao

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::salva_dados_parciais(PointCloud<PointT>::Ptr cloud,
                                      Eigen::Quaternion<float> rot,
                                      Eigen::Vector3f offset,
                                      const sensor_msgs::ImageConstPtr &imagem){
    // Atualiza contador de imagens - tambem usado para as nuvens parciais
    contador_imagens++;

    // Nome da pasta para salvar
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/teste/";
    std::string arquivo_imagem = pasta + std::to_string(contador_imagens) + ".jpg";
    std::string arquivo_nuvem  = pasta + std::to_string(contador_imagens) + ".ply";
    std::string arquivo_nvm    = pasta + std::to_string(contador_imagens) + ".nvm";

    // Converte e salva imagem
    cv_bridge::CvImagePtr imgptr;
    imgptr = cv_bridge::toCvCopy(imagem, sensor_msgs::image_encodings::RGB8);
    imwrite(arquivo_imagem, imgptr->image);

    // Salva a nuvem parcial em PLY
    pcl::io::savePLYFileASCII(arquivo_nuvem, *cloud);

    // Centro da camera, para escrever no arquivo NVM
    Eigen::MatrixXf C = calcula_centro_camera(rot, offset);

    // Escreve o arquivo NVM parcial, super necessario
    ofstream file(arquivo_nvm);
    if(file.is_open()){
        file << "NVM_V3\n\n";
        file << "1\n"; // Quantas imagens, sempre uma aqui
        file << escreve_linha_imagem(arquivo_imagem, C, rot); // Imagem com detalhes de camera
        file << "\n\n"+std::to_string(acumulada_parcial->size())+"\n"; // Total de pontos
        // Ponto a ponto apos ser projetado na imagem
        cv::Point3d ponto3D;
        cv::Point2d pontoProjetado;
        for(int i=0; i < acumulada_parcial_frame_camera->size(); i++){
            ponto3D.x = acumulada_parcial_frame_camera->points[i].x;
            ponto3D.y = acumulada_parcial_frame_camera->points[i].y;
            ponto3D.z = acumulada_parcial_frame_camera->points[i].z;
            pontoProjetado = astra_model.project3dToPixel(ponto3D);
            if(pontoProjetado.x > 0 && pontoProjetado.x < astra_model.fullResolution().width &&
               pontoProjetado.y > 0 && pontoProjetado.y < astra_model.fullResolution().height){
                string linha = to_string(ponto3D.x) + " " + to_string(ponto3D.y) + " " + to_string(ponto3D.z) + " "; // PONTO XYZ
                linha = linha + to_string(acumulada_parcial->points[i].r) + " " + to_string(acumulada_parcial->points[i].g) + " " + to_string(acumulada_parcial->points[i].b) + " "; // CORES RGB
                linha = linha + "1 " + to_string(int(pontoProjetado.x*pontoProjetado.y)) + " "; // Indice da foto e da feature
                linha = linha + to_string(pontoProjetado.x) + " "  +to_string(pontoProjetado.y) + "\n"; // PIXEL projetado
                // Escrever no arquivo completo -> o fim da string 'linha' ja pula para a proxima linha
                file << linha;

            }
        }
        file << "\n\n\n0\n\n"; // 0 intermediario
        file << "#The last part of NVM files points to the PLY files\n"; // Fim das contas
        file << "#The first number is the number of associated PLY files\n";
        file << "#each following number gives a model-index that has PLY\n";
        file << "0";
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
std::string Cloud_Work::escreve_linha_imagem(std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q){
    std::string linha = nome;
    // Adicionar foco
    // Adicionar quaternion
    linha = linha + " " + std::to_string(q.w()) + " " + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z());
    // Adicionar centro da camera
    linha = linha + " " + std::to_string(C(0, 0)) + " " + std::to_string(C(1, 0)) + " " + std::to_string(C(2, 0));
    // Adicionar distorcao radial (crendo 0) e 0 final
    linha = linha + " 0 0";
    // Escrever isso -> NAO PULA LINHA, SO RETORNA O CONTEUDO
    return linha;
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf Cloud_Work::calcula_centro_camera(Eigen::Quaternion<float> q, Eigen::Vector3f offset){
    Eigen::MatrixXf C(3, 1);
    Eigen::MatrixXf R = q.matrix();
    Eigen::MatrixXf t(3,1);
    t = offset;
    C = -R.transpose()*t;

    return C;
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
