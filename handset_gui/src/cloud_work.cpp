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
    temp_nvm                       = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

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
    string file = "file://"+std::string(home)+"/handsets_ws/src/Cameras_GRIn/astra_calibrada/calib/astra2.yaml";
    camera_info_manager::CameraInfoManager caminfo(nh_, "astra", file);
    astra_model.fromCameraInfo(caminfo.getCameraInfo());

    // Inicio do contador de imagens capturadas
    contador_imagens = 0;

    // Quantas nuvens acumular a cada captura
    n_nuvens_instantaneas = 2; // Inicia aqui, mas vai pegar do GUI

    // Por garantia iniciar aqui tambem
    set_inicio_acumulacao(false);
    set_primeira_vez(true);

    // Inicio do mutex de acumulacao - verdadeiro se estamos acumulando
    mutex_acumulacao = false;

    // Definicao de rotacao fixa entre frame da ASTRA e da ZED
    Eigen::Matrix3f matrix;
    matrix = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::Quaternion<float> rot_temp(matrix);
    rot_astra_zed = rot_temp.inverse();
    offset_astra_zed << -0.001, -0.090, -0.025;

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
        /// Fazendo de forma simplificada a principio
        ROS_INFO("Comecando o ICP...");
        // Transformacao atual em forma matricial
        Eigen::Matrix4f T_atual = qt2T(rot, offset);
        if(mutex_acumulacao == 0){
            mutex_acumulacao = 1;
            // Criacao do calculador de icp e resultado final sobre a propria acumulada
            pcl::IterativeClosestPoint<PointT, PointT> icp;
            icp.setUseReciprocalCorrespondences(true);
            icp.setMaximumIterations(100);
//            icp.setMaxCorrespondenceDistance(0.5);
            icp.setRANSACIterations(100);
//            icp.setTransformationEpsilon(1*1e-9);
//            icp.setEuclideanFitnessEpsilon(1*1e-10);
            icp.setInputSource(parcial);
            icp.setInputTarget(acumulada_global);
            ROS_INFO("Alinhando nuvens...");
            pcl::PointCloud<PointT> final;
            icp.align(final, T_atual);
            *acumulada_global += final;
            cout << "\n\nResultado do alinhamento: " << icp.hasConverged() << endl;
            cout << "\nScore: " << icp.getFitnessScore();
            // Liberar mutex e ver o resultado da acumulacao no rviz
            mutex_acumulacao = 0;
        }
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
    // Se podemos iniciar a acumular (inicialmente botao da GUI setou essa flag)
    if(realizar_acumulacao){
        Eigen::Quaternion<float> q;
        Eigen::Vector3f offset;

        // Converte odometria -> so uma vez no inicio para evitar drifts
        q.x() = (float)msg_odom->pose.pose.orientation.x;
        q.y() = (float)msg_odom->pose.pose.orientation.y;
        q.z() = (float)msg_odom->pose.pose.orientation.z;
        q.w() = (float)msg_odom->pose.pose.orientation.w;
        offset << msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z;

        // Reiniciando nuvens parciais
        acumulada_parcial->clear(); acumulada_parcial_frame_camera->clear(); temp_nvm->clear();
        // Vamos aquisitar e acumular enquanto nao juntar n_nuvens_instantaneas
//        ros::Time t_inicio_aquisicao = ros::Time::now();
//        while( (ros::Time::now() - t_inicio_aquisicao).toSec() < t_aquisicao ){
        int cont_nuvens = 0;
        while( cont_nuvens < n_nuvens_instantaneas ){
            // Converte nuvem -> ja devia estar filtrada do outro no
            PointCloud<PointT>::Ptr nuvem_inst (new PointCloud<PointT>());
            fromROSMsg(*msg_cloud, *nuvem_inst);
            // Acumula na parcial do frame da camera, sem transformar, para projetar depois e salvar NVM
            acumulada_parcial_frame_camera->header.frame_id = msg_cloud->header.frame_id;
            *acumulada_parcial_frame_camera += *nuvem_inst;
            // Rotacao para corrigir odometria
            transformPointCloud<PointT>(*nuvem_inst, *nuvem_inst, offset_astra_zed, rot_astra_zed);
            // Acumulacao durante a aquisicao instantanea, sem transladar com a odometria
            acumulada_parcial->header.frame_id = "odom";
            *acumulada_parcial += *nuvem_inst;
            // Atualiza contagem de nuvens durante captura
            cont_nuvens++;
        } // fim do while -> contagem de tempo

        ROS_INFO("Nuvem parcial capturada");

        // Nao podemos mais acumular enquanto o botao nao chamar de novo
        set_inicio_acumulacao(false);

//        // Transforma nuvem segundo odometria da ZED e acumula na nuvem parcial, enquanto dentro do tempo
//        transformPointCloud<PointT>(*nuvem_inst, *nuvem_inst, offset, q);

        // Acumular com a global, a depender se primeira vez ou nao
        registra_global_icp(acumulada_parcial, q, offset);

        // Salva os dados na pasta do projeto -> PARCIAIS
        this->salva_dados_parciais(acumulada_parcial, rot_astra_zed.inverse()*q, offset, msg_image);
        cout << "Dados salvos!!\n\n";

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

//    // Salva a nuvem parcial em PLY
//    pcl::io::savePLYFileASCII(arquivo_nuvem, *cloud);

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
                // Adicionar na nuvem de pontos que vai pro NVM e sera salva
                PointT ponto_temp;
                ponto_temp.x = ponto3D.x; ponto_temp.y = ponto3D.y; ponto_temp.z = ponto3D.z;
                ponto_temp.r = acumulada_parcial->points[i].r; ponto_temp.g = acumulada_parcial->points[i].g; ponto_temp.b = acumulada_parcial->points[i].b;
                temp_nvm->push_back(ponto_temp);
                // Construindo a linha do arquivo
                string linha = to_string(ponto3D.x) + " " + to_string(ponto3D.y) + " " + to_string(ponto3D.z) + " "; // PONTO XYZ
                linha = linha + to_string(acumulada_parcial->points[i].r) + " " + to_string(acumulada_parcial->points[i].g) + " " + to_string(acumulada_parcial->points[i].b) + " "; // CORES RGB
                linha = linha + "1 1 " + to_string(int(pontoProjetado.x*pontoProjetado.y)) + " "; // Indice da foto e da feature
                linha = linha + to_string(pontoProjetado.x) + " "  +to_string(pontoProjetado.y) + "\n"; // PIXEL projetado
                // Escrever no arquivo completo -> o fim da string 'linha' ja pula para a proxima linha
                std::replace(linha.begin(), linha.end(), ',', '.'); // Altera virgula por ponto
                file << linha;
            }
        }
        file << "\n\n\n0\n\n"; // 0 intermediario
        file << "#The last part of NVM files points to the PLY files\n"; // Fim das contas
        file << "#The first number is the number of associated PLY files\n";
        file << "#each following number gives a model-index that has PLY\n";
        file << "0";
    } // fim do if is open
    file.close(); // Fechar para nao ter erro

    // Calcular normais para a nuvem
    cout << "\nCalculando normais...\n\n";
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud(temp_nvm);
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setKSearch(12); // Quantos vizinhos procurar
    ne.compute(*cloud_normals);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr final_nvm (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*temp_nvm, *cloud_normals, *final_nvm);
    // Filtrar normais que tenham nan
    pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator it;
    for(it=final_nvm->begin(); it!=final_nvm->end(); ++it){
        if(it->normal_x != it->normal_x) // porque o nan eh maluco
            final_nvm->erase(it);
    }

    // Salvar nuvem em arquivo .ply
    pcl::io::savePLYFileASCII(arquivo_nuvem, *final_nvm);
    cout << "\nNuvem salva!\n\n\n";

} // Fim da funcao de salvar arquivos parciais
///////////////////////////////////////////////////////////////////////////////////////////
std::string Cloud_Work::escreve_linha_imagem(std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q){
    std::string linha = nome;
    // Adicionar foco
    linha = linha + " " + to_string(astra_model.fx());
    // Adicionar quaternion
    linha = linha + " " + std::to_string(q.w()) + " " + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z());
    // Adicionar centro da camera
    linha = linha + " " + std::to_string(C(0, 0)) + " " + std::to_string(C(1, 0)) + " " + std::to_string(C(2, 0));
    // Adicionar distorcao radial (crendo 0) e 0 final
    linha = linha + " 0 0";
    // Escrever isso -> NAO PULA LINHA, SO RETORNA O CONTEUDO
    std::replace(linha.begin(), linha.end(), ',', '.');
    return linha;
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf Cloud_Work::calcula_centro_camera(Eigen::Quaternion<float> q, Eigen::Vector3f offset){
    Eigen::MatrixXf C(3, 1);
    Eigen::MatrixXf R = q.matrix();
    Eigen::MatrixXf t(3,1);
    t = offset;
    C = -R.transpose()*t; // Tirei o sinal negativo porque achei que estava dando o contrario

    cout << "\nCentro da camera:\n" << C << endl;
    return C;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::salvar_acumulada(){
    // Nome da pasta para salvar
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/teste/";
    std::string arquivo_nuvem = pasta+"nuvem_final.ply";

    // A principio so salvar a nuvem
    ROS_INFO("Salvando nuvem acumulada.....");
    pcl::io::savePLYFileASCII(arquivo_nuvem, *acumulada_global);
    ROS_INFO("Nuvem salva na pasta correta!!");
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
void Cloud_Work::set_n_nuvens_aquisicao(float t){
    n_nuvens_instantaneas = t;
}

} // Fim do namespace handset_gui
