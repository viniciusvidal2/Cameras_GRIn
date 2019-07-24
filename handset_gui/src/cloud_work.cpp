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
    acumulada_parcial              = (PointCloud<PointC>::Ptr) new PointCloud<PointC>;
    acumulada_parcial_anterior     = (PointCloud<PointC>::Ptr) new PointCloud<PointC>;
    acumulada_global               = (PointCloud<PointC>::Ptr) new PointCloud<PointC>;
    acumulada_parcial_frame_camera = (PointCloud<PointC>::Ptr) new PointCloud<PointC>;
    temp_nvm                       = (PointCloud<PointC>::Ptr) new PointCloud<PointC>;

    // Inicia publicadores de nuvens
    pub_parcial = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_parcial", 10);
    pub_global  = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_global" , 10);

    // Subscribers para sincronizar
    message_filters::Subscriber<sensor_msgs::Image      > sub_im_as (nh_, "/astra2"         , 10);
    message_filters::Subscriber<sensor_msgs::Image      > sub_im_zed(nh_, "/zed2"           , 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_nuvem (nh_, "/astra_projetada", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pixel (nh_, "/pixels"         , 10);
    message_filters::Subscriber<Odometry                > sub_odom  (nh_, "/odom2"          , 10);
    sync.reset(new Sync(syncPolicy(10), sub_im_as, sub_im_zed, sub_nuvem, sub_pixel, sub_odom));
    sync->registerCallback(boost::bind(&Cloud_Work::callback_acumulacao, this, _1, _2, _3, _4, _5));

    // Inicio do modelo da camera
    char* home;
    home = getenv("HOME");
    string file = "file://"+std::string(home)+"/handsets_ws/src/Cameras_GRIn/astra_calibrada/calib/astra2.yaml";
    camera_info_manager::CameraInfoManager caminfo(nh_, "astra", file);
    astra_model.fromCameraInfo(caminfo.getCameraInfo());

    // Inicio do contador de imagens capturadas
    contador_imagens = 0;

    // Por garantia iniciar aqui tambem
    set_inicio_acumulacao(false);
    set_primeira_vez(true);

    // Quantas nuvens acumular a cada captura
    n_nuvens_instantaneas = 2; // Inicia aqui, mas vai pegar do GUI

    // Inicio do mutex de acumulacao - verdadeiro se estamos acumulando
    mutex_acumulacao = false;

    // Definicao de rotacao fixa entre frame da ASTRA e da ZED -> de ASTRA->ZED A PRINCIPIO
    Eigen::Matrix3f matrix;
    matrix = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::Quaternion<float> rot_temp(matrix);
    rot_astra_zed = rot_temp.inverse(); // Aqui esta de ZED->ASTRA (nuvens)

//    offset_astra_zed << 0.048, 0.031, -0.019; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA, CORRIGIDO MART 2
//    offset_astra_zed << 0.04536, 0.027, -0.00314; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA, MATLAB
//    offset_astra_zed << 0.025, 0.020, 0; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA, MATLAB
    offset_astra_zed << 0.052, 0.01, -0.01; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, por reconfigure com imagem raw objetos proximos

    // Matriz de transformaçao que leva ASTRA->ZED, assim pode calcular posicao da CAMERA ao multiplicar por ZED->ODOM
    T_astra_zed << matrix, offset_astra_zed,
                   0, 0, 0, 1;

    // Mais uma vez aqui para nao ter erro
    set_inicio_acumulacao(false);

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
    global.header.stamp = ros::Time::now();
    // Convertendo e Publicando
    toROSMsg(*acumulada_parcial, parcial);
    toROSMsg(*acumulada_global , global );
    pub_parcial.publish(parcial);
    pub_global.publish(global);
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::filter_grid(PointCloud<PointC>::Ptr cloud, float leaf_size){
    VoxelGrid<PointC> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::filter_grid(PointCloud<PointXYZ>::Ptr cloud, float leaf_size){
    VoxelGrid<PointXYZ> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::passthrough(PointCloud<PointC>::Ptr cloud, string field, float min, float max){
    PassThrough<PointC> ps;
    ps.setInputCloud(cloud);
    ps.setFilterFieldName(field);
    ps.setFilterLimits(min, max);

    ps.filter(*cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Cloud_Work::qt2T(Eigen::Quaternion<float> rot, Eigen::Vector3f offset){
    Eigen::Matrix4f T;
    Eigen::Matrix3f R = rot.matrix();
    Eigen::MatrixXf t(3,1);
    t = offset;
    T << R,t,
         0,0,0,1;

    return T;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::removeColorFromPoints(PointCloud<PointC>::Ptr in, PointCloud<PointXYZ>::Ptr out){
    PointXYZ point;
    for (int i=0; i<in->size(); i++) {
        point.x = in->points[i].x;
        point.y = in->points[i].y;
        point.z = in->points[i].z;
        out->push_back(point);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Cloud_Work::icp(const PointCloud<PointC>::Ptr src,
                                const PointCloud<PointC>::Ptr tgt,
                                Eigen::Matrix4f T){
    ROS_INFO("Entrando no ICP");
    // Reduzindo complexidade das nuvens
    PointCloud<PointC>::Ptr temp_src (new PointCloud<PointC>());
    PointCloud<PointC>::Ptr temp_tgt (new PointCloud<PointC>());

    *temp_src = *src; *temp_tgt = *tgt;

    float leaf_size = 0.01;
    filter_grid(temp_src, leaf_size);
    filter_grid(temp_tgt, leaf_size);

    Eigen::Matrix4f T_icp = T;

    /// ICP COMUM ///
    // Criando o otimizador de ICP comum
    pcl::IterativeClosestPoint<PointC, PointC> icp;
    icp.setUseReciprocalCorrespondences(true);
    icp.setInputTarget(temp_tgt);
    icp.setInputSource(temp_src);
    icp.setMaximumIterations(500); // Chute inicial bom 10-100
    icp.setTransformationEpsilon(1*1e-10);
    icp.setEuclideanFitnessEpsilon(1*1e-12);
    icp.setMaxCorrespondenceDistance(0.3);

    PointCloud<PointC> final2;
    icp.align(final2, T);

    if(icp.hasConverged())
        T_icp = icp.getFinalTransformation();

    temp_src->clear(); temp_tgt->clear();

    return T_icp;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::registra_global_icp(PointCloud<PointC>::Ptr parcial, Eigen::Quaternion<float> rot, Eigen::Vector3f offset){
    // Se primeira vez, so acumula, senao registra com ICP com a
    // transformada relativa entregue pela ZED como aproximacao inicial
    if(!primeira_vez){

        // Transformacao atual em forma matricial e calculos relativos da zed
        T_zed_atual = qt2T(rot, offset);
        T_zed_rel   = T_zed_anterior.inverse()*T_zed_atual;
        T_chute_icp = T_corrigida*T_zed_rel;
        // Acumular com otimizacao do ICP
        if(mutex_acumulacao == 0){
            mutex_acumulacao = 1;
            /// Alinhar nuvem de forma fina por ICP - devolve a transformacao correta para ser usada ///
            /// PASSAR PRIMEIRO A SOURCE, DEPOIS TARGET, DEPOIS A ODOMETRIA INICIAL A OTIMIZAR       ///
            // transformPointCloud(*parcial, *parcial, T_atual);
            T_corrigida = this->icp(parcial, acumulada_parcial_anterior, T_chute_icp);
            transformPointCloud(*parcial, *parcial, T_corrigida);            
            *acumulada_global += *parcial;
            // Salvar nuvem atual alinhada para proxima iteracao ser referencia
            *acumulada_parcial_anterior = *parcial;
            // Liberar mutex e ver o resultado da acumulacao no rviz
            mutex_acumulacao = 0;
        }
        T_zed_anterior = T_zed_atual; // Para a proxima iteracao

    } else {

        // Transformar SEMPRE para a referencia dos espaços -> primeira odometria capturada
        transformPointCloud(*parcial, *parcial, offset, rot);
        // Primeira nuvem
        *acumulada_global          += *parcial;
        *acumulada_parcial_anterior = *parcial;
        // Primeira transformacao vinda da odometria -> converter matriz 4x4
        T_icp = qt2T(rot, offset);
        // Ajustando transformadas para calculo de posicao da nuvem otimizado
        T_zed_atual = qt2T(rot, offset);
        T_zed_anterior = T_zed_atual;
        T_zed_rel = T_zed_anterior.inverse()*T_zed_atual;
        T_corrigida = T_zed_atual; T_chute_icp = T_zed_atual;
        // Podemos dizer que nao e mais primeira vez, chaveia a variavel
        set_primeira_vez(false);

    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::callback_acumulacao(const sensor_msgs::ImageConstPtr &msg_ast_image,
                                     const sensor_msgs::ImageConstPtr &msg_zed_image,
                                     const sensor_msgs::PointCloud2ConstPtr &msg_cloud,
                                     const sensor_msgs::PointCloud2ConstPtr &msg_pixels,
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
        int cont_nuvens = 0;
        while( cont_nuvens < n_nuvens_instantaneas ){
            // Converte nuvem -> ja devia estar filtrada do outro no
            PointCloud<PointC>::Ptr nuvem_inst (new PointCloud<PointC>());
            fromROSMsg(*msg_cloud, *nuvem_inst);
            // Garante que nao ha nenhum nan
            vector<int> indicesnan;
            removeNaNFromPointCloud(*nuvem_inst, *nuvem_inst, indicesnan);
            // Filtro de profundidade como vinda da GUI
            passthrough(nuvem_inst, "z", 0, profundidade_max);
            // Acumula na parcial do frame da camera, sem transformar, para projetar depois e salvar NVM
            acumulada_parcial_frame_camera->header.frame_id = msg_cloud->header.frame_id;
            *acumulada_parcial_frame_camera += *nuvem_inst;
            // Rotacao para corrigir odometria
            transformPointCloud<PointC>(*nuvem_inst, *nuvem_inst, offset_astra_zed, rot_astra_zed);
            // Acumulacao durante a aquisicao instantanea, sem transladar com a odometria
            acumulada_parcial->header.frame_id = "odom";
            *acumulada_parcial += *nuvem_inst;
            // Atualiza contagem de nuvens durante captura
            cont_nuvens++;
        } // fim do while -> contagem de tempo

        ROS_INFO("Nuvem parcial capturada");

        // Nao podemos mais acumular enquanto o botao nao chamar de novo
        set_inicio_acumulacao(false);

        // Acumular com a global, a depender se primeira vez ou nao
        registra_global_icp(acumulada_parcial, q, offset);

        // Calculo da pose da camera com transformaçoes homogeneas antes de obter quaternions e translacao
        // REFERENCIA : left_zed->ASTRA->ZED->ODOM
        Eigen::Matrix4f Tazo = T_astra_zed*T_corrigida.inverse();
        Eigen::Matrix3f rot_azo;
        rot_azo << Tazo(0, 0), Tazo(0, 1), Tazo(0, 2),
                   Tazo(1, 0), Tazo(1, 1), Tazo(1, 2),
                   Tazo(2, 0), Tazo(2, 1), Tazo(2, 2);
        Eigen::Quaternion<float>q_azo(rot_azo);
        Eigen::Vector3f t_azo;
        t_azo << Tazo(0, 3), Tazo(1, 3), Tazo(2, 3);

        // Salva os dados na pasta do projeto -> PARCIAIS
        if(acumulada_parcial->size() > 0){
            this->salva_dados_parciais(acumulada_parcial, msg_zed_image, msg_ast_image, msg_pixels);
            ROS_INFO("Dados Parciais salvos!");
        }

        // Salva no vetor de nuvens e poses para acumular tudo ao final
        nuvem_pose n;
        n.nuvem = *acumulada_parcial;
        n.centro_camera = calcula_centro_camera(q_azo, t_azo);
        np.push_back(n);

    } // fim do if -> acumular ou nao

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::salva_dados_parciais(PointCloud<PointC>::Ptr cloud,
                                      const sensor_msgs::ImageConstPtr &imagem_zed,
                                      const sensor_msgs::ImageConstPtr &imagem_ast,
                                      const sensor_msgs::PointCloud2ConstPtr &pixels_msg){
    // Atualiza contador de imagens - tambem usado para as nuvens parciais
    contador_imagens++;

    ROS_INFO("Salvando arquivo parcial %d.", contador_imagens);

    // Nome da pasta para salvar
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/teste/";
    std::string arquivo_imzed  = pasta + std::to_string(contador_imagens) + "z.jpg";
    std::string arquivo_imast  = pasta + std::to_string(contador_imagens) + "a.jpg";
    std::string arquivo_nuvem  = pasta + std::to_string(contador_imagens) + ".ply";
    std::string arquivo_nvm_z  = pasta + std::to_string(contador_imagens) + "z.nvm";
    std::string arquivo_nvm_a  = pasta + std::to_string(contador_imagens) + "a.nvm";
    std::string arquivo_pixels = pasta + std::to_string(contador_imagens) + "_pixels.ply";

    // Converte e salva imagem da zed
    cv_bridge::CvImagePtr imgptr, imgzptr;
    imgzptr = cv_bridge::toCvCopy(imagem_zed, sensor_msgs::image_encodings::BGR8);
    imwrite(arquivo_imzed, imgzptr->image);
    imgptr  = cv_bridge::toCvCopy(imagem_ast, sensor_msgs::image_encodings::BGR8);
    imwrite(arquivo_imast, imgptr->image);

    ///// Escrevendo para a zed aqui /////
    Eigen::Matrix4f Tazo = T_astra_zed*T_corrigida.inverse();
    Eigen::Matrix3f rot_azo;
    rot_azo << Tazo(0, 0), Tazo(0, 1), Tazo(0, 2),
               Tazo(1, 0), Tazo(1, 1), Tazo(1, 2),
               Tazo(2, 0), Tazo(2, 1), Tazo(2, 2);
    Eigen::Quaternion<float>q_azo(rot_azo);
    Eigen::Vector3f t_azo;
    t_azo << Tazo(0, 3), Tazo(1, 3), Tazo(2, 3);

    // Centro da camera, para escrever no arquivo NVM
    Eigen::MatrixXf C = calcula_centro_camera(q_azo, t_azo);

    // Escreve o arquivo NVM parcial, super necessario
    ofstream filez(arquivo_nvm_z);
    if(filez.is_open()){

        filez << "NVM_V3\n\n";
        filez << "1\n"; // Quantas imagens, sempre uma aqui
        std::string linha_imagem = escreve_linha_imagem(1462, arquivo_imzed, C, q_azo); // Imagem com detalhes de camera
        filez << linha_imagem; // Imagem com detalhes de camera
        // Anota no vetor de imagens que irao para o arquivo NVM da nuvem acumulada
        acumulada_imagens.push_back(linha_imagem);

    } // fim do if is open
    filez.close(); // Fechar para nao ter erro

    ///// Escrevendo para a astra aqui /////
    Eigen::Matrix4f so_rot_astra;
    so_rot_astra << T_astra_zed;
    so_rot_astra(0, 3) = 0; so_rot_astra(1, 3) = 0; so_rot_astra(2, 3) = 0; // Muda pois nao tem o offset
    Tazo = so_rot_astra*T_corrigida.inverse();
    rot_azo << Tazo(0, 0), Tazo(0, 1), Tazo(0, 2),
               Tazo(1, 0), Tazo(1, 1), Tazo(1, 2),
               Tazo(2, 0), Tazo(2, 1), Tazo(2, 2);
    Eigen::Quaternion<float>q_astra(rot_azo);
    Eigen::Vector3f t_astra(Tazo(0, 3), Tazo(1, 3), Tazo(2, 3));

    // Centro da camera, para escrever no arquivo NVM
    C = calcula_centro_camera(q_astra, t_astra);

    // Escreve o arquivo NVM parcial, super necessario
    ofstream filea(arquivo_nvm_a);
    if(filea.is_open()){

        filea << "NVM_V3\n\n";
        filea << "1\n"; // Quantas imagens, sempre uma aqui
        std::string linha_imagema = escreve_linha_imagem(525, arquivo_imast, C, q_astra); // Imagem com detalhes de camera
        filea << linha_imagema; // Imagem com detalhes de camera
        // Anota no vetor de imagens que irao para o arquivo NVM da nuvem acumulada
        acumulada_imagens.push_back(linha_imagema);

    } // fim do if is open
    filea.close(); // Fechar para nao ter erro

    // Calcular normais para a nuvem
    pcl::PointCloud<PointCN>::Ptr final_parcial (new pcl::PointCloud<PointCN>);
    calcula_normais_com_pose_camera(final_parcial, *cloud, C, 30);

    // Salvar nuvem em arquivo .ply
    PointCloud<PointXYZ>::Ptr pixels (new PointCloud<PointXYZ>());
    fromROSMsg(*pixels_msg, *pixels);
    if(pcl::io::savePLYFileASCII(arquivo_pixels, *pixels))
        ROS_INFO("Pixels para nuvem parcial %d salvos!", contador_imagens);

    // Salvar nuvem em arquivo .ply
    if(pcl::io::savePLYFileASCII(arquivo_nuvem, *final_parcial))
        ROS_INFO("Nuvem parcial %d salva!", contador_imagens);

} // Fim da funcao de salvar arquivos parciais
///////////////////////////////////////////////////////////////////////////////////////////
std::string Cloud_Work::escreve_linha_imagem(float foco, std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q){
    std::string linha = nome;
    // Adicionar foco
    linha = linha + " " + std::to_string(foco);
    // Adicionar quaternion
    linha = linha + " " + std::to_string(q.w()) + " " + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z());
    // Adicionar centro da camera
    linha = linha + " " + std::to_string(C(0, 0)) + " " + std::to_string(C(1, 0)) + " " + std::to_string(C(2, 0));
    // Adicionar distorcao radial (crendo 0) e 0 final
    linha = linha + " 0 0\n"; // IMPORTANTE pular linha aqui, o MeshRecon precisa disso no MART
    // Muda as virgulas por pontos no arquivo
    std::replace(linha.begin(), linha.end(), ',', '.');
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
void Cloud_Work::salvar_acumulada(){
    // Nome da pasta para salvar
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/teste/";
    std::string arquivo_nuvem = pasta+"nuvem_final.ply";
    std::string arquivo_nvm   = pasta+"cameras_final.nvm";

    ROS_INFO("Salvando nuvem acumulada.....");
    // Acumulada global com normais
    PointCloud<PointCN>::Ptr acumulada_global_normais      (new PointCloud<PointCN>());
    PointCloud<PointCN>::Ptr acumulada_global_normais_temp (new PointCloud<PointCN>());
    PointCloud<PointC>::Ptr cameras_temp (new PointCloud<PointC>());
    for(int i=0; i < np.size(); i++){
        ROS_INFO("Calculando normais na nuvem %d, de %d totais, aguarde...", i+1, np.size());
        // Acumula para cada nuvem calculando as normais no sentido correto
        calcula_normais_com_pose_camera(acumulada_global_normais_temp, np.at(i).nuvem, np.at(i).centro_camera, 30);
        *acumulada_global_normais += *acumulada_global_normais_temp;
        acumulada_global_normais_temp->clear();
        PointC ponto_temp;
        ponto_temp.x = np.at(i).centro_camera(0);
        ponto_temp.y = np.at(i).centro_camera(1);
        ponto_temp.z = np.at(i).centro_camera(2);
        ponto_temp.r = 250;
        ponto_temp.g = 0;
        ponto_temp.b = 0;
        cameras_temp->push_back(ponto_temp);
    }
    savePLYFileASCII(pasta+"camerascentro.ply", *cameras_temp);

    pcl::io::savePLYFileASCII(arquivo_nuvem, *acumulada_global_normais);

    // Salvar o arquivo NVM para a acumulada
    this->salva_nvm_acumulada(arquivo_nvm);
    ROS_INFO("Nuvem salva na pasta correta!!");

    np.clear();
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::calcula_normais_com_pose_camera(PointCloud<PointCN>::Ptr acc_temp, PointCloud<PointC> cloud, Eigen::MatrixXf C, int K){
    // Calcula centro da camera aqui
    Eigen::Vector3f p = C;
    // Estima normais viradas para o centro da camera
    NormalEstimationOMP<PointC, Normal> ne;
    PointCloud<PointC>::Ptr cloud2 (new PointCloud<PointC>());
    *cloud2 = cloud;
    ne.setInputCloud(cloud2);
    search::KdTree<PointC>::Ptr tree (new search::KdTree<PointC>());
    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(K);
    ne.setNumberOfThreads(4);

    ne.compute(*cloud_normals);

    concatenateFields(cloud, *cloud_normals, *acc_temp);

    vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*acc_temp, *acc_temp, indicesnan);

    // Forcar virar as normais na marra
    for(int i=0; i < acc_temp->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << acc_temp->points[i].normal_x, acc_temp->points[i].normal_y, acc_temp->points[i].normal_z;
        cp << p(0)-acc_temp->points[i].x, p(1)-acc_temp->points[i].y, p(2)-acc_temp->points[i].z;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            acc_temp->points[i].normal_x = -acc_temp->points[i].normal_x;
            acc_temp->points[i].normal_y = -acc_temp->points[i].normal_y;
            acc_temp->points[i].normal_z = -acc_temp->points[i].normal_z;
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::salva_nvm_acumulada(std::string nome){

    ROS_INFO("Salvando arquivo NVM acumulada");
    // Abrindo o arquivo com o nome correto
    ofstream file(nome);
    if(file.is_open()){
        file << "NVM_V3\n\n";
        file << std::to_string(acumulada_imagens.size())+"\n"; // Quantas imagens
        for(int i=0; i < acumulada_imagens.size(); i++){ // Escreve todos os nomes acumulados de imagens
            file << acumulada_imagens[i];
        }
    }
    file.close();
    ROS_INFO("Arquivo NVM da nuvem global salvo!");

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::reiniciar(){
    // Reinicia tudo que pode ser reiniciado aqui
    acumulada_global->clear(); acumulada_parcial->clear();
    acumulada_parcial_anterior->clear(); acumulada_parcial_frame_camera->clear();
    np.clear();

    system("gnome-terminal -x sh -c 'rosservice call /zed/reset_odometry'");

    set_primeira_vez(true); // Vamos ter a primeira vez de aquisicao novamente

    contador_imagens = 0;

    cout << "\n\n\n\nRESETOU TUDO CAMERAS E ZED \n\n\n\n" << endl;
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
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::set_profundidade_max(float d){
    profundidade_max = d;
}
///////////////////////////////////////////////////////////////////////////////////////////
/// Mesh
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::triangulate(){
    if(acumulada_global->size() > 0){
        int metodo = 2;

        PointCloud<PointCN>::Ptr cloud_normals (new PointCloud<PointCN>());
        calculateNormalsAndConcatenate(acumulada_global, cloud_normals, 30);

        if(metodo == 1){

            pcl::search::KdTree<PointCN>::Ptr tree2 (new pcl::search::KdTree<PointCN>);

            GreedyProjectionTriangulation<PointCN> gp3;
            gp3.setSearchRadius (0.05);
            gp3.setMu (3);
            gp3.setMaximumNearestNeighbors (30);
            gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
            gp3.setMinimumAngle(M_PI/18); // 10 degrees
            gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
            gp3.setNormalConsistency(false);
            ROS_INFO("Construindo Mesh sobre a nuvem....");
            gp3.setInputCloud (cloud_normals);
            gp3.setSearchMethod (tree2);
            gp3.reconstruct (mesh_acumulada);
            ROS_INFO("Mesh reconstruida!");

        } else if(metodo == 2){

            Poisson<PointCN> poisson;
            int depth = 12;
            ROS_INFO("Comecando processo de POISSON com depth = %d", depth);
            poisson.setDepth(12);
            poisson.setInputCloud(cloud_normals);
            poisson.reconstruct(mesh_acumulada);
            ROS_INFO("Processo de POISSON finalizado.");

        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::calculateNormalsAndConcatenate(PointCloud<PointC>::Ptr cloud, PointCloud<PointCN>::Ptr cloud2, int K){
    ROS_INFO("Calculando normais da nuvem, aguarde...");
    NormalEstimationOMP<PointC, Normal> ne;
    ne.setInputCloud(cloud);
    search::KdTree<PointC>::Ptr tree (new search::KdTree<PointC>());
    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(K);
    ne.setNumberOfThreads(4);

    ne.compute(*cloud_normals);

    concatenateFields(*cloud, *cloud_normals, *cloud2);

    vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*cloud2, *cloud2, indicesnan);
    ROS_INFO("Normais calculadas!");
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::calculateNormalsAndConcatenate(PointCloud<PointXYZ>::Ptr cloud, PointCloud<PointNormal>::Ptr cloud2, int K){
    ROS_INFO("Calculando normais da nuvem, aguarde...");
    NormalEstimationOMP<PointXYZ, Normal> ne;
    ne.setInputCloud(cloud);
    search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ>());
    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(K);
    ne.setNumberOfThreads(4);

    ne.compute(*cloud_normals);

    concatenateFields(*cloud, *cloud_normals, *cloud2);

    vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*cloud2, *cloud2, indicesnan);
    ROS_INFO("Normais calculadas!");
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::saveMesh(std::string nome){
    ROS_INFO("Salvando a Mesh no nome correto...");
    if(savePolygonFilePLY(nome, mesh_acumulada))
        ROS_INFO("Mesh salva!");
}

} // Fim do namespace handset_gui
