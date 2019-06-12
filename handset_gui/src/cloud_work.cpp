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

    // Subscribers para sincronizar
    message_filters::Subscriber<sensor_msgs::Image      > sub_imagem(nh_, "/astra_rgb"      , 10);
    message_filters::Subscriber<sensor_msgs::Image      > sub_im_zed(nh_, "/zed2"           , 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_nuvem (nh_, "/astra_projetada", 10);
    message_filters::Subscriber<Odometry                > sub_odom  (nh_, "/odom2"          , 10);
    sync.reset(new Sync(syncPolicy(10), sub_imagem, sub_im_zed, sub_nuvem, sub_odom));
    sync->registerCallback(boost::bind(&Cloud_Work::callback_acumulacao, this, _1, _2, _3, _4));

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
//    offset_astra_zed << -0.001, -0.090, -0.025;
//    offset_astra_zed << 0, 0.048, 0.03; // No frame da ZED, apos rotaçao de ASTRA->ZED
//    offset_astra_zed << 0.048, 0.03, 0; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA_RGB
//    offset_astra_zed << 0.06, 0, 0; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA_RGB
//    offset_astra_zed << 0.022, 0.03, -0.009; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA_DEPTH
//    offset_astra_zed << 0.048, 0.031, 0; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA, CORRIGIDO MART 1
    offset_astra_zed << 0.048, 0.031, -0.019; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA, CORRIGIDO MART 2
//    offset_astra_zed << -0.004, 0.031, -0.019; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA, CORRIGIDO MART 3
//    offset_astra_zed << -0.004, 0.031, 0; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA, CORRIGIDO MART 4
    // Matriz de transformaçao que leva ASTRA->ZED, assim pode calcular posicao da CAMERA ao multiplicar por ZED->ODOM
    T_astra_zed << matrix, offset_astra_zed,
                   0, 0, 0, 1;

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
void Cloud_Work::filter_grid(PointCloud<PointXYZ>::Ptr cloud, float leaf_size){
    VoxelGrid<PointXYZ> grid;
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
    Eigen::Matrix3f R = rot.matrix();
    Eigen::MatrixXf t(3,1);
    t = offset;
    T << R,t,
         0,0,0,1;

    return T;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::removeColorFromPoints(PointCloud<PointT>::Ptr in, PointCloud<PointXYZ>::Ptr out){
    PointXYZ point;
    for (int i=0; i<in->size(); i++) {
        point.x = in->points[i].x;
        point.y = in->points[i].y;
        point.z = in->points[i].z;
        out->push_back(point);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f Cloud_Work::icp(const PointCloud<PointT>::Ptr src,
                                const PointCloud<PointT>::Ptr tgt,
                                Eigen::Matrix4f T){
    ROS_INFO("Entrando no ICP");
    // Reduzindo complexidade das nuvens
    PointCloud<PointT>::Ptr temp_src (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr temp_tgt (new PointCloud<PointT>());

    *temp_src = *src; *temp_tgt = *tgt;

    float leaf_size = 0.01;
    filter_grid(temp_src, leaf_size);
    filter_grid(temp_tgt, leaf_size);

//    pcl::io::savePLYFileASCII("/home/grin/Desktop/nuvem_filtrada.ply", *temp_tgt);

    Eigen::Matrix4f T_icp = T;

    /// ICP COMUM ///
    // Criando o otimizador de ICP comum
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setUseReciprocalCorrespondences(true);
    icp.setInputTarget(temp_tgt);
    icp.setInputSource(temp_src);
    icp.setMaximumIterations(500); // Chute inicial bom 10-100
    icp.setTransformationEpsilon(1*1e-10);
    icp.setEuclideanFitnessEpsilon(1*1e-12);
    icp.setMaxCorrespondenceDistance(0.1);

    PointCloud<PointT> final2;
    icp.align(final2, T);

    if(icp.hasConverged())
        T_icp = icp.getFinalTransformation();

    temp_src->clear(); temp_tgt->clear();

    return T_icp;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::registra_global_icp(PointCloud<PointT>::Ptr parcial, Eigen::Quaternion<float> rot, Eigen::Vector3f offset){
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
void Cloud_Work::callback_acumulacao(const sensor_msgs::ImageConstPtr &msg_image,
                                     const sensor_msgs::ImageConstPtr &msg_zed_image,
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
        int cont_nuvens = 0;
        while( cont_nuvens < n_nuvens_instantaneas ){
            // Converte nuvem -> ja devia estar filtrada do outro no
            PointCloud<PointT>::Ptr nuvem_inst (new PointCloud<PointT>());
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
            this->salva_dados_parciais(acumulada_parcial, q_azo, t_azo, msg_image, msg_zed_image);
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
void Cloud_Work::salva_dados_parciais(PointCloud<PointT>::Ptr cloud,
                                      Eigen::Quaternion<float> rot,
                                      Eigen::Vector3f offset,
                                      const sensor_msgs::ImageConstPtr &imagem,
                                      const sensor_msgs::ImageConstPtr &imagem_zed){
    // Atualiza contador de imagens - tambem usado para as nuvens parciais
    contador_imagens++;

    ROS_INFO("Salvando arquivo parcial %d.", contador_imagens);

    // Nome da pasta para salvar
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/teste/";
    std::string arquivo_imagem = pasta + std::to_string(contador_imagens) + ".jpg";
    std::string arquivo_imzed  = pasta + std::to_string(contador_imagens) + "z.jpg";
    std::string arquivo_nuvem  = pasta + std::to_string(contador_imagens) + ".ply";
    std::string arquivo_nvm    = pasta + std::to_string(contador_imagens) + ".nvm";

    // Converte e salva imagem
    cv_bridge::CvImagePtr imgptr, imgzptr;
    imgptr  = cv_bridge::toCvCopy(imagem    , sensor_msgs::image_encodings::BGR8);
    imgzptr = cv_bridge::toCvCopy(imagem_zed, sensor_msgs::image_encodings::BGR8);
    imwrite(arquivo_imagem, imgptr->image );
    imwrite(arquivo_imzed , imgzptr->image);

    // Centro da camera, para escrever no arquivo NVM
    Eigen::MatrixXf C = calcula_centro_camera(rot, offset);

    // Escreve o arquivo NVM parcial, super necessario
    ofstream file(arquivo_nvm);
    if(file.is_open()){

        file << "NVM_V3\n\n";
        file << "1\n"; // Quantas imagens, sempre uma aqui
//        std::string linha_imagem = escreve_linha_imagem(arquivo_imagem, C, rot); // Imagem com detalhes de camera
        std::string linha_imagem = escreve_linha_imagem(arquivo_imzed, C, rot); // Imagem com detalhes de camera
        file << linha_imagem; // Imagem com detalhes de camera
        // Anota no vetor de imagens que irao para o arquivo NVM da nuvem acumulada
        acumulada_imagens.push_back(linha_imagem);

    } // fim do if is open
    file.close(); // Fechar para nao ter erro

    // Calcular normais para a nuvem
    pcl::PointCloud<PointTN>::Ptr final_parcial (new pcl::PointCloud<PointTN>);
    this->calculateNormalsAndConcatenate(cloud, final_parcial, 30);

    // Salvar nuvem em arquivo .ply
    if(pcl::io::savePLYFileASCII(arquivo_nuvem, *final_parcial))
        ROS_INFO("Nuvem parcial %d salva!", contador_imagens);

} // Fim da funcao de salvar arquivos parciais
///////////////////////////////////////////////////////////////////////////////////////////
std::string Cloud_Work::escreve_linha_imagem(std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q){
    std::string linha = nome;
    float fzed = 1462.324341;
    // Adicionar foco
//    linha = linha + " " + to_string(astra_model.fx());
    linha = linha + " " + std::to_string(fzed);
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

//    // A principio so salvar a nuvem
//    ROS_INFO("Salvando nuvem acumulada.....");
//    PointCloud<PointTN>::Ptr acumulada_global_normais (new PointCloud<PointTN>());
//    this->calculateNormalsAndConcatenate(acumulada_global, acumulada_global_normais, 30);
//    pcl::io::savePLYFileASCII(arquivo_nuvem, *acumulada_global_normais);
//    ROS_INFO("Nuvem salva na pasta correta!!");
    ROS_INFO("Salvando nuvem acumulada.....");
    // Acumulada global com normais
    PointCloud<PointTN>::Ptr acumulada_global_normais      (new PointCloud<PointTN>());
    PointCloud<PointTN>::Ptr acumulada_global_normais_temp (new PointCloud<PointTN>());
    for(int i=0; i < np.size(); i++){
        ROS_INFO("Calculando normais na nuvem %d, de %d totais, aguarde...", i+1, np.size());
        // Acumula para cada nuvem calculando as normais no sentido correto
        calcula_normais_com_pose_camera(acumulada_global_normais_temp, np.at(i).nuvem, np.at(i).centro_camera, 30);
        *acumulada_global_normais += *acumulada_global_normais_temp;
        acumulada_global_normais_temp->clear();
    }
    pcl::io::savePLYFileASCII(arquivo_nuvem, *acumulada_global_normais);
    ROS_INFO("Nuvem salva na pasta correta!!");

    // Salvar o arquivo NVM para a acumulada
    this->salva_nvm_acumulada(arquivo_nvm);

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::calcula_normais_com_pose_camera(PointCloud<PointTN>::Ptr acc_temp, PointCloud<PointT> cloud, Eigen::MatrixXf C, int K){
    // Calcula centro da camera aqui
    Eigen::Vector3f p = C;
    // Estima normais viradas para o centro da camera
    NormalEstimationOMP<PointT, Normal> ne;
    PointCloud<PointT>::Ptr cloud2 (new PointCloud<PointT>());
    *cloud2 = cloud;
    ne.setInputCloud(cloud2);
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.setKSearch(K);
    ne.setNumberOfThreads(4);
    ne.setViewPoint(p(0), p(1), p(2));

    ne.compute(*cloud_normals);

    concatenateFields(cloud, *cloud_normals, *acc_temp);

    vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*acc_temp, *acc_temp, indicesnan);

    // Forcar virar as normais na marra
    for(int i=0; i < cloud_normals->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << acc_temp->points[i].normal_x, acc_temp->points[i].normal_y, acc_temp->points[i].normal_z;
        cp << p(0)-acc_temp->points[i].x, p(1)-acc_temp->points[i].y, p(2)-acc_temp->points[i].z;
        float cos_theta = normal.dot(cp)/(normal.norm()*cp.norm());
        if(cos_theta < 0){ // Esta apontando errado, deve inverter
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

        PointCloud<PointTN>::Ptr cloud_normals (new PointCloud<PointTN>());
        calculateNormalsAndConcatenate(acumulada_global, cloud_normals, 30);

        if(metodo == 1){

            pcl::search::KdTree<PointTN>::Ptr tree2 (new pcl::search::KdTree<PointTN>);

            GreedyProjectionTriangulation<PointTN> gp3;
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

            Poisson<PointTN> poisson;
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
void Cloud_Work::calculateNormalsAndConcatenate(PointCloud<PointT>::Ptr cloud, PointCloud<PointTN>::Ptr cloud2, int K){
    ROS_INFO("Calculando normais da nuvem, aguarde...");
    NormalEstimationOMP<PointT, Normal> ne;
    ne.setInputCloud(cloud);
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>());
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
