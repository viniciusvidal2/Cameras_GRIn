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

template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    message_filters::SimpleFilter<M>::signalMessage(msg);
  }
};

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
    acumulada_global_anterior      = (PointCloud<PointC>::Ptr) new PointCloud<PointC>;

    // Inicia publicadores de nuvens
    pub_parcial = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_parcial", 10);
    pub_global  = nh_.advertise<sensor_msgs::PointCloud2>("/acumulada_global" , 10);

    pub_nuvemastra_offline = nh_.advertise<sensor_msgs::PointCloud2>("/astra_fonte_offline", 10);
    pub_zed_offline = nh_.advertise<sensor_msgs::Image>("/zed_offline", 10);

    // Subscribers para sincronizar
    message_filters::Subscriber<sensor_msgs::Image      > sub_im_as (nh_, "/astra2"         , 10);
    message_filters::Subscriber<sensor_msgs::Image      > sub_im_zed(nh_, "/zed2"           , 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_nuvem (nh_, "/astra_projetada", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_pixel (nh_, "/pixels"         , 10);
    message_filters::Subscriber<Odometry                > sub_odom  (nh_, "/odom2"          , 10);
    sync.reset(new Sync(syncPolicy(10), sub_im_as, sub_im_zed, sub_nuvem, sub_pixel, sub_odom));
    funcionou_porra = sync->registerCallback(boost::bind(&Cloud_Work::callback_acumulacao, this, _1, _2, _3, _4, _5));

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

    // Inicio do mutex de acumulacao - verdadeiro se estamos acumulando
    mutex_acumulacao = false;

    // Controle de online ou offline iniciando como false, que e o normal, e o GUI mandara do contrario qualquer coisa
    vamos_de_bag = false;
    comando_bag = 0;

    // Definicao de rotacao fixa entre frame da ASTRA e da ZED -> de ASTRA->ZED A PRINCIPIO
    Eigen::Matrix3f matrix;
    matrix = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::Quaternion<float> rot_temp(matrix);
    rot_astra_zed = rot_temp.inverse(); // Aqui esta de ZED->ASTRA (nuvens)

    offset_astra_zed << 0.052, 0.01, -0.01; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, por reconfigure com imagem raw objetos proximos

    // Matriz de transformaçao que leva ASTRA->ZED, assim pode calcular posicao da CAMERA ao multiplicar por ZED->ODOM
    T_astra_zed << matrix, offset_astra_zed,
                   0,   0,   0,   1;

    // Foco da ZED e ASTRA estimado inicial
    fzed = 1462.0;
    fastra = 525.0;

    // Rodar o no
    ros::Rate rate(2);
    while(ros::ok()){

        // Rotina para rodar dados com bag, se assim for desejado, vamos forçar loop eterno la dentro
        if(vamos_de_bag)
            ProcessaOffline();

        // Publica recorrentemente a nuvem atual
        this->publica_nuvens();

        ros::spinOnce();
        rate.sleep();

    }

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::cancela_listeners(bool eai){
    if(eai){
        funcionou_porra.disconnect();
        vamos_de_bag = true;
    } else {
        vamos_de_bag = false;
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
void Cloud_Work::publica_nuvens_offline(sensor_msgs::ImageConstPtr im, sensor_msgs::PointCloud2ConstPtr pt){
    // Convertendo e Publicando
    pub_zed_offline.publish(*im);
    pub_nuvemastra_offline.publish(*pt);
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
void Cloud_Work::filterForNan(PointCloud<PointC>::Ptr cloud, PointCloud<PointXYZ>::Ptr pixels){
    vector<int> indicesnan;
    removeNaNFromPointCloud(*cloud, *cloud, indicesnan);
    boost::shared_ptr<vector<int> > indicesnanptr (new vector<int> (indicesnan));
    ExtractIndices<PointXYZ> ext;
    ext.setInputCloud(pixels);
    ext.setIndices(indicesnanptr);
    ext.setNegative(false); // A funcao ja retorna os indices que nao tinham sido filtrados antes
    ext.filter(*pixels);
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::passthrough(PointCloud<PointC>::Ptr cloud, PointCloud<PointXYZ>::Ptr foto, string field, float min, float max){
    PassThrough<PointC> ps(true);
    ps.setInputCloud(cloud);
    ps.setFilterFieldName(field);
    ps.setFilterLimits(min, max);

    ps.filter(*cloud);
    IndicesConstPtr indices_removidos = ps.getRemovedIndices();

    ExtractIndices<PointXYZ> extract;
    extract.setInputCloud(foto);
    extract.setIndices(indices_removidos);
    extract.setNegative(true);
    extract.filter(*foto);
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
    for (unsigned long i=0; i<in->size(); i++) {
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
    icp.setMaximumIterations(300); // Chute inicial bom 10-100
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
            T_corrigida = this->icp(parcial, acumulada_global, T_chute_icp);
            transformPointCloud(*parcial, *parcial, T_corrigida);
            // Guarda a acumulada anterior aqui
            *acumulada_global_anterior = *acumulada_global;
            // Adiciona a parcial na global
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
        *acumulada_global_anterior  = *parcial;
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
        acumulada_parcial->clear(); //acumulada_parcial_frame_camera->clear(); temp_nvm->clear();

        // Converte nuvem -> ja devia estar filtrada do outro no
        PointCloud<PointC  >::Ptr nuvem_inst (new PointCloud<PointC  >());
        fromROSMsg(*msg_cloud, *nuvem_inst);
        PointCloud<PointXYZ>::Ptr nuvem_pix  (new PointCloud<PointXYZ>());
        fromROSMsg(*msg_pixels, *nuvem_pix);
        // Garante que nao ha nenhum nan, e se tiver remove tambem na nuvem de pixels
        filterForNan(nuvem_inst, nuvem_pix);
        // Guarda a nuvem total, para o bat aproveitar melhor a otimizacao
        PointCloud<PointC  >::Ptr nuvem_inst_total (new PointCloud<PointC  >());
        PointCloud<PointXYZ>::Ptr nuvem_pix_total  (new PointCloud<PointXYZ>());
        *nuvem_inst_total = *nuvem_inst;
        *nuvem_pix_total  = *nuvem_pix ;
        // Filtro de profundidade como vinda da GUI
        passthrough(nuvem_inst, nuvem_pix, "z", 0, profundidade_max);

        // Rotacao para corrigir odometria
        transformPointCloud<PointC>(*nuvem_inst, *nuvem_inst, offset_astra_zed, rot_astra_zed);
        transformPointCloud<PointC>(*nuvem_inst_total, *nuvem_inst_total, offset_astra_zed, rot_astra_zed);
        // Acumulacao durante a aquisicao instantanea, sem transladar com a odometria
        acumulada_parcial->header.frame_id = "odom";
        *acumulada_parcial += *nuvem_inst;

        ROS_INFO("Nuvem parcial capturada");

        // Nao podemos mais acumular enquanto o botao nao chamar de novo
        set_inicio_acumulacao(false);

        // Acumular com a global, a depender se primeira vez ou nao
        registra_global_icp(acumulada_parcial, q, offset);

        // Corrigir a nuvem completa tambem, para ser otimizada com o melhor chute inicial
        transformPointCloud<PointC>(*nuvem_inst_total, *nuvem_inst_total, T_corrigida);

        // Calculo da pose da camera com transformaçoes homogeneas antes de obter quaternions e translacao
        // REFERENCIA : left_zed->ASTRA->ZED->ODOM
        Eigen::Matrix4f Tazo = T_astra_zed*T_corrigida.inverse();
        Eigen::Matrix3f rot_azo;
        rot_azo = Tazo.block(0, 0, 3, 3);
        Eigen::Quaternion<float>q_azo(rot_azo);
        Eigen::Vector3f t_azo;
        t_azo << Tazo(0, 3), Tazo(1, 3), Tazo(2, 3);

        // Salva os dados na pasta do projeto -> PARCIAIS
        if(acumulada_parcial->size() > 0){
            this->salva_dados_parciais(acumulada_parcial, msg_zed_image, msg_ast_image, nuvem_pix_total, nuvem_inst_total);
            ROS_INFO("Dados Parciais %d salvos!", contador_imagens);
        }

        // Salva no vetor de nuvens e poses para acumular tudo ao final
        nuvem_pose n;
        n.nuvem = *acumulada_parcial;
        n.centro_camera = calcula_centro_camera(q_azo, t_azo);
        np.push_back(n);

    } // fim do if -> acumular ou nao

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::callback_offline(const sensor_msgs::ImageConstPtr &msg_ast_image,
                                  const sensor_msgs::ImageConstPtr &msg_zed_image,
                                  const sensor_msgs::PointCloud2ConstPtr &msg_cloud,
                                  const sensor_msgs::PointCloud2ConstPtr &msg_pixels,
                                  const OdometryConstPtr &msg_odom){
    ros::Rate r(2);
    conjunto_dados_atual++;
    this->publica_nuvens();
    this->publica_nuvens_offline(msg_zed_image, msg_cloud);
    // Aguarda comando aqui
    while(comando_bag == 0){
        this->publica_nuvens();
        this->publica_nuvens_offline(msg_zed_image, msg_cloud);
        ROS_INFO("Aguardando comando da GUI no conjunto %d ....", conjunto_dados_atual);
        r.sleep();
    }

    switch(comando_bag){

    case 1: // Nao vamos usar essas mensagens

        ROS_INFO("Nao usou o conjunto %d na acumulacao!", conjunto_dados_atual);
        this->publica_nuvens();
        this->publica_nuvens_offline(msg_zed_image, msg_cloud);
        break;

    case 3: // Corrigir para a acumulada anterior ja que essa nao foi boa

        // Nuvem visual
        *acumulada_global = *acumulada_global_anterior;
        // Lista de imagens pra salvar ao fim
        acumulada_imagens.erase(acumulada_imagens.end());
        // Lista de nuvens para acumular com normais ao final
        np.erase(np.end());
        // Publica pra ver se ta tudo certo
        this->publica_nuvens();
        ROS_INFO("Foi retirado o frame anterior %d...", conjunto_dados_atual-1);
        break;

    case 2: // Aqui deveria ter o mesmo processamento visto quando online

        // Converte odometria -> so uma vez no inicio para evitar drifts
        Eigen::Quaternion<float> q;
        Eigen::Vector3f offset;

        // Converte odometria -> so uma vez no inicio para evitar drifts
        q.x() = (float)msg_odom->pose.pose.orientation.x;
        q.y() = (float)msg_odom->pose.pose.orientation.y;
        q.z() = (float)msg_odom->pose.pose.orientation.z;
        q.w() = (float)msg_odom->pose.pose.orientation.w;
        offset << msg_odom->pose.pose.position.x, msg_odom->pose.pose.position.y, msg_odom->pose.pose.position.z;

        // Reiniciando nuvens parciais
        acumulada_parcial->clear();

        // Converte nuvem -> ja devia estar filtrada do outro no
        PointCloud<PointC  >::Ptr nuvem_inst (new PointCloud<PointC  >());
        fromROSMsg(*msg_cloud, *nuvem_inst);
        PointCloud<PointXYZ>::Ptr nuvem_pix  (new PointCloud<PointXYZ>());
        fromROSMsg(*msg_pixels, *nuvem_pix);
        // Garante que nao ha nenhum nan, e se tiver remove tambem na nuvem de pixels
        filterForNan(nuvem_inst, nuvem_pix);
        // Guarda a nuvem total, para o bat aproveitar melhor a otimizacao
        PointCloud<PointC  >::Ptr nuvem_inst_total (new PointCloud<PointC  >());
        PointCloud<PointXYZ>::Ptr nuvem_pix_total  (new PointCloud<PointXYZ>());
        *nuvem_inst_total = *nuvem_inst;
        *nuvem_pix_total  = *nuvem_pix ;

        // Filtro de profundidade como vinda da GUI
        passthrough(nuvem_inst, nuvem_pix, "z", 0, profundidade_max);
        // Rotacao para corrigir odometria
        transformPointCloud<PointC>(*nuvem_inst, *nuvem_inst, offset_astra_zed, rot_astra_zed);
        transformPointCloud<PointC>(*nuvem_inst_total, *nuvem_inst_total, offset_astra_zed, rot_astra_zed);
        // Acumulacao durante a aquisicao instantanea, sem transladar com a odometria
        acumulada_parcial->header.frame_id = "odom";
        *acumulada_parcial += *nuvem_inst;

        ROS_INFO("Nuvem parcial capturada");

        // Acumular com a global, a depender se primeira vez ou nao
        registra_global_icp(acumulada_parcial, q, offset);

        // Corrigir a nuvem completa tambem, para ser otimizada com o melhor chute inicial
        transformPointCloud<PointC>(*nuvem_inst_total, *nuvem_inst_total, T_corrigida);

        // Calculo da pose da camera com transformaçoes homogeneas antes de obter quaternions e translacao
        // REFERENCIA : left_zed->ASTRA->ZED->ODOM
        Eigen::Matrix4f Tazo = T_astra_zed*T_corrigida.inverse();
        Eigen::Matrix3f rot_azo;
        rot_azo = Tazo.block(0, 0, 3, 3);
        Eigen::Quaternion<float>q_azo(rot_azo);
        Eigen::Vector3f t_azo;
        t_azo << Tazo(0, 3), Tazo(1, 3), Tazo(2, 3);

        // Salva os dados na pasta do projeto -> PARCIAIS
        if(acumulada_parcial->size() > 0){
            this->salva_dados_parciais(acumulada_parcial, msg_zed_image, msg_ast_image, nuvem_pix_total, nuvem_inst_total);
            ROS_INFO("Dados Parciais %d salvos!", contador_imagens);
        }

        // Salva no vetor de nuvens e poses para acumular tudo ao final
        nuvem_pose n;
        n.nuvem = *acumulada_parcial;
        n.centro_camera = calcula_centro_camera(q_azo, t_azo);
        np.push_back(n);

        // Publica pra mostrar pra galera
        this->publica_nuvens();
        this->publica_nuvens_offline(msg_zed_image, msg_cloud);
        break;

    }

    // Fim do comando bag = 1 ou 2
    // Reseta o comando para 0 e fica em aguardo
    comando_bag = 0;

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::ProcessaOffline(){
    // Lendo a bag
    cout << "\n\nEntramos no processo offline com a bag " << caminho_bag << endl;
    rosbag::Bag bag;
    if(!caminho_bag.empty())
        bag.open(caminho_bag,rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/astra2"         ));
    topics.push_back(std::string("/astra_projetada"));
    topics.push_back(std::string("/odom2"          ));
    topics.push_back(std::string("/pixels"         ));
    topics.push_back(std::string("/zed2"           ));

    // Subscribers para sincronizar
    BagSubscriber<sensor_msgs::Image      > sub_im_as ;
    BagSubscriber<sensor_msgs::Image      > sub_im_zed;
    BagSubscriber<sensor_msgs::PointCloud2> sub_nuvem ;
    BagSubscriber<sensor_msgs::PointCloud2> sub_pixel ;
    BagSubscriber<Odometry                > sub_odom  ;
    syncoff.reset(new SyncOff(syncPolicy(100), sub_im_as, sub_im_zed, sub_nuvem, sub_pixel, sub_odom));
    funcionou_porra = syncoff->registerCallback(boost::bind(&Cloud_Work::callback_offline, this, _1, _2, _3, _4, _5));

    sensor_msgs::PointCloud2ConstPtr msg_cloud, msg_pixels;
    sensor_msgs::ImageConstPtr msg_ast_image, msg_zed_image;
    OdometryConstPtr msg_odom;

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    /// Para cada grupo de mensagens sincronizadas da bag resolver o que faz
    conjunto_dados_atual = 0;
    BOOST_FOREACH(rosbag::MessageInstance const m, view){
        if(m.getTopic() == "/astra2"){
            msg_ast_image = m.instantiate<sensor_msgs::Image>();
            if(msg_ast_image != NULL)
                sub_im_as.newMessage(msg_ast_image);
        }
        if(m.getTopic() == "/astra_projetada"){
            msg_cloud = m.instantiate<sensor_msgs::PointCloud2>();
            if(msg_cloud != NULL)
                sub_nuvem.newMessage(msg_cloud);
        }
        if(m.getTopic() == "/odom2"){
            msg_odom = m.instantiate<Odometry>();
            if(msg_odom != NULL)
                sub_odom.newMessage(msg_odom);
        }
        if(m.getTopic() == "/pixels"){
            msg_pixels = m.instantiate<sensor_msgs::PointCloud2>();
            if(msg_pixels != NULL)
                sub_pixel.newMessage(msg_pixels);
        }
        if(m.getTopic() == "/zed2"){
            msg_zed_image = m.instantiate<sensor_msgs::Image>();
            if(msg_zed_image != NULL)
                sub_im_zed.newMessage(msg_zed_image);
        }

    } // Fim do FOREACH message

    bag.close();
    vamos_de_bag = false; // Ja fizemos tudo com os dados, para o programa seguir como de costume e nao cair
    ROS_WARN("Acabamos a leitura da BAG offline, salve sua reconstrucao!!!!");
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::salva_dados_parciais(PointCloud<PointC>::Ptr cloud,
                                      const sensor_msgs::ImageConstPtr &imagem_zed,
                                      const sensor_msgs::ImageConstPtr &imagem_ast,
                                      PointCloud<PointXYZ>::Ptr nuvem_pix_total,
                                      PointCloud<PointC>::Ptr nuvem_bat){
    // Atualiza contador de imagens - tambem usado para as nuvens parciais
    contador_imagens++;

    ROS_INFO("Salvando arquivo parcial %d.", contador_imagens);

    // Nome da pasta para salvar
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/teste/";
    std::string arquivo_imzed  = pasta + std::to_string(contador_imagens) + "z.jpg";
    std::string arquivo_nuvem  = pasta + std::to_string(contador_imagens) + ".ply";
    std::string arquivo_nvm_z  = pasta + std::to_string(contador_imagens) + "z.nvm";
    std::string arquivo_corr   = pasta + std::to_string(contador_imagens) + "_cor.txt";

    // Converte e salva imagem da zed e da astra
    cv_bridge::CvImagePtr imgptr, imgzptr;
    imgzptr = cv_bridge::toCvCopy(imagem_zed, sensor_msgs::image_encodings::BGR8);
    imwrite(arquivo_imzed, imgzptr->image);
    imgptr  = cv_bridge::toCvCopy(imagem_ast, sensor_msgs::image_encodings::BGR8);

    // Comparar aqui a zed e a astra com sift e usar a correspondencia da nuvem de pixels
    comparaSift(imgptr, imgzptr, nuvem_bat);
    resolveAstraPixeis(nuvem_pix_total, nuvem_bat, imgzptr);

    ///// Escrevendo para a zed aqui /////
    Eigen::Matrix4f Tazo = T_astra_zed*T_corrigida.inverse();

    if(imagePointsZed.size() > 9)
        Tazo = T_depth_astra_zed;

    // Arquivo para conferencia de pontos 2D com 3D
    ofstream temp(arquivo_corr);
    if(temp.is_open()){
        temp << std::to_string(imagePointsZed.size())+"\n";
        for (unsigned long k=0;k<imagePointsZed.size();k++) {
            std::string linha = std::to_string(imagePointsZed[k].x) + " " + std::to_string(imagePointsZed[k].y) + "\n";
            std::replace(linha.begin(), linha.end(), ',', '.');
            temp << linha;
        }
        for (unsigned long k=0;k<objectPointsZed.size();k++) {
            std::string linha = std::to_string(objectPointsZed[k].x) + " " + std::to_string(objectPointsZed[k].y) + " " + std::to_string(objectPointsZed[k].z) + "\n";
            std::replace(linha.begin(), linha.end(), ',', '.');
            temp << linha;
        }
    } // fim do if is open
    temp.close(); // Fechar para nao ter erro

    // Chamar aqui o bat para otimizar em cima de toda a matriz de Transformacao
    // Otimizar foco, rotacao e posicao para ZED
    camera co; // camera com resultados para otimizar
    Eigen::Vector2f s(imgzptr->image.cols/2.0, imgzptr->image.rows/2.0);
    bool funcionou;
    co = bat(imagePointsZed, objectPointsZed, Tazo, fzed, s, funcionou);
    cout << endl << "Foi usado o bat? " << funcionou << endl;

    // Uma vez otimizado pelo bat a partir das correspondencias de pontos
    if(funcionou){
        Tazo = co.T;
        fzed = co.foco;
    } else {
        fzed = 1462.0;
    }
    cout << "\n Depois do bat, com foco " << fzed <<":\n";

        Eigen::Matrix3f rot_azo;
        rot_azo << Tazo.block(0, 0, 3, 3);
        Eigen::Quaternion<float>q_azo(rot_azo);
        Eigen::Vector3f t_azo;
        t_azo << Tazo(0, 3), Tazo(1, 3), Tazo(2, 3);

        // Centro da camera, para escrever no arquivo NVM
        Eigen::MatrixXf C = calcula_centro_camera(q_azo, t_azo);

        // Escreve o arquivo NVM parcial, super necessario
        ofstream nvmz(arquivo_nvm_z);
        if(nvmz.is_open()){

            nvmz << "NVM_V3\n\n";
            nvmz << "1\n"; // Quantas imagens, sempre uma aqui
            std::string linha_imagem = escreve_linha_imagem(fzed, arquivo_imzed, C, q_azo); // Imagem com detalhes de camera
            nvmz << linha_imagem; // Imagem com detalhes de camera
            // Anota no vetor de imagens que irao para o arquivo NVM da nuvem acumulada
            acumulada_imagens.push_back(linha_imagem);

        } // fim do if is open
        nvmz.close(); // Fechar para nao ter erro

        // Calcular normais para a nuvem
        pcl::PointCloud<PointCN>::Ptr final_parcial (new pcl::PointCloud<PointCN>);
        calcula_normais_com_pose_camera(final_parcial, *cloud, C, 30);

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
Eigen::MatrixXf Cloud_Work::calcula_centro_camera(Eigen::Quaternion<float> q, Eigen::Vector3f t){
    Eigen::MatrixXf C(3, 1);
    Eigen::MatrixXf R = q.matrix();
    //    Eigen::MatrixXf t(3,1);
    //    t = offset;
    C = -R.transpose()*t;

    return C;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::comparaSift(cv_bridge::CvImagePtr astra, cv_bridge::CvImagePtr zed, PointCloud<PointC>::Ptr cloud){
    /// Calculando descritores SIFT ///
    // Keypoints e descritores para astra e zed
    std::vector<cv::KeyPoint> keypointsa, keypointsz;
    cv::Mat descriptorsa, descriptorsz;
    /// Comparando e filtrando matchs ///
    cv::Ptr<cv::DescriptorMatcher> matcher;
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector< cv::DMatch > > matches;
    std::vector< cv::DMatch > good_matches;

    int tent = 0; // tentativas de achar X correspondencias bacanas
    float min_hessian = 2000;

    // Achando limites na imagem em X e Y onde podemos ter pontos 3D, estimativa inicial pra ter correspondencias
    Eigen::Matrix3f Kz;
    Kz << fzed,   0 , zed->image.cols/2.0,
            0 , fzed, zed->image.rows/2.0,
            0 ,   0 ,          1         ;
    Eigen::MatrixXf Pz = Kz*(T_astra_zed*T_corrigida.inverse()).block<3, 4>(0, 0);
    Eigen::Vector4f limites(1000, -1000, 1000, -1000), ponto;
    Eigen::Vector3f ponto_proj;
    for(size_t i=0; i<cloud->size(); i++){
        ponto << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1.0;
        ponto_proj = Pz*ponto;
        ponto_proj = ponto_proj/ponto_proj[2];
        if(ponto_proj[0] > 0 && ponto_proj[0] < zed->image.cols && ponto_proj[1] > 0 && ponto_proj[1] < zed->image.rows){
            if(ponto_proj[0] < limites[0])
                limites[0] = ponto_proj[0];
            if(ponto_proj[0] > limites[1])
                limites[1] = ponto_proj[0];
            if(ponto_proj[1] < limites[2])
                limites[2] = ponto_proj[1];
            if(ponto_proj[1] > limites[3])
                limites[3] = ponto_proj[1];
        }
    }

    cout << "\nLimites x: " << limites[0] << " " << limites[1] << endl;
    cout << "Limites y: "   << limites[2] << " " << limites[3] << endl << endl;
    goodKeypointsLeft.clear(); goodKeypointsRight.clear();

    while(goodKeypointsLeft.size() < 15 && tent < 20){
        good_matches.clear();
        Ptr<xfeatures2d::SURF> f2d = xfeatures2d::SURF::create(min_hessian);
        // Astra
        f2d->detectAndCompute(astra->image, Mat(), keypointsa, descriptorsa);
        // Zed
        f2d->detectAndCompute(zed->image  , Mat(), keypointsz, descriptorsz);

        matcher->knnMatch(descriptorsa, descriptorsz, matches, 2);

        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches.at(i).size() >= 2)
            {
                if (matches.at(i).at(0).distance < 0.75*matches.at(i).at(1).distance)
                {
                    good_matches.push_back(matches.at(i).at(0));
                }
            }
        }

        tent += 1;
        min_hessian = 0.7*min_hessian;
        //    } // alterar aqui

        // Daqui para baixo temos astra->left e zed->right

        std::vector<cv::Point2f> imgLeftPts;
        std::vector<cv::Point2f> imgRightPts;

        goodKeypointsLeft.clear();
        goodKeypointsRight.clear();

        float scaleazx = zed->image.cols/astra->image.cols, scaleazy = zed->image.rows/astra->image.rows, window = 200.0;
        cout << "antes do meu filtro por janela: " << good_matches.size() << endl;
        for (size_t i = 0; i < good_matches.size(); i++)
        {
//            KeyPoint kp_as = keypointsa[good_matches[i].queryIdx], kp_zed = keypointsz[good_matches[i].trainIdx];
            //-- Get the keypoints from the good matches
//            if(kp_zed.pt.x > kp_as.pt.x*scaleazx-window && kp_zed.pt.x < kp_as.pt.x*scaleazx+window &&
//               kp_zed.pt.y > kp_as.pt.y*scaleazy-window && kp_zed.pt.y < kp_as.pt.y*scaleazy+window   ){
                goodKeypointsLeft.push_back(keypointsa[good_matches[i].queryIdx]);
                goodKeypointsRight.push_back(keypointsz[good_matches[i].trainIdx]);
                imgLeftPts.push_back(keypointsa[good_matches[i].queryIdx].pt);
                imgRightPts.push_back(keypointsz[good_matches[i].trainIdx].pt);
//            }
        }

        cout << "Aqui quantos keypoints bons depois da minha moda? " << goodKeypointsLeft.size() << endl;
        if(imgLeftPts.size() > 0){
            cv::Mat inliers;
            cv::Mat Ka = (cv::Mat_<double>(3, 3) << fastra, 0, astra->image.cols / 2.0, 0, fastra, astra->image.rows / 2.0, 0, 0, 1);
            cv::Mat E = findEssentialMat(imgLeftPts, imgRightPts, Ka, CV_RANSAC, 0.99999, 1.0, inliers);

            std::vector<cv::KeyPoint> goodKeypointsLeftTemp;
            std::vector<cv::KeyPoint> goodKeypointsRightTemp;
            bool dx = false, dy = false;
            for (size_t i = 0; i < inliers.rows; i++)
            {
                // Filtrando aqui pelos limites provaveis da nuvem tambem
                dx = false; dy = false;
                if(goodKeypointsLeft.at(i).pt.x > limites[0] && goodKeypointsLeft.at(i).pt.x < limites[1])
                    dx = true;
                if(goodKeypointsLeft.at(i).pt.y > limites[2] && goodKeypointsLeft.at(i).pt.y < limites[3])
                    dy = true;
                if (inliers.at<uchar>(i, 0) == 1 && dx && dy)
                {
                    goodKeypointsLeftTemp.push_back(goodKeypointsLeft.at(i));
                    goodKeypointsRightTemp.push_back(goodKeypointsRight.at(i));
                }
            }
            goodKeypointsLeft = goodKeypointsLeftTemp;
            goodKeypointsRight = goodKeypointsRightTemp;
            cout << "Aqui quantos keypoints bons depois do teste inliers e limites? " << goodKeypointsLeft.size() << endl;
        }

    } // fim do while

    if(goodKeypointsLeft.size()){
        char* home;
        home = getenv("HOME");
        std::string pasta = std::string(home)+"/Desktop/teste/";
        Mat a, z;
        astra->image.copyTo(a); zed->image.copyTo(z);
        for(size_t i=0; i<goodKeypointsLeft.size(); i++){
            cv::Scalar color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
            circle(a, goodKeypointsLeft[i].pt, 5, color, 2);
            circle(z, goodKeypointsRight[i].pt, 5, color, 2);
        }
        std::string foto_zed = pasta+"fotozed.jpeg", foto_astra = pasta+"fotoastra.jpeg";
        imwrite(foto_astra, a);
        imwrite(foto_zed,   z);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::resolveAstraPixeis(PointCloud<PointXYZ>::Ptr pixeis, PointCloud<PointC>::Ptr nuvem_total_bat, cv_bridge::CvImagePtr zed){
    if(goodKeypointsLeft.size() > 0){

        cout << "goodkeypointsleft aqui: " << goodKeypointsLeft.size() << endl;

        // Jeito meu, a principio burro, mas logico
        std::vector<int> indices_nuvem;
        indices_nuvem.resize(goodKeypointsLeft.size(), -1);
        std::vector<float> melhores_distancias;
        melhores_distancias.resize(indices_nuvem.size(), 1000);
        int lim_coord = 4; // Limite de distancia ao quadrado em pixels para cada coordenada

        #pragma omp parallel for num_threads(int(goodLeftKeypoints.size()))
        for(size_t i=0; i < goodKeypointsLeft.size(); i++) {
            for(unsigned long j=0; j < pixeis->size(); j++){
                float dx2 = (pixeis->points[j].x - goodKeypointsLeft[i].pt.x)*(pixeis->points[j].x - goodKeypointsLeft[i].pt.x);
                float dy2 = (pixeis->points[j].y - goodKeypointsLeft[i].pt.y)*(pixeis->points[j].y - goodKeypointsLeft[i].pt.y);

                if(sqrt(dx2) < lim_coord && sqrt(dy2) < lim_coord && sqrt(dx2+dy2) < melhores_distancias[i]){
                    melhores_distancias[i] = sqrt(dx2+dy2);
                    indices_nuvem[i] = j;
                }
            }
        }

        // Relacionando pontos 3D com o SIFT da Zed
        imagePointsZed.clear();
        objectPointsZed.clear();
        for (unsigned long i=0; i<indices_nuvem.size(); i++)
        {
            if (indices_nuvem[i] == -1)
                continue;
            imagePointsZed.push_back(goodKeypointsRight[i].pt);
            PointC pnuvem = nuvem_total_bat->points[indices_nuvem[i]];
            cv::Point3f p(pnuvem.x, pnuvem.y, pnuvem.z);
            objectPointsZed.push_back(p);
        }

        cout << "imagePointszed aqui: " << imagePointsZed.size() << endl;

        updateRTFromSolvePNP(imagePointsZed, objectPointsZed, zed);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::updateRTFromSolvePNP(std::vector<cv::Point2f> imagePoints, std::vector<cv::Point3f> objectPoints, cv_bridge::CvImagePtr zed)
{
    if(imagePoints.size() > 4){
        cv::Mat R, t, distCoef;
        cv::Mat Ka = (cv::Mat_<double>(3, 3) << fzed, 0, zed->image.cols/2.0, 0, fzed, zed->image.rows/2.0, 0, 0, 1);
        cv::solvePnPRansac(objectPoints, imagePoints, Ka, distCoef, R, t);
        cv::Mat R_3_3;
        cv::Rodrigues(R, R_3_3);
        for (unsigned int i = 0; i < 3; i++)
        {
            for (unsigned int j = 0; j < 3; j++)
            {
                T_depth_astra_zed(i, j) = R_3_3.at<double>(i, j);
            }
        }
        for (unsigned int j = 0; j < 3; j++)
        {
            T_depth_astra_zed(j, 3) = t.at<double>(j, 0);
        }
        T_depth_astra_zed(3, 0) = T_depth_astra_zed(3, 1) = T_depth_astra_zed(3, 2) = 0; T_depth_astra_zed(3, 3) = 1;
    } else {
        // Nao ha pontos suficientes, permanece com o chute de pose e foco vindos da zed+icp
        T_depth_astra_zed = T_astra_zed*T_corrigida.inverse();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
Cloud_Work::camera Cloud_Work::bat(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, float foco_est, Eigen::Vector2f c_img, bool &valid){
    // Variavel de saida
    camera c;

    cout << "\nquantos pixels correspondentes? " << xy_zed.size() << endl;

    // Restriçoes do espaço de busca aqui - a principio somente somente translacao e foco, depois adicionar a rotacao tambem
    Eigen::MatrixXf rest(2, 4);
    float libt = 0.1, libf = 150; // Metros de liberdade em translaçao / unidade de foco, para espaço de busca
    rest << foco_est-libf, T_est(0, 3)-libt, T_est(1, 3)-libt, T_est(2, 3)-libt,
            foco_est+libf, T_est(0, 3)+libt, T_est(1, 3)+libt, T_est(2, 3)+libt;

    /// Parametros para os bats ///
    int nbats = 2000;
    float alfa = 0.5, lambda = 0.6, beta = 0.2, e = -0.1;

    // Caracteristicas dos bats (velocidade, taxa de emissao de pulso e amplitude sonora
    Eigen::MatrixXf v(nbats, rest.cols()), r(nbats, rest.cols());
    Eigen::MatrixXf As = Eigen::MatrixXf::Constant(nbats, rest.cols(), 1);
    Eigen::VectorXf F(nbats); // Vetor contendo a FOB de cada morcego
    float fob_temp = 0;

    // Iteraçoes
    int t = 0, t_max = 40, t_lim = 7;

    /// Iniciando os bats
    Eigen::MatrixXf bats(nbats, rest.cols());
    for(int i=0; i<nbats; i++){
        bats.row(i) << Eigen::MatrixXf::Random(1, rest.cols());
        F(i) = fob(xy_zed, X_zed, T_est, bats.row(i), c_img, rest);
    }
    // Variavel para o indice e menor valor do vetor de fob
    Eigen::VectorXf::Index indice_melhor_bat;
    float melhor_valor = F.minCoeff(&indice_melhor_bat);

    /// Rodando o algoritmo, realmente, vamos la ///
    float valor_anterior = melhor_valor;
    int contador_repeticoes = 0;
    while(t < t_max){
        // Controle de repeticao
        if(valor_anterior - melhor_valor <= 1e-2){
            contador_repeticoes += 1;
        } else {
            contador_repeticoes = 0;
        }
        valor_anterior = melhor_valor;

        // Se nao estourou repeticao, rodam os morcegos
        if(contador_repeticoes < t_lim){
            for(int i=0; i<nbats; i++){

                // Calculo da velocidade do morcego
                v.row(i) << v.row(i) + (bats.row(indice_melhor_bat)-bats.row(i))*beta;
                Eigen::MatrixXf bat_temp(1, 4);
                bat_temp << bats.row(i) + v.row(i);
                // Etapa de busca local
                if((double)rand()/(RAND_MAX) < r(i))
                    bat_temp << bats.row(indice_melhor_bat) + Eigen::MatrixXf::Constant(1, rest.cols(), e*As.mean());
                // Etapa de avaliacao da fob
                for(int j=0; j<bat_temp.cols(); j++){
                    if(bat_temp(j) < -1) bat_temp(j) = -1;
                    if(bat_temp(j) >  1) bat_temp(j) =  1;
                }
                fob_temp = fob(xy_zed, X_zed, T_est, bat_temp, c_img, rest);
                // Atualizando o morcego ou nao por busca global
                if((double)rand()/(RAND_MAX) < As(i) || fob_temp < F(i)){
                    bats.row(i) << bat_temp;
                    r(i)  = 1 - exp(-lambda*t);
                    As(i) = alfa*As(i);
                    F(i)  = fob_temp;
                }
                // Busca novamente pelo melhor valor dentre os morcegos pela fob
                melhor_valor = F.minCoeff(&indice_melhor_bat);

            } // fim do for de bats
            // Aumenta o contador de t iteracoes corridas
            t += 1;
            cout << "\nValor da FOB do melhor bat por pixel: " << melhor_valor/xy_zed.size() << endl;
        } else {
            break; // Ja acabou a busca aqui entao
        }

    } // fim do while t<tmax

    // Traz os valores de volta para o range original a partir do melhor bat e guarda na camera
    float f_bom  = (rest(1,0) - rest(0,0))*(bats.row(indice_melhor_bat)(0) + 1)/2 + rest(0, 0);
//    float tx_bom = (rest(1,1) - rest(0,1))*(bats.row(indice_melhor_bat)(1) + 1)/2 + rest(0, 1);
//    float ty_bom = (rest(1,2) - rest(0,2))*(bats.row(indice_melhor_bat)(2) + 1)/2 + rest(0, 2);
//    float tz_bom = (rest(1,3) - rest(0,3))*(bats.row(indice_melhor_bat)(3) + 1)/2 + rest(0, 3);
    c.foco = f_bom;
//    T_est.block<3, 1>(0, 3) << tx_bom, ty_bom, tz_bom;
    c.T << T_est;

    // Limpar os vetores
    imagePointsZed.clear(); objectPointsZed.clear();

    // Ver se o algoritmo foi capaz de otimizar
    valid = false;
    if(melhor_valor/xy_zed.size() <= 15.0)
        valid = true;

    return c;
}
///////////////////////////////////////////////////////////////////////////////////////////
float Cloud_Work::fob(std::vector<Point2f> xy_zed, std::vector<Point3f> X_zed, Eigen::Matrix4f T_est, Eigen::MatrixXf bat, Eigen::Vector2f c_img, Eigen::MatrixXf range){
    // Somatorio da fob final aqui
    float fob_final = 0;
    // Trazendo os valores de volta ao range original
    float f  = (range(1,0) - range(0,0))*(bat(0,0) + 1)/2 + range(0, 0);
    float tx = (range(1,1) - range(0,1))*(bat(0,1) + 1)/2 + range(0, 1);
    float ty = (range(1,2) - range(0,2))*(bat(0,2) + 1)/2 + range(0, 2);
    float tz = (range(1,3) - range(0,3))*(bat(0,3) + 1)/2 + range(0, 3);
    // Alterando a matriz de transformaçao
    T_est(0,3) = tx; T_est(1,3) = ty; T_est(2,3) = tz;
    // Matriz intrinseca
    Eigen::Matrix3f K_est;
    K_est << f, 0, c_img(0),
             0, f, c_img(1),
             0, 0,     1   ;
    // Ajustando matriz de transformaçao para 3x4
    Eigen::MatrixXf T(3, 4);
    T << T_est.block(0, 0, 3, 4);
    // Conferindo o tamanho dos vetores para evitar erro
    size_t pontos = (X_zed.size() <= xy_zed.size()) ? X_zed.size() : xy_zed.size();
    // Passando por todos os pontos 3D e 2D para calcular a fob final
    for(size_t i=0; i<pontos; i++){
        Eigen::Vector4f X_(X_zed[i].x, X_zed[i].y, X_zed[i].z, 1);
        Eigen::Vector3f x_goal(xy_zed[i].x, xy_zed[i].y, 1);
        Eigen::Vector3f X = T*X_;
        X = X/X(2); // Normalizando pela escala na ultima casa
        Eigen::Vector3f x = K_est*X;

        fob_final += sqrt( (x(0)-x_goal(0))*(x(0)-x_goal(0)) + (x(1)-x_goal(1))*(x(1)-x_goal(1)) );//(x - x_goal).norm();
    }

    return fob_final;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::printT(Eigen::Matrix4f T){
    cout << endl << endl;
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(0,0), T(0,1), T(0,2), T(0,3));
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(1,0), T(1,1), T(1,2), T(1,3));
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(2,0), T(2,1), T(2,2), T(2,3));
    ROS_INFO("%.4f  %.4f  %.4f  %.4f", T(3,0), T(3,1), T(3,2), T(3,3));
    cout << endl << endl;
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
    for(unsigned long i=0; i < np.size(); i++){
        ROS_INFO("Calculando normais na nuvem %zu, de %zu totais, aguarde...", i+1, np.size());
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

    savePLYFileASCII(arquivo_nuvem, *acumulada_global_normais);

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
    for(unsigned long i=0; i < acc_temp->size(); i++){
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
        for(unsigned long i=0; i < acumulada_imagens.size(); i++){ // Escreve todos os nomes acumulados de imagens
            file << acumulada_imagens[i];
        }
    }
    file.close();
    ROS_INFO("Arquivo NVM da nuvem global salvo!");

}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::reiniciar(){
    // Reinicia tudo que pode ser reiniciado aqui
    acumulada_global->clear(); acumulada_global_anterior->clear();
    acumulada_parcial->clear(); acumulada_parcial_anterior->clear();
    np.clear();

    system("gnome-terminal -x sh -c 'rosservice call /zed/reset_odometry'");

    set_primeira_vez(true); // Vamos ter a primeira vez de aquisicao novamente

    contador_imagens = 0;

    cout << "\n\n\n\nRESETOU TUDO CAMERAS E ZED\n\n\n\n" << endl;
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
void Cloud_Work::set_nome_bag(QString nome){
    caminho_bag = nome.toStdString();
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::set_dados_online_offline(bool vejamos){
    vamos_de_bag = vejamos;
}
///////////////////////////////////////////////////////////////////////////////////////////
void Cloud_Work::set_comando_bag(int c){
    comando_bag = c;
}

} // Fim do namespace handset_gui
