#include "../include/handset_gui/registra_nuvem.hpp"
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
///////////////////////////////////////////////////////////////////////////////////////////
RegistraNuvem::RegistraNuvem(int argc, char** argv):init_argc(argc), init_argv(argv){
    QFuture<void> future = QtConcurrent::run(this, &RegistraNuvem::init);
}
///////////////////////////////////////////////////////////////////////////////////////////
RegistraNuvem::~RegistraNuvem(){
    if(ros::isStarted())
        ros::shutdown();
    wait();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::init(){
    ros::init(init_argc, init_argv, "RegistraNuvem");
    if ( !ros::master::check() )  {
        cout << "check ros master not good" << endl;
        return;
    }
    ros::start();
    ros::NodeHandle nh_;

    // Inicia as nuvens
    src       = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    src_temp  = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    tgt       = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    acumulada = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    nuvem_filtrar      = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();
    nuvem_filtrar_temp = (PointCloud<PointT>::Ptr) new PointCloud<PointT>();

    // Inicia a transformaçao
    T_fim = Eigen::Matrix4f::Identity();
    R_fim = Eigen::Matrix3f::Identity();
    t_fim << 0.0, 0.0, 0.0;

    // Definicao de rotacao fixa entre frame da ASTRA e da ZED -> de ASTRA->ZED A PRINCIPIO
    Eigen::Matrix3f matrix;
    matrix = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::Quaternion<float> rot_temp(matrix);
    rot_astra_zed = rot_temp.inverse(); // Aqui esta de ZED->ASTRA (nuvens)
//    offset_astra_zed << 0.04936, 0.034, -0.00314; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, da LEFT_ZED para ASTRA_RGB
//    offset_astra_zed << 0.052, 0.01, -0.01; // No frame da ASTRA, apos rotaçao de ZED->ASTRA, por reconfigure com imagem raw objetos proximos
    offset_astra_zed << 0, 0, 0; // Já resolvido antes de salvar na outra classe
    // Matriz de transformaçao que leva ASTRA->ZED, assim pode calcular posicao da CAMERA ao multiplicar por ZED->ODOM
    T_astra_zed << matrix, offset_astra_zed,
                   0, 0, 0, 1;

    // Publicadores (a principio)
    pub_srctemp   = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_fonte_temp"    , 1);
    pub_tgt       = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_alvo_temp"     , 1);
    pub_acumulada = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_acumulada_temp", 1);
    pub_filtrada  = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_filtrada"      , 1);

    // Mutex de publicacao
    mutex_publicar = true;

    // Estamos na aba3?
    aba3 = false;

    ros::Rate rate(0.5);
    while(ros::ok()){
        rate.sleep();
        publicar_nuvens();
    }

    ros::shutdown();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_inicio_processo(bool inicio){
    processando = inicio;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::criaMatriz(){
    T_fim << R_fim, t_fim,
             0, 0, 0, 1;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::publicar_nuvens(){
    sensor_msgs::PointCloud2 src_temp_msg, tgt_msg, acumulada_msg;
    std::string frame = "registro";
    if(processando && mutex_publicar && !aba3){
        if(src_temp->size() > 0){
            toROSMsg(*src_temp, src_temp_msg);
            src_temp_msg.header.frame_id = frame;
            pub_srctemp.publish(src_temp_msg);
        }
        if(tgt->size() > 0){
            toROSMsg(*tgt, tgt_msg);
            tgt_msg.header.frame_id = frame;
            pub_tgt.publish(tgt_msg);
        }
        if(acumulada->size() > 0){
            toROSMsg(*acumulada, acumulada_msg);
            acumulada_msg.header.frame_id = frame;
            pub_acumulada.publish(acumulada_msg);
        }
    }
    // Aqui quando publicando na terceira aba de filtragem somente
    if(mutex_publicar && aba3){
        if(nuvem_filtrar_temp->size() > 0){
            sensor_msgs::PointCloud2 filtrada_msg;
            toROSMsg(*nuvem_filtrar_temp, filtrada_msg);
            filtrada_msg.header.frame_id = frame;
            pub_filtrada.publish(filtrada_msg);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_alvo(QString nome){
    mutex_publicar = false;
    arquivo_tgt = nome.toStdString();
    loadPLYFile(arquivo_tgt, *tgt);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    std::string nome_arquivo_sozinho;
    for(int i=nome.length(); i>0; i--){
      if (nome[i] == '/'){
        pasta_tgt = nome.left(i).toStdString();        
        nome_arquivo_sozinho = nome.right(nome.length()-i-1).toStdString();
        break;
      }
    }
    // Calcular centroide para deslocar nuvem
    ROS_INFO("Deslocando nuvem alvo para o centro do frame...");
    centroide_tgt = this->calcula_centroide(tgt);
    transformPointCloud(*tgt, *tgt,  -centroide_tgt, Eigen::Quaternion<float>::Identity());
    ROS_INFO("Nuvem alvo deslocada.");
    mutex_publicar = true;

    // Já adicionar o arquivo nvm da nuvem aqui de uma vez
    nome_arquivo_sozinho = nome_arquivo_sozinho.substr(0, nome_arquivo_sozinho.find_first_of('.'));
    arquivo_cameras_alvo = pasta_tgt + "/" + nome_arquivo_sozinho + ".nvm";
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_arquivo_cameras_alvo(QString nome){
    arquivo_cameras_alvo = nome.toStdString();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_fonte(QString nome){
    mutex_publicar = false;
    arquivo_src = nome.toStdString();
    loadPLYFile(arquivo_src, *src);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    std::string nome_arquivo_sozinho;
    for(int i=nome.length(); i>0; i--){
      if (nome[i] == '/'){
        pasta_src = nome.left(i).toStdString();
        nome_arquivo_sozinho = nome.right(nome.length()-i-1).toStdString();
        break;
      }
    }
    // Calcular centroide para deslocar nuvem
    ROS_INFO("Deslocando nuvem fonte para o centro do frame...");
    centroide_src = this->calcula_centroide(src);
    transformPointCloud(*src, *src,  -centroide_src, Eigen::Quaternion<float>::Identity());
    ROS_INFO("Nuvem fonte deslocada.");
    // A principio as nuvens sao iguais, depois havera modificacao
    *src_temp = *src;
    mutex_publicar = true;

    // Já adicionar o arquivo nvm da nuvem aqui de uma vez
    nome_arquivo_sozinho = nome_arquivo_sozinho.substr(0, nome_arquivo_sozinho.find_first_of('.'));
    arquivo_cameras_fonte = pasta_src + "/" + nome_arquivo_sozinho + ".nvm";
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_arquivo_cameras_fonte(QString nome){
    arquivo_cameras_fonte = nome.toStdString();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_translacao(float tx, float ty, float tz){
    mutex_publicar = false;
    // Recebe em Centimetros, converte pra METROS
    t_fim << tx/100.0, ty/100.0, tz/100.0;
    criaMatriz();

    src_temp->clear();
    transformPointCloudWithNormals(*src, *src_temp, T_fim);
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_rotacao(float rx, float ry, float rz){
    mutex_publicar = false;
    // Recebe em GRAUS, converte pra radianos
    rx = deg2rad(rx);
    ry = deg2rad(ry);
    rz = deg2rad(rz);

    R_fim = Eigen::AngleAxisf(rx, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(ry, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(rz, Eigen::Vector3f::UnitZ());
    criaMatriz();

    src_temp->clear();
    transformPointCloudWithNormals(*src, *src_temp, T_fim);
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::registrar_nuvens(bool icp_flag){
    mutex_publicar = false;
    // Se vamos usar o ICP ou nao, decide aqui
    if(icp_flag){

        // Recebe a matriz de transformacao final do ICP
        Eigen::Matrix4f Ticp = icp(src, tgt, T_fim);
        // Transforma de forma fina para a src_temp, para nao perder a src
        transformPointCloudWithNormals(*src, *src_temp, Ticp);
        *acumulada = *tgt + *src_temp;
        // Guarda para escrever no arquivo de cameras
        T_fim = Ticp;

    } else {

        *acumulada = *tgt + *src_temp;
        criaMatriz();

    }
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::filter_grid(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, float leaf_size){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(in);
    grid.filter(*out);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::get_TFinal(float &x, float &y, float &z, float &rx, float &ry, float &rz){
    x = 100*T_fim(0, 3); y = 100*T_fim(1, 3); z = 100*T_fim(2, 3); // em centimetros aqui

    cout << "translacao final: " << x << " " << y << " " << z << endl;

    Eigen::Matrix3f rot;
    rot = T_fim.block(0, 0, 3, 3);
    Eigen::Vector3f rpy;
    rpy = rot.eulerAngles(0, 1, 2);

    cout << "vetor de angulos: " << 180/M_PI*rpy << endl;

    rx = rad2deg(rpy[0]); ry = rad2deg(rpy[1]); rz = rad2deg(rpy[2]);
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f RegistraNuvem::icp(const PointCloud<PointT>::Ptr src,
                                   const PointCloud<PointT>::Ptr tgt,
                                   Eigen::Matrix4f T){
    ROS_INFO("Entrando no ICP");
    // Reduzindo complexidade das nuvens
    PointCloud<PointT>::Ptr temp_src (new PointCloud<PointT>());
    PointCloud<PointT>::Ptr temp_tgt (new PointCloud<PointT>());

    *temp_src = *src; *temp_tgt = *tgt;

    float leaf_size;
    if(profundidade_icp == 0){
        leaf_size = 0.01;
    } else {
        leaf_size = profundidade_icp;
    }
    filter_grid(temp_src, leaf_size);
    filter_grid(temp_tgt, leaf_size);

    Eigen::Matrix4f T_icp = T;

    /// ICP COMUM ///
    // Criando o otimizador de ICP comum
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setUseReciprocalCorrespondences(true);
    icp.setInputTarget(temp_tgt);
    icp.setInputSource(temp_src);
    icp.setMaximumIterations(100); // Chute inicial bom 10-100
    icp.setTransformationEpsilon(1*1e-10);
    icp.setEuclideanFitnessEpsilon(1*1e-12);
    icp.setMaxCorrespondenceDistance(0.1);

    #pragma omp parallel
    PointCloud<PointT> final2;
    icp.align(final2, T);

    if(icp.hasConverged())
        T_icp = icp.getFinalTransformation();

    temp_src->clear(); temp_tgt->clear();

    ROS_INFO("ICP realizado.");

    return T_icp;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::salvar_dados_finais(QString pasta){
    // Reiniciando vetores de dados de cameras
    cameras_src.clear(); cameras_tgt.clear();

    // Nova pasta no Desktop
    char* home;
    home = getenv("HOME");
    std::string pasta_final = std::string(home)+"/Desktop/"+pasta.toStdString();

    // Ler os arquivos
    ifstream nvm_src, nvm_tgt;
    std::string linha_atual;
    int conta_linha = 0, conta_imagem = 1;

    nvm_src.open(arquivo_cameras_fonte);
    if(nvm_src.is_open()){
        // For ao longo das linhas, ler as poses
        while(getline(nvm_src, linha_atual)){

            conta_linha++; // atualiza aqui para pegar o numero 1 na primeira e assim por diante

            if(conta_linha >= 4){ // A partir daqui tem cameras
                // Elemento da estrutura da camera atual
                camera cam;
                // Separando a string de entrada em nome e dados numericos
                int pos = linha_atual.find_first_of(' ');
                std::string path = linha_atual.substr(0, pos);
                std::string numericos = linha_atual.substr(pos+1);
                std::replace(numericos.begin(), numericos.end(), '.', ',');
                // Elementos divididos por espaços
                std::istringstream ss(numericos);
                std::vector<std::string> results((std::istream_iterator<std::string>(ss)), std::istream_iterator<std::string>());

                float foco;
                Eigen::Quaternion<float> qantes;
                Eigen::Matrix3f rotantes, Rzo;
                Eigen::Vector3f Cantes, tantes, Catual, tatual;
                Eigen::Matrix4f Tpose, Tzo, Toz;

                // Foco da camera
                foco = stod(results.at(0));
                // Quaternion antigo
                qantes.w() = stod(results.at(1)); qantes.x() = stod(results.at(2));
                qantes.y() = stod(results.at(3)); qantes.z() = stod(results.at(4));
                rotantes = qantes.matrix();
                // Centro da camera antigo
                Cantes(0) = stod(results.at(5)); Cantes(1) = stod(results.at(6)); Cantes(2) = stod(results.at(7));
                // Alterando devido ao novo centroide da nuvem
                Cantes = Cantes - centroide_src;
                // Vetor de translaçao anterior
                tantes = -rotantes.transpose().inverse()*Cantes; // t = -(R')^-1*C = -R*C
                // Calculo de da POSE da camera anterior em matriz homogenea
                Tpose << rotantes, tantes,
                       0, 0, 0, 1;
                // Separando a parte da odometria, no sentido ZED->ODOM
                Tzo = T_astra_zed.inverse()*Tpose;
                // Multiplicadno pela correcao obtida, no sentido ZED->ODOM
                Tzo = Tzo*T_fim.inverse();
                // Matriz de transformacao de ASTRA-> ZED com a matriz ZED->ODOM para calculo da nova pose da camera
                Tzo = T_astra_zed*Tzo;
                Rzo << Tzo(0, 0), Tzo(0, 1), Tzo(0, 2),
                       Tzo(1, 0), Tzo(1, 1), Tzo(1, 2),
                       Tzo(2, 0), Tzo(2, 1), Tzo(2, 2);
                Eigen::Quaternion<float> qzo(Rzo); // Novo quaternion
                tatual << Tzo(0, 3), Tzo(1, 3), Tzo(2, 3);
                Catual = -Rzo.transpose()*tatual; // Novo centro da camera

                // Nome da imagem
                QString path2 = QString::fromStdString(path);
                for(int i=path2.length(); i > 0; i--){
                    if(path2[i] == '/'){
                        cam.nome_imagem_anterior = path2.right(path2.length()-i-1).toStdString();
                        cam.nome_imagem = std::to_string(conta_imagem)+".jpg";// "src"+path2.right(path2.length()-i-1).toStdString();
                        conta_imagem++;
                        break;
                    }
                }

                // Preenchendo a estrutura da camera atual
                cam.linha = linha_atual;
                cam.caminho_original = path;
                cam.foco = foco;
                cam.q_original = qantes;
                cam.q_modificado = qzo;
                cam.C_original = Cantes;
                cam.C_modificado = Catual;

                // Salvando a estrutura no vetor de cameras source
                cameras_src.push_back(cam);

            } // Fim do if para iteracao de linhas
        } // Fim do while linhas

    } // Fim do if open para arquivo src
    nvm_src.close();

    conta_linha = 0; // Reiniciando a leitura de arquivo

    nvm_tgt.open(arquivo_cameras_alvo);
    if(nvm_tgt.is_open()){

        // For ao longo das linhas, ler as poses
        while(getline(nvm_tgt, linha_atual)){
            conta_linha++; // atualiza aqui para pegar o numero 1 na primeira e assim por diante

            if(conta_linha >= 4){ // A partir daqui tem cameras
                // Elemento da estrutura da camera atual
                camera cam;
                // Separando a string de entrada em nome e dados numericos
                int pos = linha_atual.find_first_of(' ');
                std::string path = linha_atual.substr(0, pos);
                std::string numericos = linha_atual.substr(pos+1);
                std::replace(numericos.begin(), numericos.end(), '.', ',');
                // Elementos divididos por espaços
                std::istringstream iss(numericos);
                std::vector<std::string> results((std::istream_iterator<std::string>(iss)), std::istream_iterator<std::string>());

                float foco;
                Eigen::Quaternion<float> qantes;
                Eigen::Vector3f Cantes;

                // Foco da camera
                foco = stod(results.at(0));
                // Quaternion antigo
                qantes.w() = stod(results.at(1)); qantes.x() = stod(results.at(2));
                qantes.y() = stod(results.at(3)); qantes.z() = stod(results.at(4).data());
                // Centro da camera antigo
                Cantes(0) = stof(results.at(5)); Cantes(1) = stof(results.at(6)); Cantes(2) = stof(results.at(7));
                // Alterando devido ao novo centroide da nuvem
                Cantes = Cantes - centroide_tgt;
                // Nome da imagem
                QString path2 = QString::fromStdString(path);
                for(int i=path2.length(); i > 0; i--){
                    if(path2[i] == '/'){
                        cam.nome_imagem_anterior = path2.right(path2.length()-i-1).toStdString();
                        cam.nome_imagem = std::to_string(conta_imagem)+".jpg";// "tgt"+path2.right(path2.length()-i-1).toStdString();
                        conta_imagem++;
                        break;
                    }
                }

                // Preenchendo a estrutura da camera atual
                cam.linha = linha_atual;
                cam.caminho_original = path;
                cam.foco = foco;
                cam.q_original = qantes;
                cam.q_modificado = qantes;
                cam.C_original = Cantes;
                cam.C_modificado = Cantes;

                // Salvando a estrutura no vetor de cameras target
                cameras_tgt.push_back(cam);

            } // Fim do if para iteracao de linhas
        } // Fim do while linhas

    } // Fim do if open para arquivo tgt
    nvm_tgt.close();

    // Criar nova pasta na area de trabalho
    const int dir_err = mkdir(pasta_final.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if( !(dir_err == -1) ){ // Nao houve erro na criacao do diretorio
        // Escrever o novo arquivo
        ROS_INFO("Salvando arquivo NVM final.");
        std::string arquivo_final = pasta_final+"/nuvem_final.nvm";
        ofstream nvm_final(arquivo_final);
        if(nvm_final.is_open()){

            nvm_final << "NVM_V3\n\n";
            nvm_final << std::to_string(cameras_src.size()+cameras_tgt.size())+"\n"; // Quantas imagens total
            // Escreve as linhas para nuvem src
            for(int i=0; i < cameras_src.size(); i++){
                std::string linha_imagem = escreve_linha_imagem(pasta_final, cameras_src.at(i)); // Imagem com detalhes de camera
                nvm_final << linha_imagem;
            }
            // Escreve as linhas para nuvem tgt
            for(int i=0; i < cameras_tgt.size(); i++){
                std::string linha_imagem = escreve_linha_imagem(pasta_final, cameras_tgt.at(i)); // Imagem com detalhes de camera
                nvm_final << linha_imagem;
            }

        } // Fim do if arquivo is open
        nvm_final.close();

        ROS_INFO("Arquivo NVM salvo na pasta %s.", pasta_final.c_str());

        ROS_INFO("Copiando fotos e criando nuvens na pasta %s...", pasta_final.c_str());

        // Mover cada imagem para a nova pasta
        for(int i=0; i < cameras_src.size(); i++){
            std::string caminho_saida = pasta_src+"/"+cameras_src.at(i).nome_imagem_anterior;
            std::string caminho_final = pasta_final+"/"+cameras_src.at(i).nome_imagem;
            boost::filesystem::copy_file(caminho_saida.c_str(), caminho_final.c_str(), boost::filesystem::copy_option::overwrite_if_exists);
        }
        for(int i=0; i < cameras_tgt.size(); i++){
            std::string caminho_saida = pasta_tgt+"/"+cameras_tgt.at(i).nome_imagem_anterior;
            std::string caminho_final = pasta_final+"/"+cameras_tgt.at(i).nome_imagem;
            boost::filesystem::copy_file(caminho_saida.c_str(), caminho_final.c_str(), boost::filesystem::copy_option::overwrite_if_exists);
        }

        // Gravar a nuvem registrada no diretorio final, junto com a nuvem fonte e a alvo deslocadas
        std::string arquivo_nuvem_final = pasta_final + "/nuvem_final.ply";
        std::string arquivo_fonte_final = pasta_final + "/nuvem_fonte.ply";
        std::string arquivo_alvo_final  = pasta_final + "/nuvem_alvo.ply" ;
        savePLYFileASCII(arquivo_fonte_final, *src_temp );
        savePLYFileASCII(arquivo_alvo_final , *tgt      );
        savePLYFileASCII(arquivo_nuvem_final, *acumulada);

        ROS_INFO("Arquivos criados com sucesso.");

    } // Fim do if para mkdir



}
///////////////////////////////////////////////////////////////////////////////////////////
std::string RegistraNuvem::escreve_linha_imagem(std::string pasta, camera c){
    std::string linha = pasta+"/"+c.nome_imagem;
    // Adicionar foco
    linha = linha + " " + std::to_string(c.foco);
    // Adicionar quaternion
    linha = linha + " " + std::to_string(c.q_modificado.w()) + " " + std::to_string(c.q_modificado.x()) + " " + std::to_string(c.q_modificado.y()) + " " + std::to_string(c.q_modificado.z());
    // Adicionar centro da camera
    linha = linha + " " + std::to_string(c.C_modificado(0, 0)) + " " + std::to_string(c.C_modificado(1, 0)) + " " + std::to_string(c.C_modificado(2, 0));
    // Adicionar distorcao radial (crendo 0) e 0 final
    linha = linha + " 0 0\n"; // IMPORTANTE pular linha aqui, o MeshRecon precisa disso no MART
    // Muda as virgulas por pontos no arquivo
    std::replace(linha.begin(), linha.end(), ',', '.');
    return linha;
}
///////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3f RegistraNuvem::calcula_centroide(PointCloud<PointT>::Ptr cloud){
    float x = 0, y = 0, z = 0;
    for(unsigned long i=0; i < cloud->size(); i++){
        x = x + cloud->points[i].x;
        y = y + cloud->points[i].y;
        z = z + cloud->points[i].z;
    }
    Eigen::Vector3f centroide;
    centroide << x/(float)cloud->size(), y/(float)cloud->size(), z/(float)cloud->size();
    // Retorna centro medio dos pontos da nuvem, na transformacao deve ser passado negativo
    return centroide;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::filter_color(PointCloud<PointT>::Ptr cloud_in, int rmin, int rmax, int gmin, int gmax, int bmin, int bmax){
  int rMax = rmax;
  int rMin = rmin;
  int gMax = gmax;
  int gMin = gmin;
  int bMax = bmax;
  int bMin = bmin;

  ConditionAnd<PointT>::Ptr color_cond (new ConditionAnd<PointT> ());
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("r", ComparisonOps::LT, rMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("r", ComparisonOps::GT, rMin)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("g", ComparisonOps::LT, gMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("g", ComparisonOps::GT, gMin)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("b", ComparisonOps::LT, bMax)));
  color_cond->addComparison (PackedRGBComparison<PointT>::Ptr (new PackedRGBComparison<PointT> ("b", ComparisonOps::GT, bMin)));

  // build the filter
  ConditionalRemoval<PointT> condrem;
  condrem.setCondition(color_cond);
  condrem.setInputCloud(cloud_in);
  condrem.setKeepOrganized(true);

  // apply filter
  condrem.filter (*cloud_in);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::remove_outlier(PointCloud<PointT>::Ptr in, PointCloud<PointT>::Ptr out, float mean, float deviation){
  StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(in);
  sor.setMeanK(mean);
  sor.setStddevMulThresh(deviation);
  sor.filter(*out);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_filtrar(QString n){
    aba3 = true; // Vai funcionar e publicar as coisas dessa aba

    loadPLYFile(n.toStdString(), *nuvem_filtrar);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    for(int i=n.length(); i>0; i--){
      if (n[i] == '/'){
        pasta_filtrada = n.left(i).toStdString();
        break;
      }
    }
    *nuvem_filtrar_temp = *nuvem_filtrar;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_new_voxel(float v){
    mutex_publicar = false;
    filter_grid(nuvem_filtrar_temp, nuvem_filtrar_temp, v);
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_new_outlier(float k, float stddev){
    mutex_publicar = false;
    remove_outlier(nuvem_filtrar_temp, nuvem_filtrar_temp, k, stddev);
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_filter_colors(int rmin, int rmax, int gmin, int gmax, int bmin, int bmax){
    mutex_publicar = false;
    filter_color(nuvem_filtrar_temp, rmin, rmax, gmin, gmax, bmin, bmax);
    mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_profundidade_icp(double p){
    profundidade_icp = p;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::salvar_nuvem_filtrada(QString nome){
    std::string caminho = pasta_filtrada+"/"+nome.toStdString()+".ply";
    ROS_INFO("Salvando nuvem filtrada como %s...", caminho.c_str());
    savePLYFileASCII(caminho, *nuvem_filtrar_temp);
    ROS_INFO("Nuvem filtrada salva.");
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::reseta_filtros(){
    *nuvem_filtrar_temp = *nuvem_filtrar;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::aplica_filtro_polinomio(int grau){
  mutex_publicar = false;

  PointCloud<PointXYZRGB>::Ptr temp (new PointCloud<PointXYZRGB>());
  PointCloud<PointT>::Ptr tempnormal (new PointCloud<PointT>());
  PointXYZRGB point;
  for(unsigned long i=0; i<nuvem_filtrar_temp->size(); i++){
    point.x = nuvem_filtrar_temp->points[i].x;
    point.y = nuvem_filtrar_temp->points[i].y;
    point.z = nuvem_filtrar_temp->points[i].z;
    point.r = nuvem_filtrar_temp->points[i].r;
    point.g = nuvem_filtrar_temp->points[i].g;
    point.b = nuvem_filtrar_temp->points[i].b;
    temp->push_back(point);
  }
  ROS_INFO("Tamanho da nuvem %zu.", temp->size());
  ROS_INFO("Começando a suavizar a nuvem...");
  pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>());
  MovingLeastSquares<PointXYZRGB, PointT> mls;
  mls.setSearchRadius(0.05);
  mls.setComputeNormals(false);
  mls.setSearchMethod(tree);
  mls.setPolynomialOrder(grau);
  mls.setInputCloud(temp);
  mls.process(*tempnormal);
  ROS_INFO("Nuvem suavizada!");
  for(unsigned long i=0; i<nuvem_filtrar_temp->size(); i++){
    nuvem_filtrar_temp->points[i].x = tempnormal->points[i].x;
    nuvem_filtrar_temp->points[i].y = tempnormal->points[i].y;
    nuvem_filtrar_temp->points[i].z = tempnormal->points[i].z;
  }

  mutex_publicar = true;
}
///////////////////////////////////////////////////////////////////////////////////////////
} // fim do namespace handset_gui
