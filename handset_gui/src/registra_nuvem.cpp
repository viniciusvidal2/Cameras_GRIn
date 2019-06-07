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

    // Inicia a transformaçao
    T_fim = Eigen::Matrix4f::Identity();
    R_fim = Eigen::Matrix3f::Identity();
    t_fim << 0.0, 0.0, 0.0;

    // Publicadores (a principio)
    pub_srctemp   = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_fonte_temp"    , 1);
    pub_tgt       = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_alvo_temp"     , 1);
    pub_acumulada = nh_.advertise<sensor_msgs::PointCloud2>("/nuvem_acumulada_temp", 1);

    ros::Rate rate(2);
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
    if(processando){
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
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_alvo(QString nome){
    arquivo_tgt = nome.toStdString();
    loadPLYFile(arquivo_tgt, *tgt);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    for(int i=nome.length(); i>0; i--){
      if (nome[i] == '/'){
        pasta_tgt = nome.left(i).toStdString();
        break;
      }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_arquivo_cameras_alvo(QString nome){
    arquivo_cameras_alvo = nome.toStdString();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_nuvem_fonte(QString nome){
    arquivo_src = nome.toStdString();
    loadPLYFile(arquivo_src, *src);
    // Recolher aqui tambem so o nome da pasta pra fazer o arquivo final depois
    for(int i=nome.length(); i>0; i--){
      if (nome[i] == '/'){
        pasta_src = nome.left(i).toStdString();
        break;
      }
    }
    // A principio as nuvens sao iguais, depois havera modificacao
    *src_temp = *src;
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_arquivo_cameras_fonte(QString nome){
    arquivo_cameras_fonte = nome.toStdString();
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_translacao(float tx, float ty, float tz){
    // Recebe em Centimetros, converte pra METROS
    t_fim << tx/100.0, ty/100.0, tz/100.0;
    criaMatriz();

    src_temp->clear();
    transformPointCloudWithNormals(*src, *src_temp, T_fim);
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::set_rotacao(float rx, float ry, float rz){
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
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::registrar_nuvens(bool icp_flag){
    // Se vamos usar o ICP ou nao, decide aqui
    if(icp_flag){

        // Recebe a matriz de transformacao final do ICP
        Eigen::Matrix4f Ticp = icp(src, tgt, T_fim);
        // Transforma de forma fina para a src_temp, para nao perder a src
        transformPointCloud(*src, *src_temp, Ticp);
        *acumulada = *tgt + *src_temp;
        // Guarda para escrever no arquivo de cameras
        T_fim = Ticp;

    } else {

        *acumulada = *tgt + *src_temp;

    }
}
///////////////////////////////////////////////////////////////////////////////////////////
void RegistraNuvem::filter_grid(PointCloud<PointT>::Ptr cloud, float leaf_size){
    VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
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

    float leaf_size = 0.01;
    filter_grid(temp_src, leaf_size);
    filter_grid(temp_tgt, leaf_size);

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
    int conta_linha = 0;

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
                Eigen::Matrix4f Tzo, Toz;

                // Foco da camera
                foco = stod(results.at(0));
                // Quaternion antigo
                qantes.w() = stod(results.at(1)); qantes.x() = stod(results.at(2));
                qantes.y() = stod(results.at(3)); qantes.z() = stod(results.at(4));
                rotantes = qantes.matrix();
                // Centro da camera antigo
                Cantes(0) = stod(results.at(5)); Cantes(1) = stod(results.at(6)); Cantes(2) = stod(results.at(7));
                // Vetor de translaçao anterior
                tantes = -rotantes.inverse()*Cantes;
                // Calculo de T entre frames ODOM->ZED anterior
                Tzo << rotantes, tantes,
                       0, 0, 0, 1;
                Toz = Tzo.inverse();
                // Nova matriz de transformaçao ODOM->ZED após a correçao do algoritmo
                Toz = Toz*T_fim;
                // Matriz inversa (ZED->ODOM) para calculo da nova pose da camera
                Tzo = Toz.inverse();
                Rzo << Tzo(0, 0), Tzo(0, 1), Tzo(0, 2),
                       Tzo(1, 0), Tzo(1, 1), Tzo(1, 2),
                       Tzo(2, 0), Tzo(2, 1), Tzo(2, 2);
                Eigen::Quaternion<float> qzo(Rzo); // Novo quaternion
                tatual << Tzo(0, 3), Tzo(1, 3), Tzo(2, 3);
                Catual = -Rzo*tatual; // Novo centro da camera

                // Nome da imagem
                QString path2 = QString::fromStdString(path);
                for(int i=path2.length(); i > 0; i--){
                    if(path2[i] == '/'){
                        cam.nome_imagem_anterior = path2.right(path2.length()-i-1).toStdString();
                        cam.nome_imagem = "src"+path2.right(path2.length()-i-1).toStdString();
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

    cout << "arquivo cameras alvo: " << arquivo_cameras_alvo << endl;

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

                // Nome da imagem
                QString path2 = QString::fromStdString(path);
                for(int i=path2.length(); i > 0; i--){
                    if(path2[i] == '/'){
                        cam.nome_imagem_anterior = path2.right(path2.length()-i-1).toStdString();
                        cam.nome_imagem = "tgt"+path2.right(path2.length()-i-1).toStdString();
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

        // Gravar a nuvem registrada no diretorio final
        std::string arquivo_nuvem_final = pasta_final + "/nuvem_final.ply";
        savePLYFileASCII(arquivo_nuvem_final, *acumulada);

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

} // fim do namespace handset_gui
