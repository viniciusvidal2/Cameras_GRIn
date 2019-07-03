/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/handset_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace handset_gui {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent), qnode(argc,argv), cw(argc, argv, &mutex), rn(argc, argv){

    // Inicia UI
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    // Ajustes da UI
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.aba_1->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    controle_gravacao = false; // Nao estamos gravando ainda
    ui.pushButton_gravardados->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)");
    ui.pushButton_capturar->setEnabled(false); // So se as cameras ligarem ele habilita
    ui.checkBox_online->setChecked(false);

    qnode.init();

    // Inicia a classe que trabalha as nuvens, principal do programa
    cw.set_inicio_acumulacao(false);
    cw.set_primeira_vez(true);
    cw.set_inicio_acumulacao(ui.lineEdit_tempo->text().toFloat()); // Tempo default para garantir

    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    ui.checkBox_icp->setChecked(true); // Checkbox do icp começa a principio valendo
    // Ajustando Sliders com limites e valores iniciais
    ui.horizontalSlider_x->setMinimum(-(ui.lineEdit_limitex->text().toFloat()));
    ui.horizontalSlider_x->setMaximum(ui.lineEdit_limitex->text().toFloat());
    ui.horizontalSlider_x->setValue(0);
    ui.horizontalSlider_y->setMinimum(-(ui.lineEdit_limitey->text().toFloat()));
    ui.horizontalSlider_y->setMaximum(ui.lineEdit_limitey->text().toFloat());
    ui.horizontalSlider_y->setValue(0);
    ui.horizontalSlider_z->setMinimum(-(ui.lineEdit_limitez->text().toFloat()));
    ui.horizontalSlider_z->setMaximum(ui.lineEdit_limitez->text().toFloat());
    ui.horizontalSlider_z->setValue(0);
    // Ajustando Dials com limites e valores iniciais -> GRAUS AQUI
    ui.dial_x->setMinimum(-180);
    ui.dial_x->setMaximum(180);
    ui.dial_x->setValue(0);
    ui.dial_y->setMinimum(-180);
    ui.dial_y->setMaximum(180);
    ui.dial_y->setValue(0);
    ui.dial_z->setMinimum(-180);
    ui.dial_z->setMaximum(180);
    ui.dial_z->setValue(0);

    // Desabilitando componentes da aba 3
    ui.groupBox_filtrosforma->setEnabled(false);
    ui.groupBox_cores->setEnabled(false);
    ui.frame_salvar->setEnabled(false);
    ui.pushButton_resetafiltro->setEnabled(false);
    ui.groupBox_polinomio->setEnabled(false);

}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    // Terminar tudo no sistema para nao conflitar com a proxima abertura
    system("gnome-terminal -x sh -c 'rosnode kill --all'");

    QMainWindow::closeEvent(event);
}

/// Botao de inicio do sistemas de cameras
void MainWindow::on_pushButton_inicio_clicked(){
    if(ui.checkBox_online->isChecked())
        system("gnome-terminal -x sh -c 'roslaunch handset_gui lancar_cameras.launch online:=true'");
    else
        system("gnome-terminal -x sh -c 'roslaunch handset_gui lancar_cameras.launch online:=false'");
    ui.pushButton_capturar->setEnabled(true); // Agora pode capturar
}

/// Botao para iniciar a captura corretamente por tanto tempo
void MainWindow::on_pushButton_capturar_clicked(){
    float t = ui.lineEdit_tempo->text().toFloat();
    cw.set_n_nuvens_aquisicao(t); // Numero de nuvens para aquisitar no instante
    cw.set_profundidade_max(ui.lineEdit_profundidade->text().toFloat()); // Profundidade para filtrar as nuvens capturadas
    cw.set_inicio_acumulacao(true); // Liberar a flag de inicio de aquisicao
}

/// Botao para gravar a parar gravacao da bag
void MainWindow::on_pushButton_gravardados_clicked(){
    std::string bag = ui.lineEdit_nomebag->text().toStdString();
    if(controle_gravacao){ // Estamos gravando, clica para parar, fica verde
        system("rosnode kill /rosbag_record");
        ui.pushButton_gravardados->setText("Gravar Dados");
        ui.pushButton_gravardados->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)");
        controle_gravacao = false;
    } else { // Nao estamos gravando, clica para comecar, fica vermelho
        std::string comando = "gnome-terminal -x sh -c 'roslaunch handset_gui record.launch bag:="+bag+"'";
        system(comando.c_str());
        ui.pushButton_gravardados->setText("Parar gravacao");
        ui.pushButton_gravardados->setStyleSheet("background-color: rgb(200, 0, 0); color: rgb(0, 0, 0)");
        controle_gravacao = true;
    }
}

/// Botao para chamar o RVIZ
void MainWindow::on_pushButton_visualizar_clicked(){
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/handsets_ws/src/Cameras_GRIn/handset_gui/resources/inst.rviz'");
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/handsets_ws/src/Cameras_GRIn/handset_gui/resources/fonte.rviz'");
}

/// Botao para salvar tudo referente a acumulada
void MainWindow::on_pushButton_encerrar_clicked(){
    cw.salvar_acumulada();
}

/// Botao para reiniciar as nuvens e odometria da ZED, ou seja, tudo novo na captura
void MainWindow::on_pushButton_reiniciar_clicked(){
    cw.reiniciar();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// --------------------------------------------------- ABA2 ----------------------------------------------------------------- ///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Botao para pegar arquivo da nuvem ALVO
void MainWindow::on_pushButton_nuvemalvo_clicked(){
    QString nome_alvo;
    nome_alvo = QFileDialog::getOpenFileName(this, "Nuvem Alvo", "", "PLY Files (*.ply)");
    ui.lineEdit_nuvemalvo->setText(nome_alvo);

    rn.set_nuvem_alvo(nome_alvo);
}

/// Botao para arquivo NVM de cameras da nuvem ALVO
void MainWindow::on_pushButton_camerasalvo_clicked(){
    QString cameras_alvo = QFileDialog::getOpenFileName(this, "Cameras da Nuvem Alvo", "", "NVM Files (*.nvm)");
    ui.lineEdit_camerasalvo->setText(cameras_alvo);

    rn.set_arquivo_cameras_alvo(cameras_alvo);
}

/// Botao para pegar arquivo da nuvem FONTE
void MainWindow::on_pushButton_nuvemfonte_clicked(){
    QString nome_fonte;
    nome_fonte = QFileDialog::getOpenFileName(this, "Nuvem Fonte", "", "PLY Files (*.ply)");
    ui.lineEdit_nuvemfonte->setText(nome_fonte);

    rn.set_nuvem_fonte(nome_fonte);
}

/// Botao para arquivo NVM de cameras da nuvem ALVO
void MainWindow::on_pushButton_camerasfonte_clicked(){
    QString cameras_fonte = QFileDialog::getOpenFileName(this, "Cameras da Nuvem Fonte", "", "NVM Files (*.nvm)");
    ui.lineEdit_camerasfonte->setText(cameras_fonte);

    rn.set_arquivo_cameras_fonte(cameras_fonte);
}

/// Botao para iniciar os visualizadores para os topicos de nuvens alvo, fonte modificada e acumulada
void MainWindow::on_pushButton_iniciararquivos_clicked(){
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/handsets_ws/src/Cameras_GRIn/handset_gui/resources/tgt_src.rviz'");
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/handsets_ws/src/Cameras_GRIn/handset_gui/resources/acumulada_ajustada.rviz'");
}

/// Sliders sao liberados, a nuvem pode ser transformada
void MainWindow::on_horizontalSlider_x_sliderReleased(){
    double x = (double)ui.horizontalSlider_x->value();
    double y = ui.lineEdit_Y->text().toDouble();
    double z = ui.lineEdit_Z->text().toDouble();

    ui.lineEdit_X->setText(QString::number(x));

    rn.set_translacao(x, y, z);
}
void MainWindow::on_horizontalSlider_y_sliderReleased(){
    double x = ui.lineEdit_X->text().toDouble();
    double y = (double)ui.horizontalSlider_y->value();
    double z = ui.lineEdit_Z->text().toDouble();

    ui.lineEdit_Y->setText(QString::number(y));

    rn.set_translacao(x, y, z);
}
void MainWindow::on_horizontalSlider_z_sliderReleased(){
    double x = ui.lineEdit_X->text().toDouble();
    double y = ui.lineEdit_Y->text().toDouble();
    double z = (double)ui.horizontalSlider_z->value();

    ui.lineEdit_Z->setText(QString::number(z));

    rn.set_translacao(x, y, z);
}

void MainWindow::on_dial_x_sliderReleased(){
    double x = (double)ui.dial_x->value();
    double y = ui.lineEdit_rotacaoy->text().toDouble();
    double z = ui.lineEdit_rotacaoz->text().toDouble();

    ui.lineEdit_rotacaox->setText(QString::number(x));

    rn.set_rotacao(x, y, z);
}
void MainWindow::on_dial_y_sliderReleased(){
    double x = ui.lineEdit_rotacaox->text().toDouble();
    double y = (double)ui.dial_y->value();
    double z = ui.lineEdit_rotacaoz->text().toDouble();

    ui.lineEdit_rotacaoy->setText(QString::number(y));

    rn.set_rotacao(x, y, z);
}
void MainWindow::on_dial_z_sliderReleased(){
    double x = ui.lineEdit_rotacaox->text().toDouble();
    double y = ui.lineEdit_rotacaoy->text().toDouble();
    double z = (double)ui.dial_z->value();

    ui.lineEdit_rotacaoz->setText(QString::number(z));

    rn.set_rotacao(x, y, z);
}

/// Ajustes sobre os limites de translação a partir dos linedits
void MainWindow::on_lineEdit_limitex_returnPressed(){
    ui.horizontalSlider_x->setMaximum( ui.lineEdit_limitex->text().toInt());
    ui.horizontalSlider_x->setMinimum(-ui.lineEdit_limitex->text().toInt());
}
void MainWindow::on_lineEdit_limitey_returnPressed(){
    ui.horizontalSlider_y->setMaximum( ui.lineEdit_limitey->text().toInt());
    ui.horizontalSlider_y->setMinimum(-ui.lineEdit_limitey->text().toInt());
}
void MainWindow::on_lineEdit_limitez_returnPressed(){
    ui.horizontalSlider_z->setMaximum( ui.lineEdit_limitez->text().toInt());
    ui.horizontalSlider_z->setMinimum(-ui.lineEdit_limitez->text().toInt());
}

/// Refinando o valor a partir dos linedits do lado direito
void MainWindow::on_lineEdit_X_returnPressed(){
    int valor = ui.lineEdit_X->text().toInt();
    if(valor >= ui.horizontalSlider_x->minimum() && valor <= ui.horizontalSlider_x->maximum()){
        ui.horizontalSlider_x->setValue(valor);

        double x = ui.lineEdit_X->text().toDouble();
        double y = ui.lineEdit_Y->text().toDouble();
        double z = ui.lineEdit_Z->text().toDouble();

        rn.set_translacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_Y_returnPressed(){
    int valor = ui.lineEdit_Y->text().toInt();
    if(valor >= ui.horizontalSlider_y->minimum() && valor <= ui.horizontalSlider_y->maximum()){
        ui.horizontalSlider_y->setValue(valor);

        double x = ui.lineEdit_X->text().toDouble();
        double y = ui.lineEdit_Y->text().toDouble();
        double z = ui.lineEdit_Z->text().toDouble();

        rn.set_translacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_Z_returnPressed(){
    int valor = ui.lineEdit_Z->text().toInt();
    if(valor >= ui.horizontalSlider_z->minimum() && valor <= ui.horizontalSlider_z->maximum()){
        ui.horizontalSlider_z->setValue(valor);

        double x = ui.lineEdit_X->text().toDouble();
        double y = ui.lineEdit_Y->text().toDouble();
        double z = ui.lineEdit_Z->text().toDouble();

        rn.set_translacao(x, y, z);
    }
}

/// Refinando o valor a partir dos linedits de rotacao
void MainWindow::on_lineEdit_rotacaox_returnPressed(){
    int valor = ui.lineEdit_rotacaox->text().toInt();
    if(valor >= ui.dial_x->minimum() && valor <= ui.dial_x->maximum()){
        ui.dial_x->setValue(valor);

        double x = ui.lineEdit_rotacaox->text().toDouble();
        double y = ui.lineEdit_rotacaoy->text().toDouble();
        double z = ui.lineEdit_rotacaoz->text().toDouble();

        rn.set_rotacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_rotacaoy_returnPressed(){
    int valor = ui.lineEdit_rotacaoy->text().toInt();
    if(valor >= ui.dial_y->minimum() && valor <= ui.dial_y->maximum()){
        ui.dial_y->setValue(valor);

        double x = ui.lineEdit_rotacaox->text().toDouble();
        double y = ui.lineEdit_rotacaoy->text().toDouble();
        double z = ui.lineEdit_rotacaoz->text().toDouble();

        rn.set_rotacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_rotacaoz_returnPressed(){
    int valor = ui.lineEdit_rotacaoz->text().toInt();
    if(valor >= ui.dial_z->minimum() && valor <= ui.dial_z->maximum()){
        ui.dial_z->setValue(valor);

        double x = ui.lineEdit_rotacaox->text().toDouble();
        double y = ui.lineEdit_rotacaoy->text().toDouble();
        double z = ui.lineEdit_rotacaoz->text().toDouble();

        rn.set_rotacao(x, y, z);
    }
}

/// Botao para registrar as nuvens
void MainWindow::on_pushButton_registrar_clicked(){
    rn.registrar_nuvens(ui.checkBox_icp->isChecked());

    float x, y, z, rx, ry, rz;
    rn.get_TFinal(x, y, z, rx, ry, rz);

    ui.lineEdit_X->setText(QString::number(x));
    ui.lineEdit_Y->setText(QString::number(y));
    ui.lineEdit_Z->setText(QString::number(z));
    ui.lineEdit_rotacaox->setText(QString::number(rx));
    ui.lineEdit_rotacaoy->setText(QString::number(ry));
    ui.lineEdit_rotacaoz->setText(QString::number(rz));

    ui.dial_x->setValue(int(rx));
    ui.dial_y->setValue(int(ry));
    ui.dial_z->setValue(int(rz));
    ui.horizontalSlider_x->setValue(int(x));
    ui.horizontalSlider_y->setValue(int(y));
    ui.horizontalSlider_z->setValue(int(z));
}

/// Botao para salvar ler os arquivos nvm, criar os objetos de cameras e escrever o novo arquivo no lugar certo
void MainWindow::on_pushButton_salvarfinal_clicked(){
    rn.salvar_dados_finais(ui.lineEdit_pastafinal->text());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// --------------------------------------------------- ABA3 ----------------------------------------------------------------- ///
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// Botao para carregar arquivo da nuvem que sera filtrada
void MainWindow::on_pushButton_nuvemacorrigir_clicked(){
    QString nome_alvo;
    nome_alvo = QFileDialog::getOpenFileName(this, "Nuvem a Filtrar", "", "PLY Files (*.ply)");
    ui.lineEdit_nuvemparafiltrar->setText(nome_alvo);

    rn.set_nuvem_filtrar(nome_alvo);
    // Habilitar o resto da aba
    ui.groupBox_filtrosforma->setEnabled(true);
    ui.groupBox_cores->setEnabled(true);
    ui.frame_salvar->setEnabled(true);
    ui.pushButton_resetafiltro->setEnabled(true);
    ui.groupBox_polinomio->setEnabled(true);
}

/// Botao para visulizar enquanto a filtragem ocorre
void MainWindow::on_pushButton_visualizarcorrecao_clicked(){
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/handsets_ws/src/Cameras_GRIn/handset_gui/resources/filtrando.rviz'");
}

/// Botao para aplicar o filtro de voxel
void MainWindow::on_pushButton_voxel_clicked(){
    float voxel = ui.lineEdit_voxel->text().toFloat(); // aqui em centimetros, tem que mandar em metros
    ROS_INFO("Tamanho do voxel: %.4f cm", voxel);
    if(voxel != 0)
        rn.set_new_voxel(voxel/100.0);
}

/// Botao para aplicar filtros de outliers
void MainWindow::on_pushButton_outliers_clicked(){
    float mean = ui.lineEdit_outliersmean->text().toFloat();
    float dev  = ui.lineEdit_outliersstd->text().toFloat();
    ROS_INFO("Media: %.4f    Desvio: %.4f", mean, dev);
    if(mean != 0 && dev != 0)
        rn.set_new_outlier(mean, dev);
}

/// Botao para salvar a nuvem final na mesma pasta que a original
void MainWindow::on_pushButton_salvarnuvemfiltrada_clicked(){
    rn.salvar_nuvem_filtrada(ui.lineEdit_nuvemfiltradasalvar->text());
}

/// Ajuste de cores quando mudar linedits respectivos
void MainWindow::on_lineEdit_rmin_returnPressed(){
    int rmin = ui.lineEdit_rmin->text().toInt();
    int rmax = ui.lineEdit_rmax->text().toInt();
    int gmin = ui.lineEdit_gmin->text().toInt();
    int gmax = ui.lineEdit_gmax->text().toInt();
    int bmin = ui.lineEdit_bmin->text().toInt();
    int bmax = ui.lineEdit_bmax->text().toInt();

    rn.set_filter_colors(rmin, rmax, gmin, gmax, bmin, bmax);
}
void MainWindow::on_lineEdit_rmax_returnPressed(){
    int rmin = ui.lineEdit_rmin->text().toInt();
    int rmax = ui.lineEdit_rmax->text().toInt();
    int gmin = ui.lineEdit_gmin->text().toInt();
    int gmax = ui.lineEdit_gmax->text().toInt();
    int bmin = ui.lineEdit_bmin->text().toInt();
    int bmax = ui.lineEdit_bmax->text().toInt();

    rn.set_filter_colors(rmin, rmax, gmin, gmax, bmin, bmax);
}
void MainWindow::on_lineEdit_gmin_returnPressed(){
    int rmin = ui.lineEdit_rmin->text().toInt();
    int rmax = ui.lineEdit_rmax->text().toInt();
    int gmin = ui.lineEdit_gmin->text().toInt();
    int gmax = ui.lineEdit_gmax->text().toInt();
    int bmin = ui.lineEdit_bmin->text().toInt();
    int bmax = ui.lineEdit_bmax->text().toInt();

    rn.set_filter_colors(rmin, rmax, gmin, gmax, bmin, bmax);
}
void MainWindow::on_lineEdit_gmax_returnPressed(){
    int rmin = ui.lineEdit_rmin->text().toInt();
    int rmax = ui.lineEdit_rmax->text().toInt();
    int gmin = ui.lineEdit_gmin->text().toInt();
    int gmax = ui.lineEdit_gmax->text().toInt();
    int bmin = ui.lineEdit_bmin->text().toInt();
    int bmax = ui.lineEdit_bmax->text().toInt();

    rn.set_filter_colors(rmin, rmax, gmin, gmax, bmin, bmax);
}
void MainWindow::on_lineEdit_bmin_returnPressed(){
    int rmin = ui.lineEdit_rmin->text().toInt();
    int rmax = ui.lineEdit_rmax->text().toInt();
    int gmin = ui.lineEdit_gmin->text().toInt();
    int gmax = ui.lineEdit_gmax->text().toInt();
    int bmin = ui.lineEdit_bmin->text().toInt();
    int bmax = ui.lineEdit_bmax->text().toInt();

    rn.set_filter_colors(rmin, rmax, gmin, gmax, bmin, bmax);
}
void MainWindow::on_lineEdit_bmax_returnPressed(){
    int rmin = ui.lineEdit_rmin->text().toInt();
    int rmax = ui.lineEdit_rmax->text().toInt();
    int gmin = ui.lineEdit_gmin->text().toInt();
    int gmax = ui.lineEdit_gmax->text().toInt();
    int bmin = ui.lineEdit_bmin->text().toInt();
    int bmax = ui.lineEdit_bmax->text().toInt();

    rn.set_filter_colors(rmin, rmax, gmin, gmax, bmin, bmax);
}

// Resetar os filtros aplicados
void MainWindow::on_pushButton_resetafiltro_clicked(){
    rn.reseta_filtros();
}

// Aplicar suavizacao polinomial na nuvem
void MainWindow::on_pushButton_filtropolinomio_clicked(){
  rn.aplica_filtro_polinomio(ui.lineEdit_graupolinomio->text().toInt());
}

}  // namespace handset_gui

