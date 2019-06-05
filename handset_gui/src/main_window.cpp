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
    if(controle_gravacao){ // Estamos gravando, clica para parar, fica verde
        system("rosnode kill /rosbag_record");
        ui.pushButton_gravardados->setText("Gravar Dados");
        ui.pushButton_gravardados->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)");
        controle_gravacao = false;
    } else { // Nao estamos gravando, clica para comecar, fica vermelho
        system("gnome-terminal -x sh -c 'roslaunch handset_gui record.launch'");
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
    rn.set_inicio_processo(true);

    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/handsets_ws/src/Cameras_GRIn/handset_gui/resources/tgt_src.rviz'");
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/handsets_ws/src/Cameras_GRIn/handset_gui/resources/acumulada_ajustada.rviz'");
}

/// Sliders sao liberados, a nuvem pode ser transformada
void MainWindow::on_horizontalSlider_x_sliderReleased(){
    double x = (double)ui.horizontalSlider_x->value();
    double y = (double)ui.horizontalSlider_y->value();
    double z = (double)ui.horizontalSlider_z->value();

    ui.lineEdit_X->setText(QString::number(x));

    rn.set_translacao(x, y, z);
}
void MainWindow::on_horizontalSlider_y_sliderReleased(){
    double x = (double)ui.horizontalSlider_x->value();
    double y = (double)ui.horizontalSlider_y->value();
    double z = (double)ui.horizontalSlider_z->value();

    ui.lineEdit_Y->setText(QString::number(y));

    rn.set_translacao(x, y, z);
}
void MainWindow::on_horizontalSlider_z_sliderReleased(){
    double x = (double)ui.horizontalSlider_x->value();
    double y = (double)ui.horizontalSlider_y->value();
    double z = (double)ui.horizontalSlider_z->value();

    ui.lineEdit_Z->setText(QString::number(z));

    rn.set_translacao(x, y, z);
}

void MainWindow::on_dial_x_sliderReleased(){
    double x = (double)ui.dial_x->value();
    double y = (double)ui.dial_y->value();
    double z = (double)ui.dial_z->value();

    ui.lineEdit_rotacaox->setText(QString::number(x));

    rn.set_rotacao(x, y, z);
}
void MainWindow::on_dial_y_sliderReleased(){
    double x = (double)ui.dial_x->value();
    double y = (double)ui.dial_y->value();
    double z = (double)ui.dial_z->value();

    ui.lineEdit_rotacaoy->setText(QString::number(y));

    rn.set_rotacao(x, y, z);
}
void MainWindow::on_dial_z_sliderReleased(){
    double x = (double)ui.dial_x->value();
    double y = (double)ui.dial_y->value();
    double z = (double)ui.dial_z->value();

    ui.lineEdit_rotacaoz->setText(QString::number(z));

    rn.set_rotacao(x, y, z);
}

/// Ajustes sobre os limites de translação a partir dos linedits
void MainWindow::on_lineEdit_limitex_textEdited(QString s){
    ui.horizontalSlider_x->setMaximum( s.toInt());
    ui.horizontalSlider_x->setMinimum(-s.toInt());
}
void MainWindow::on_lineEdit_limitey_textEdited(QString s){
    ui.horizontalSlider_y->setMaximum( s.toInt());
    ui.horizontalSlider_y->setMinimum(-s.toInt());
}
void MainWindow::on_lineEdit_limitez_textEdited(QString s){
    ui.horizontalSlider_z->setMaximum( s.toInt());
    ui.horizontalSlider_z->setMinimum(-s.toInt());
}

/// Refinando o valor a partir dos linedits do lado direito
void MainWindow::on_lineEdit_X_textEdited(QString s){
    int valor = s.toInt();
    if(valor >= ui.horizontalSlider_x->minimum() && valor <= ui.horizontalSlider_x->maximum()){
        ui.horizontalSlider_x->setValue(valor);

        double x = (double)ui.horizontalSlider_x->value();
        double y = (double)ui.horizontalSlider_y->value();
        double z = (double)ui.horizontalSlider_z->value();

        rn.set_translacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_Y_textEdited(QString s){
    int valor = s.toInt();
    if(valor >= ui.horizontalSlider_y->minimum() && valor <= ui.horizontalSlider_y->maximum()){
        ui.horizontalSlider_y->setValue(valor);

        double x = (double)ui.horizontalSlider_x->value();
        double y = (double)ui.horizontalSlider_y->value();
        double z = (double)ui.horizontalSlider_z->value();

        rn.set_translacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_Z_textEdited(QString s){
    int valor = s.toInt();
    if(valor >= ui.horizontalSlider_z->minimum() && valor <= ui.horizontalSlider_z->maximum()){
        ui.horizontalSlider_z->setValue(valor);

        double x = (double)ui.horizontalSlider_x->value();
        double y = (double)ui.horizontalSlider_y->value();
        double z = (double)ui.horizontalSlider_z->value();

        rn.set_translacao(x, y, z);
    }
}

/// Refinando o valor a partir dos linedits de rotacao
void MainWindow::on_lineEdit_rotacaox_textEdited(QString s){
    int valor = s.toInt();
    if(valor >= ui.dial_x->minimum() && valor <= ui.dial_x->maximum()){
        ui.dial_x->setValue(valor);

        double x = (double)ui.dial_x->value();
        double y = (double)ui.dial_y->value();
        double z = (double)ui.dial_z->value();

        rn.set_rotacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_rotacaoy_textEdited(QString s){
    int valor = s.toInt();
    if(valor >= ui.dial_y->minimum() && valor <= ui.dial_y->maximum()){
        ui.dial_y->setValue(valor);

        double x = (double)ui.dial_x->value();
        double y = (double)ui.dial_y->value();
        double z = (double)ui.dial_z->value();

        rn.set_rotacao(x, y, z);
    }
}
void MainWindow::on_lineEdit_rotacaoz_textEdited(QString s){
    int valor = s.toInt();
    if(valor >= ui.dial_z->minimum() && valor <= ui.dial_z->maximum()){
        ui.dial_z->setValue(valor);

        double x = (double)ui.dial_x->value();
        double y = (double)ui.dial_y->value();
        double z = (double)ui.dial_z->value();

        rn.set_rotacao(x, y, z);
    }
}

/// Botao para registrar as nuvens
void MainWindow::on_pushButton_registrar_clicked(){
    rn.registrar_nuvens(ui.checkBox_icp->isChecked());
}

/// Botao para salvar ler os arquivos nvm, criar os objetos de cameras e escrever o novo arquivo no lugar certo
void MainWindow::on_pushButton_salvarfinal_clicked(){
    rn.salvar_dados_finais(ui.lineEdit_pastafinal->text());
}

}  // namespace handset_gui

