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
    : QMainWindow(parent), qnode(argc,argv), cw(argc, argv, &mutex){

    // Inicia UI
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    // Metodos para conectar QT SIGNALS e SLOTS nas funcoes das classes
    //    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    //    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    // Ajustes da UI
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.aba_1->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    controle_gravacao = false; // Nao estamos gravando ainda
    ui.pushButton_gravardados->setStyleSheet("background-color: rgb(0, 200, 50); color: rgb(0, 0, 0)");

    qnode.init();

    // Inicia a classe que trabalha as nuvens, principal do programa
    cw.set_inicio_acumulacao(false);
    cw.set_primeira_vez(true);
    cw.set_inicio_acumulacao(ui.lineEdit_tempo->text().toFloat()); // Tempo default para garantir
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
    system("gnome-terminal -x sh -c 'roslaunch handset_gui lancar_cameras.launch'");
}

/// Botao para iniciar a captura corretamente por tanto tempo
void MainWindow::on_pushButton_capturar_clicked(){
    float t = ui.lineEdit_tempo->text().toFloat();
    cw.set_tempo_aquisicao(t); // Tempo maximo de aquisicao
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

}  // namespace handset_gui

