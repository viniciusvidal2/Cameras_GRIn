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

    // Inicia a classe que trabalha as nuvens, principal do programa
    cw.init();
    cw.set_inicio_acumulacao(false);
    cw.set_primeira_vez(true);
}

MainWindow::~MainWindow() {}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //	WriteSettings();
    QMainWindow::closeEvent(event);
}

}  // namespace handset_gui

