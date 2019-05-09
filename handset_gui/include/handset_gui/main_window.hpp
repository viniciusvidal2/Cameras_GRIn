/**
 * @file /include/handset_gui/main_window.hpp
 *
 * @brief Qt based gui for handset_gui.
 *
 * @date November 2010
 **/
#ifndef handset_gui_MAIN_WINDOW_H
#define handset_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include "cloud_work.hpp"
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QFileDialog>
#include <QLabel>

#include <QTimer>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/videoio.hpp"

#include <iostream>

#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>

/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace cv;
using namespace std;

namespace handset_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    QMutex mutex;
    ~MainWindow();

    void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:

private Q_SLOTS:
    void on_pushButton_inicio_clicked();
    void on_pushButton_capturar_clicked();
    void on_pushButton_gravardados_clicked();
    void on_pushButton_visualizar_clicked();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    Cloud_Work cw;

    bool controle_gravacao; // Se esta gravando ou nao para mudar botao e mexer com o processo
};

}  // namespace handset_gui

#endif // handset_gui_MAIN_WINDOW_H
