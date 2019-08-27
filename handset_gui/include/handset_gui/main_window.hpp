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
#include "registra_nuvem.hpp"
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
    /// ABA 1 ///
    void on_pushButton_inicio_clicked();
    void on_pushButton_naocapturar_clicked();
    void on_pushButton_capturar_clicked();
    void on_pushButton_corrigirultimacaptura_clicked();
    void on_pushButton_gravardados_clicked();
    void on_pushButton_visualizar_clicked();
    void on_pushButton_encerrar_clicked();
    void on_pushButton_reiniciar_clicked();
    void on_pushButton_selecionabag_clicked();
    void on_checkBox_online_clicked();

    /// ABA 2 ///
    void on_pushButton_nuvemalvo_clicked();
    void on_pushButton_nuvemfonte_clicked();
    void on_pushButton_iniciararquivos_clicked();

    void on_horizontalSlider_x_sliderReleased();
    void on_horizontalSlider_y_sliderReleased();
    void on_horizontalSlider_z_sliderReleased();
    void on_dial_x_sliderReleased();
    void on_dial_y_sliderReleased();
    void on_dial_z_sliderReleased();

    void on_lineEdit_limitex_returnPressed();
    void on_lineEdit_limitey_returnPressed();
    void on_lineEdit_limitez_returnPressed();
    void on_lineEdit_X_returnPressed();
    void on_lineEdit_Y_returnPressed();
    void on_lineEdit_Z_returnPressed();
    void on_lineEdit_rotacaox_returnPressed();
    void on_lineEdit_rotacaoy_returnPressed();
    void on_lineEdit_rotacaoz_returnPressed();
    
    void on_pushButton_registrar_clicked();
    void on_pushButton_salvarfinal_clicked();

    /// ABA3 ///
    void on_pushButton_nuvemacorrigir_clicked();
    void on_pushButton_visualizarcorrecao_clicked();
    void on_pushButton_voxel_clicked();
    void on_pushButton_outliers_clicked();
    void on_pushButton_salvarnuvemfiltrada_clicked();
    void on_pushButton_resetafiltro_clicked();
    void on_pushButton_filtropolinomio_clicked();

    void on_lineEdit_rmin_returnPressed();
    void on_lineEdit_rmax_returnPressed();
    void on_lineEdit_bmin_returnPressed();
    void on_lineEdit_bmax_returnPressed();
    void on_lineEdit_gmin_returnPressed();
    void on_lineEdit_gmax_returnPressed();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    Cloud_Work cw;
    RegistraNuvem rn;

    bool controle_gravacao; // Se esta gravando ou nao para mudar botao e mexer com o processo
};

}  // namespace handset_gui

#endif // handset_gui_MAIN_WINDOW_H
