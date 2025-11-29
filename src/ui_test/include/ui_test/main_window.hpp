/**
 * @file /include/ui_test/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef ui_test_MAIN_WINDOW_H
#define ui_test_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QIcon>
#include <QWidget>
#include <QFileDialog>
#include <QKeyEvent>
#include <QDebug>
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include "controlitem.hpp"
#include <QFile>
#include <QFileSystemWatcher>
#include <QTimer>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <QString>
#include <QImage>
#include <QFile>
#include <QTextStream>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

private slots:
void on_pushButton_7_clicked();

void on_pushButton_11_clicked();

void on_pushButton_18_clicked();

void on_pushButton_19_clicked();

void on_pushButton_20_clicked();

void on_pushButton_21_clicked();

void on_pushButton_23_clicked();

void on_pushButton_8_clicked();

void on_pushButton_9_clicked();

void on_pushButton_10_clicked();

void on_pushButton_22_clicked();

void on_doubleSpinBox_valueChanged(double arg1);

void on_doubleSpinBox_2_valueChanged(double arg1);

void on_pushButton_15_clicked();

void on_pushButton_16_clicked();

void on_pushButton_17_clicked();

  void on_pushButton_24_clicked();

  void on_pushButton_26_clicked();

  void on_pushButton_25_clicked();

  void on_pushButton_27_clicked();

  void on_doubleSpinBox_3_valueChanged(double arg1);

  void on_doubleSpinBox_4_valueChanged(double arg1);

  void on_doubleSpinBox_5_valueChanged(double arg1);

  void on_doubleSpinBox_6_valueChanged(double arg1);

  void on_pushButton_30_clicked();

  void on_pushButton_33_clicked();

  void on_pushButton_31_clicked();

  void on_pushButton_32_clicked();

  void on_pushButton_41_clicked();

  void on_pushButton_28_clicked();
  void on_pushButton_29_clicked();

  void on_pushButton_34_clicked();
  void on_pushButton_35_clicked();
  void on_pushButton_36_clicked();
  void on_pushButton_37_clicked();
  void on_pushButton_38_clicked();
  void on_pushButton_39_clicked();
  void on_pushButton_40_clicked();

    //plainTextEdit에서 글자 가져오기

  void on_pushButton_42_clicked();
    
  void on_pushButton_43_clicked();
  void camera_callback();
  void updateImage(const QPixmap &pixmap, int index);  // 이미지 업데이트 슬롯

private:
  Ui::MainWindowDesign* ui;
  void keyPressEvent(QKeyEvent *event) override;
  void closeEvent(QCloseEvent* event);
	QPixmap m_img[2];
  QNode *qnode;

};

#endif  // ui_test_MAIN_WINDOW_H
