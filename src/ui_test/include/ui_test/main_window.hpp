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
#include "QIcon"
#include <QWidget>
#include <QFileDialog>
#include <QKeyEvent>
#include <QDebug>
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include "controlitem.hpp"
#include <QFile>
#include <QFileSystemWatcher>

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
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  QNode* qnode;

private slots:
  void on_pushButton_clicked();

  void on_pushButton_2_clicked();

  void on_pushButton_4_clicked();

  void on_pushButton_3_clicked();

  void on_pushButton_5_clicked();

  void on_pushButton_6_clicked();

  void on_pushButton_7_clicked();

  void on_pushButton_8_clicked();

  void on_pushButton_9_clicked();

  void on_pushButton_10_clicked();

  void on_pushButton_11_clicked();

  void on_doubleSpinBox_valueChanged(double arg1);

  void on_doubleSpinBox_2_valueChanged(double arg1);

  void on_pushButton_12_clicked();
    
  void on_pushButton_13_clicked();

  void on_pushButton_14_clicked();

private:
  Ui::MainWindowDesign* ui;
  void keyPressEvent(QKeyEvent *event) override;
  void closeEvent(QCloseEvent* event);
};

#endif  // ui_test_MAIN_WINDOW_H
