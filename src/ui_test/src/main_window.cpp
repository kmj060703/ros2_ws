/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ui_test/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    //forward
    left_right_=0;
    forw_back_=1;
    std::cout<<"FORWARD"<<std::endl;
}

void MainWindow::on_pushButton_2_clicked()
{
    //back
    left_right_=0;
    forw_back_=-1;
    std::cout<<"BACKWARD"<<std::endl;
}

void MainWindow::on_pushButton_4_clicked()
{
    //left
    left_right_=1;
    forw_back_=0;
    std::cout<<"LEFT"<<std::endl;
}

void MainWindow::on_pushButton_3_clicked()
{
    //right
    left_right_=-1;
    forw_back_=0;
    std::cout<<"RIGHT"<<std::endl;
}

void MainWindow::on_pushButton_5_clicked()
{
    //stop
    start_flag_=0;
    std::cout<<"STOP"<<std::endl;
}

void MainWindow::on_pushButton_6_clicked()
{
    //start
    start_flag_=1;
    std::cout<<"START"<<std::endl;
}

void MainWindow::on_pushButton_7_clicked()
{
    //교차로
    state_flag_=1;
    std::cout<<"state:INTERSECTION"<<std::endl;
}

void MainWindow::on_pushButton_8_clicked()
{
    //벽돌
    state_flag_=2;
    std::cout<<"state:CONSTRUCTIONS"<<std::endl;
}

void MainWindow::on_pushButton_9_clicked()
{
    //주차
    state_flag_=3;
    std::cout<<"state:PARKING"<<std::endl;
}

void MainWindow::on_pushButton_10_clicked()
{
    //차단기
    state_flag_=4;
    std::cout<<"state:LEVEL-CROSS"<<std::endl;
}

void MainWindow::on_pushButton_11_clicked()
{
    //라트
    state_flag_=0;
    std::cout<<"state:LANE"<<std::endl;
}

void MainWindow::on_doubleSpinBox_valueChanged(double arg1)
{
    //선속.x
    x_=arg1;
    std::cout<<"linear.x changed"<<std::endl;
}

void MainWindow::on_doubleSpinBox_2_valueChanged(double arg1)
{
    //각속.z
    z_=arg1;
    std::cout<<"angular.z changed"<<std::endl;
}

void MainWindow::on_pushButton_12_clicked()
{
    // save
    QString path = QDir::homePath() + "/ros2_ws/src/ui_test/work/";
    QDir().mkpath(path); 

    QString filePath = QFileDialog::getSaveFileName(
    this,
    "파일 저장",
    path,
    "Text Files (*.txt)");

    if (filePath.isEmpty()) return;

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream out(&file);
    out << "left_right=" << left_right_ << "\n";
    out << "forw_back=" << forw_back_ << "\n";
    out << "start_flag=" << start_flag_ << "\n";
    out << "state_flag=" << state_flag_ << "\n";
    out << "x=" << x_ << "\n";
    out << "z=" << z_ << "\n";

    file.close();
    qDebug() << "saved to" << filePath;
}


void MainWindow::on_pushButton_13_clicked()
{
    QString path = QDir::homePath() + "/ros2_ws/src/ui_test/work/";
    QDir().mkpath(path); 

    QString filePath = QFileDialog::getSaveFileName(
    this,
    "파일 열기",
    path,
    "Text Files (*.txt)");

    if (filePath.isEmpty()) return;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd())
    {
        QString line = in.readLine();
        
        if (line.startsWith("left_right="))
            left_right_ = line.split("=")[1].toInt();
        else if (line.startsWith("forw_back="))
            forw_back_ = line.split("=")[1].toInt();
        else if (line.startsWith("start_flag="))
            start_flag_ = line.split("=")[1].toInt();
        else if (line.startsWith("state_flag="))
            state_flag_ = line.split("=")[1].toInt();
        else if (line.startsWith("x="))
            x_ = line.split("=")[1].toDouble();
        else if (line.startsWith("z="))
            z_ = line.split("=")[1].toDouble();
    }

    ui->doubleSpinBox->setValue(x_);
    ui->doubleSpinBox_2->setValue(z_);

    file.close();
    qDebug() << "loaded from" << filePath;
}

void MainWindow::on_pushButton_14_clicked()
{
    //set 0
    left_right_=0;
    forw_back_=0;
    x_=0;
    z_=0;
    ui->doubleSpinBox->setValue(x_);
    ui->doubleSpinBox_2->setValue(z_);
    start_flag_=0;
    std::cout<<"set 0"<<std::endl;

}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
  if(event->key() == Qt::Key_W)
  {
    left_right_=0;
    forw_back_=1;
  }
  else if(event->key() == Qt::Key_S)
  {
    left_right_=0;
    forw_back_=-1;
  }
  else if(event->key() == Qt::Key_A)
  {
    left_right_=1;
    forw_back_=0;
  }
  else if(event->key() == Qt::Key_D)
  {
    left_right_=-1;
    forw_back_=0;
  }
  else if(event->key() == Qt::Key_PageUp)
  {
    x_+=0.01;
    ui->doubleSpinBox->setValue(x_);
  }
  else if(event->key() == Qt::Key_PageDown)
  {
    x_-=0.01;
    ui->doubleSpinBox->setValue(x_);
  }
  else if(event->key() == Qt::Key_End)
  {
    z_+=0.01;
    ui->doubleSpinBox_2->setValue(z_);
  }
  else if(event->key() == Qt::Key_Home)
  {
    z_-=0.01;
    ui->doubleSpinBox_2->setValue(z_);
  }

}
