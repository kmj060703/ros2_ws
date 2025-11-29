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
  connect(qnode, &QNode::imageReceived, this, &MainWindow::updateImage);
  
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


void MainWindow::on_pushButton_7_clicked()
{
    //forward
    left_right_=0;
    forw_back_=1;
    std::cout<<"FORWARD"<<std::endl;
}

void MainWindow::on_pushButton_11_clicked()
{
    //back
    left_right_=0;
    forw_back_=-1;
    std::cout<<"BACKWARD"<<std::endl;
}

void MainWindow::on_pushButton_18_clicked()
{
    //left
    left_right_=1;
    forw_back_=0;
    std::cout<<"LEFT"<<std::endl;
}

void MainWindow::on_pushButton_19_clicked()
{
    //right
    left_right_=-1;
    forw_back_=0;
    std::cout<<"RIGHT"<<std::endl;
}

void MainWindow::on_pushButton_20_clicked()
{
    //stop
    start_flag_=0;
    std::cout<<"STOP"<<std::endl;
}

void MainWindow::on_pushButton_21_clicked()
{
    //start
    start_flag_=1;
    std::cout<<"START"<<std::endl;
}

void MainWindow::on_pushButton_23_clicked()
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

void MainWindow::on_pushButton_22_clicked()
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

void MainWindow::on_pushButton_15_clicked()
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

    // 확장자가 없으면 .txt 추가
    if (!filePath.endsWith(".txt", Qt::CaseInsensitive)) {
        filePath += ".txt";
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream out(&file);
    out << "left_right=" << left_right_ << "\n";
    out << "forw_back=" << forw_back_ << "\n";
    out << "state_flag=" << state_flag_ << "\n";
    out << "x=" << x_ << "\n";
    out << "z=" << z_ << "\n";

    file.close();
    qDebug() << "saved to" << filePath;
}

void MainWindow::on_pushButton_16_clicked()
{
    QString path = QDir::homePath() + "/ros2_ws/src/ui_test/work/";
    QDir().mkpath(path); 

    //getOpenFileName써야 save안뜸...왜 못찾았지 
    QString filePath = QFileDialog::getOpenFileName(
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

void MainWindow::on_pushButton_17_clicked()
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

void MainWindow::on_pushButton_24_clicked()
{
    //yolo_pause
    imshow_flag_1=0;
    std::cout<<"Yolo PAUSE"<<std::endl;
}

void MainWindow::on_pushButton_26_clicked()
{
    //yolo_play
    imshow_flag_1=1;
    std::cout<<"Yolo PLAY"<<std::endl;
}

void MainWindow::on_pushButton_25_clicked()
{
    //bird_pause
    imshow_flag_2=0;
    std::cout<<"Bird PAUSE"<<std::endl;
}

void MainWindow::on_pushButton_27_clicked()
{
    //bird_play
    imshow_flag_2=1;
    std::cout<<"Bird PLAY"<<std::endl;
}

void MainWindow::on_doubleSpinBox_3_valueChanged(double arg1)
{
    //Kp
    kp_=arg1;
    std::cout<<"kp changed"<<std::endl;
}

void MainWindow::on_doubleSpinBox_4_valueChanged(double arg1)
{
    //Kd
    kd_=arg1;
    std::cout<<"kd changed"<<std::endl;
}

void MainWindow::on_doubleSpinBox_5_valueChanged(double arg1)
{
    //start.x
    l_x_=arg1;
    std::cout<<"lane-linear.x changed"<<std::endl;
}

void MainWindow::on_doubleSpinBox_6_valueChanged(double arg1)
{
    //start.z
    l_z_=arg1;
    std::cout<<"lane-angular.z changed"<<std::endl;
}

void MainWindow::on_pushButton_30_clicked()
{
    //state_lane_pd
    l_state_flag_=0;
    std::cout<<"line state:IANE-DETECTING"<<std::endl;
}

void MainWindow::on_pushButton_33_clicked()
{
    //state_intersection_pd
    l_state_flag_=1;
    std::cout<<"line state:INTERSECTION"<<std::endl;
}

void MainWindow::on_pushButton_31_clicked()
{
    //state_construction_pd
    l_state_flag_=2;
    std::cout<<"line state:CONSTRUCTION"<<std::endl;
}

void MainWindow::on_pushButton_32_clicked()
{
    //state_parking_pd
    l_state_flag_=3;
    std::cout<<"line state:PARKING"<<std::endl;
}

void MainWindow::on_pushButton_41_clicked()
{
    //state_level_crossing_pd
    l_state_flag_=4;
    std::cout<<"line state:LEVEL-CROSSING"<<std::endl;
}

void MainWindow::on_pushButton_28_clicked()
{
    //line_save
    // save
    QString path = QDir::homePath() + "/ros2_ws/src/ui_test/work_pd/";
    QDir().mkpath(path); 

    QString filePath = QFileDialog::getSaveFileName(
        this,
        "파일 저장",
        path,
        "Text Files (*.txt)");

    if (filePath.isEmpty()) return;

    // 확장자가 없으면 .txt 추가
    if (!filePath.endsWith(".txt", Qt::CaseInsensitive)) {
        filePath += ".txt";
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }
    QString comment;
    comment=ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "comment:" << comment << "\n";

    file.close();
    qDebug() << "saved to" << filePath;
}

void MainWindow::on_pushButton_29_clicked()
{
    //line_load
    QString path = QDir::homePath() + "/ros2_ws/src/ui_test/work_pd/";
    QDir().mkpath(path); 

    //getOpenFileName써야 save안뜸...왜 못찾았지 
    QString filePath = QFileDialog::getOpenFileName(
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
        
        if (line.startsWith("kp="))
            kp_ = line.split("=")[1].toInt();
        else if (line.startsWith("kd="))
            kd_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_state_flag="))
            l_state_flag_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_x="))
            l_x_ = line.split("=")[1].toDouble();
        else if (line.startsWith("l_z="))
            l_z_ = line.split("=")[1].toDouble();
        else if (line.startsWith("comment:"))
            {QString comment;
             comment = line.split(":")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);}
    
    ui->doubleSpinBox_3->setValue(kp_);
    ui->doubleSpinBox_4->setValue(kd_);
    ui->doubleSpinBox_5->setValue(x_);
    ui->doubleSpinBox_6->setValue(z_);

    file.close();
    qDebug() << "loaded from" << filePath;
}
}

void MainWindow::on_pushButton_34_clicked()
{
    //set0
    kp_=0;
    kd_=0;
    l_x_=0;
    l_z_=0;
    l_start_flag_=0;
    l_state_flag_=0;
    ui->doubleSpinBox_3->setValue(kp_);
    ui->doubleSpinBox_4->setValue(kd_);
    ui->doubleSpinBox_5->setValue(l_x_);
    ui->doubleSpinBox_6->setValue(l_z_);
    ui->plainTextEdit->clear();
    std::cout<<"lane set 0"<<std::endl;
}
void MainWindow::on_pushButton_35_clicked()
{
    //set1
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set1.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd())
    {
        QString line = in.readLine();
        
        if (line.startsWith("kp="))
            kp_ = line.split("=")[1].toInt();
        else if (line.startsWith("kd="))
            kd_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_state_flag="))
            l_state_flag_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_x="))
            l_x_ = line.split("=")[1].toDouble();
        else if (line.startsWith("l_z="))
            l_z_ = line.split("=")[1].toDouble();
        else if (line.startsWith("comment:"))
            {QString comment;
             comment = line.split(":")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);}
    
    ui->doubleSpinBox_3->setValue(kp_);
    ui->doubleSpinBox_4->setValue(kd_);
    ui->doubleSpinBox_5->setValue(x_);
    ui->doubleSpinBox_6->setValue(z_);

    file.close();}
}
void MainWindow::on_pushButton_36_clicked()
{
    //set2
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set2.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd())
    {
        QString line = in.readLine();
        
        if (line.startsWith("kp="))
            kp_ = line.split("=")[1].toInt();
        else if (line.startsWith("kd="))
            kd_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_state_flag="))
            l_state_flag_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_x="))
            l_x_ = line.split("=")[1].toDouble();
        else if (line.startsWith("l_z="))
            l_z_ = line.split("=")[1].toDouble();
        else if (line.startsWith("comment:"))
            {QString comment;
             comment = line.split(":")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);}
    
    ui->doubleSpinBox_3->setValue(kp_);
    ui->doubleSpinBox_4->setValue(kd_);
    ui->doubleSpinBox_5->setValue(x_);
    ui->doubleSpinBox_6->setValue(z_);

    file.close();}
}
void MainWindow::on_pushButton_37_clicked()
{
    //set3
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set3.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream in(&file);
    while (!in.atEnd())
    {
        QString line = in.readLine();
        
        if (line.startsWith("kp="))
            kp_ = line.split("=")[1].toInt();
        else if (line.startsWith("kd="))
            kd_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_state_flag="))
            l_state_flag_ = line.split("=")[1].toInt();
        else if (line.startsWith("l_x="))
            l_x_ = line.split("=")[1].toDouble();
        else if (line.startsWith("l_z="))
            l_z_ = line.split("=")[1].toDouble();
        else if (line.startsWith("comment:"))
            {QString comment;
             comment = line.split(":")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);}
    
    ui->doubleSpinBox_3->setValue(kp_);
    ui->doubleSpinBox_4->setValue(kd_);
    ui->doubleSpinBox_5->setValue(x_);
    ui->doubleSpinBox_6->setValue(z_);

    file.close();}
}
void MainWindow::on_pushButton_38_clicked()
{
    //save1
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set1.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }
    QString comment;
    comment=ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "comment:" << comment << "\n";
    file.close();
}
void MainWindow::on_pushButton_39_clicked()
{
    //save2
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set2.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }
    QString comment;
    comment=ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "comment:" << comment << "\n";
    file.close();
}
void MainWindow::on_pushButton_40_clicked()
{
    //save3
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set3.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }
    QString comment;
    comment=ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "comment:" << comment << "\n";
    file.close();
}

void MainWindow::on_pushButton_42_clicked()
{
    //lane_start
    l_start_flag_=1;
    std::cout<<"lane-detecting work start"<<std::endl;
}


void MainWindow::on_pushButton_43_clicked()
{
    //lane_stop
    l_start_flag_=0;
    std::cout<<"lane-detecting work stop"<<std::endl;
}

void MainWindow::camera_callback(){
    
}

void MainWindow::updateImage(const QPixmap &pixmap, int index) {
  if (index >= 0 && index < 2) {
    m_img[index] = pixmap;
    
    // imshow_flag에 따라 즉시 표시
    if (index == 0 && imshow_flag_1 == 1) {
      ui->label_18->setPixmap(
        m_img[0].scaled(640, 360, Qt::KeepAspectRatio)
      );
    }
    else if (index == 1 && imshow_flag_2 == 1) {
      ui->label_19->setPixmap(
        m_img[1].scaled(640, 360, Qt::KeepAspectRatio)
      );
    }
  }
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
