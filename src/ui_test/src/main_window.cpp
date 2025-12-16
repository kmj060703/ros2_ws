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

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);

    QIcon icon("://ros-icon.png");
    this->setWindowIcon(icon);
    new_timer1 = new QTimer(this);
    new_timer1->setInterval(100);
    connect(new_timer1, &QTimer::timeout, this, &MainWindow::combine_callback);

    new_timer1->start();

    // 초기값 설정
    imshow_flag_1 = 1;
    imshow_flag_2 = 1;
    camera_1_state = 1; // index 0
    camera_2_state = 2; // index 1
    ui->comboBox_camera1->addItem("raw_camera");
    ui->comboBox_camera2->addItem("raw_camera");
    ui->comboBox_camera1->addItem("raw_bird");
    ui->comboBox_camera2->addItem("raw_bird");
    ui->comboBox_camera1->addItem("binary_bird");
    ui->comboBox_camera2->addItem("binary_bird");
    ui->comboBox_camera1->addItem("yolo_camera");
    ui->comboBox_camera2->addItem("yolo_camera");

    QImage default_img(640, 360, QImage::Format_BGR888);
    default_img.fill(QColor(128, 128, 128));
    m_img[0] = QPixmap::fromImage(default_img);
    m_img[1] = QPixmap::fromImage(default_img);
    m_img[2] = QPixmap::fromImage(default_img);
    m_img[3] = QPixmap::fromImage(default_img);

    ui->horizontalSlider->setRange(0, 255);
    ui->horizontalSlider_2->setRange(0, 255);
    ui->horizontalSlider_3->setRange(0, 255);
    ui->horizontalSlider_4->setRange(0, 255);
    ui->horizontalSlider_5->setRange(0, 255);
    ui->horizontalSlider_6->setRange(0, 255);

    ui->dial->setRange(-180, 180); // yaw
    ui->dial->setWrapping(true);
    ui->dial_localang->setRange(-180, 180); // local_yaw
    ui->dial_localang->setWrapping(true);

    // ui->dial_5->setRange(0, 1);  // x
    // ui->dial_6->setRange(-5, 5); // z

    h_high["lw"] = 0;
    h_high["ly"] = 0;
    h_high["lr"] = 0;
    h_high["tr"] = 0;
    h_high["ty"] = 0;
    h_high["tg"] = 0;
    h_high["bb"] = 0;

    s_high["lw"] = 0;
    s_high["ly"] = 0;
    s_high["lr"] = 0;
    s_high["tr"] = 0;
    s_high["ty"] = 0;
    s_high["tg"] = 0;
    s_high["bb"] = 0;

    v_high["lw"] = 0;
    v_high["ly"] = 0;
    v_high["lr"] = 0;
    v_high["tr"] = 0;
    v_high["ty"] = 0;
    v_high["tg"] = 0;
    v_high["bb"] = 0;

    h_low["lw"] = 0;
    h_low["ly"] = 0;
    h_low["lr"] = 0;
    h_low["tr"] = 0;
    h_low["ty"] = 0;
    h_low["tg"] = 0;
    h_low["bb"] = 0;

    s_low["lw"] = 0;
    s_low["ly"] = 0;
    s_low["lr"] = 0;
    s_low["tr"] = 0;
    s_low["ty"] = 0;
    s_low["tg"] = 0;
    s_low["bb"] = 0;

    v_low["lw"] = 0;
    v_low["ly"] = 0;
    v_low["lr"] = 0;
    v_low["tr"] = 0;
    v_low["ty"] = 0;
    v_low["tg"] = 0;
    v_low["bb"] = 0;

    qnode = new QNode();
    connect(qnode, &QNode::imageReceived, this, &MainWindow::updateImage);
    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::combine_callback()
{ // imu
    ui->dial->setValue(imu_yaw);
    ui->dial_localang->setValue(imu_yaw_local);
    // psd
    if (psd_flag[0] == 1)
        ui->label_p_forward->setText("왼쪽 장애물!");
    else
        ui->label_p_forward->clear();
    if (psd_flag[1] == 1)
        ui->label_p_left->setText("정면 장애물!");
    else
        ui->label_p_left->clear();
    if (psd_flag[2] == 1)
        ui->label_p_right->setText("우측 장애물!");
    else
        ui->label_p_right->clear();
    // 신호등
    if (traffic_state == 1)
    {
        ui->label_f_traf->setText("빨간불! 정지");
    }
    else if (traffic_state == 2)
    {
        ui->label_f_traf->setText("파란불");
    }
    else if (traffic_state == 3)
    {
        ui->label_f_traf->setText("노란불! 감속?");
    }
    else
    {
        ui->label_f_traf->clear();
    }
    switch (driving_state)
    {
    case 1:
        ui->label_state_show->setText("교차로");
        break;
    case 4:
        ui->label_state_show->setText("공사구간");
        break;
    case 5:
        ui->label_state_show->setText("주차");
        break;
    case 6:
        ui->label_state_show->setText("차단바");
        break;
    case 0:
        ui->label_state_show->setText("주행");
        break;
    default:
        break;
    }
}

void MainWindow::on_pushButton_7_clicked()
{
    // forward
    left_right_ = 0;
    forw_back_ = 1;
    std::cout << "FORWARD" << std::endl;
}

void MainWindow::on_pushButton_11_clicked()
{
    // back
    left_right_ = 0;
    forw_back_ = -1;
    std::cout << "BACKWARD" << std::endl;
}

void MainWindow::on_pushButton_18_clicked()
{
    // left
    left_right_ = 1;
    forw_back_ = 0;
    std::cout << "LEFT" << std::endl;
}

void MainWindow::on_pushButton_19_clicked()
{
    // right
    left_right_ = -1;
    forw_back_ = 0;
    std::cout << "RIGHT" << std::endl;
}

void MainWindow::on_pushButton_20_clicked()
{
    // stop
    start_flag_ = 0;
    std::cout << "STOP" << std::endl;
}

void MainWindow::on_pushButton_21_clicked()
{
    // start
    start_flag_ = 1;
    l_start_flag_ = 0;
    std::cout << "START" << std::endl;
}

void MainWindow::on_pushButton_23_clicked()
{
    // 교차로
    state_flag_ = 1;
    std::cout << "state:INTERSECTION" << std::endl;
}

void MainWindow::on_pushButton_8_clicked()
{
    // 벽돌
    state_flag_ = 2;
    std::cout << "state:CONSTRUCTIONS" << std::endl;
}

void MainWindow::on_pushButton_9_clicked()
{
    // 주차
    state_flag_ = 3;
    std::cout << "state:PARKING" << std::endl;
}

void MainWindow::on_pushButton_10_clicked()
{
    // 차단기
    state_flag_ = 4;
    std::cout << "state:LEVEL-CROSS" << std::endl;
}

void MainWindow::on_pushButton_22_clicked()
{
    // 라트
    state_flag_ = 0;
    std::cout << "state:LANE" << std::endl;
}

void MainWindow::on_doubleSpinBox_valueChanged(double arg1)
{
    // 선속.x
    x_ = arg1;
    std::cout << "linear.x changed" << std::endl;
}

void MainWindow::on_doubleSpinBox_2_valueChanged(double arg1)
{
    // 각속.z
    z_ = arg1;
    std::cout << "angular.z changed" << std::endl;
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

    if (filePath.isEmpty())
        return;

    // 확장자가 없으면 .txt 추가
    if (!filePath.endsWith(".txt", Qt::CaseInsensitive))
    {
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

    // getOpenFileName써야 save안뜸...왜 못찾았지
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "파일 열기",
        path,
        "Text Files (*.txt)");

    if (filePath.isEmpty())
        return;

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
    // set 0
    left_right_ = 0;
    forw_back_ = 0;
    x_ = 0;
    z_ = 0;
    ui->doubleSpinBox->setValue(x_);
    ui->doubleSpinBox_2->setValue(z_);
    start_flag_ = 0;
    std::cout << "set 0" << std::endl;
}

void MainWindow::on_pushButton_24_clicked()
{
    // yolo_pause
    imshow_flag_1 = 0;
    std::cout << "Yolo PAUSE" << std::endl;
}

void MainWindow::on_pushButton_26_clicked()
{
    // yolo_play
    imshow_flag_1 = 1;
    std::cout << "Yolo PLAY" << std::endl;
}

void MainWindow::on_pushButton_25_clicked()
{
    // bird_pause
    imshow_flag_2 = 0;
    std::cout << "Bird PAUSE" << std::endl;
}

void MainWindow::on_pushButton_27_clicked()
{
    // bird_play
    imshow_flag_2 = 1;
    std::cout << "Bird PLAY" << std::endl;
}

void MainWindow::on_doubleSpinBox_3_valueChanged(double arg1)
{
    // Kp
    kp_ = arg1;
    std::cout << "kp changed" << std::endl;
}

void MainWindow::on_doubleSpinBox_4_valueChanged(double arg1)
{
    // Kd
    kd_ = arg1;
    std::cout << "kd changed" << std::endl;
}

void MainWindow::on_doubleSpinBox_5_valueChanged(double arg1)
{
    // start.x
    l_x_ = arg1;
    std::cout << "lane-linear.x changed" << std::endl;
}

void MainWindow::on_doubleSpinBox_6_valueChanged(double arg1)
{
    // start.z
    l_z_ = arg1;
    std::cout << "lane-angular.z changed" << std::endl;
}

void MainWindow::on_pushButton_30_clicked()
{
    // state_lane_pd
    l_state_flag_ = 0;
    std::cout << "line state:IANE-DETECTING" << std::endl;
}

void MainWindow::on_pushButton_33_clicked()
{
    // state_intersection_pd
    l_state_flag_ = 1;
    std::cout << "line state:INTERSECTION" << std::endl;
}

void MainWindow::on_pushButton_31_clicked()
{
    // state_construction_pd
    l_state_flag_ = 2;
    std::cout << "line state:CONSTRUCTION" << std::endl;
}

void MainWindow::on_pushButton_32_clicked()
{
    // state_parking_pd
    l_state_flag_ = 3;
    std::cout << "line state:PARKING" << std::endl;
}

void MainWindow::on_pushButton_41_clicked()
{
    // state_level_crossing_pd
    l_state_flag_ = 4;
    std::cout << "line state:LEVEL-CROSSING" << std::endl;
}

void MainWindow::on_pushButton_28_clicked()
{
    // line_save
    //  save
    QString path = QDir::homePath() + "/ros2_ws/src/ui_test/work_pd/";
    QDir().mkpath(path);

    QString filePath = QFileDialog::getSaveFileName(
        this,
        "파일 저장",
        path,
        "Text Files (*.txt)");

    if (filePath.isEmpty())
        return;

    // 확장자가 없으면 .txt 추가
    if (!filePath.endsWith(".txt", Qt::CaseInsensitive))
    {
        filePath += ".txt";
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }
    QString comment;
    comment = ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "max_vel=" << max_vel_ << "\n";
    out << "comment:" << comment << "\n";

    file.close();
    qDebug() << "saved to" << filePath;
}

// 로드가 잘 안되는 것 같은데 나중에 한번 보렴-->고쳤다!
void MainWindow::on_pushButton_29_clicked()
{
    QString path = QDir::homePath() + "/ros2_ws/src/ui_test/work_pd/";
    QDir().mkpath(path);
    QString filePath = QFileDialog::getOpenFileName(
        this,
        "파일 열기",
        path,
        "Text Files (*.txt)");

    if (filePath.isEmpty())
        return;

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
        {
            kp_ = line.split("=")[1].toDouble();
            ui->doubleSpinBox_3->setValue(kp_);
        }
        else if (line.startsWith("kd="))
        {
            kd_ = line.split("=")[1].toDouble();
            ui->doubleSpinBox_4->setValue(kd_);
        }
        else if (line.startsWith("l_state_flag="))
        {
            l_state_flag_ = line.split("=")[1].toInt();
        }
        else if (line.startsWith("l_x="))
        {
            l_x_ = line.split("=")[1].toDouble();
            ui->doubleSpinBox_5->setValue(l_x_);
        }
        else if (line.startsWith("l_z="))
        {
            l_z_ = line.split("=")[1].toDouble();
            ui->doubleSpinBox_6->setValue(l_z_);
        }
        else if (line.startsWith("max_vel="))
        {
            max_vel_ = line.split("=")[1].toDouble();
            ui->doubleSpinBox_7->setValue(max_vel_);
        }
        else if (line.startsWith("comment:"))
        {
            QString comment = line.split("comment:")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);
        }
    }

    file.close();
    qDebug() << "loaded from" << filePath;
}

void MainWindow::on_pushButton_34_clicked()
{
    // set0
    kp_ = 0;
    kd_ = 0;
    l_x_ = 0;
    l_z_ = 0;
    l_start_flag_ = 0;
    l_state_flag_ = 0;
    max_vel_ = 0;
    ui->doubleSpinBox_3->setValue(kp_);
    ui->doubleSpinBox_4->setValue(kd_);
    ui->doubleSpinBox_5->setValue(l_x_);
    ui->doubleSpinBox_6->setValue(l_z_);
    ui->doubleSpinBox_7->setValue(max_vel_);
    ui->plainTextEdit->clear();
    std::cout << "lane set 0" << std::endl;
}
void MainWindow::on_pushButton_35_clicked()
{
    // set1
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set1.txt");
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        ui->doubleSpinBox_7->setValue(max_vel_);
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
        else if (line.startsWith("max_vel="))
            max_vel_ = line.split("=")[1].toDouble();
        else if (line.startsWith("comment:"))
        {
            QString comment;
            comment = line.split(":")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);
        }
        ui->doubleSpinBox_3->setValue(kp_);
        ui->doubleSpinBox_4->setValue(kd_);
        ui->doubleSpinBox_5->setValue(x_);
        ui->doubleSpinBox_6->setValue(z_);
        ui->doubleSpinBox_7->setValue(max_vel_);

        file.close();
    }
}
void MainWindow::on_pushButton_36_clicked()
{
    // set2
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
        else if (line.startsWith("max_vel="))
            max_vel_ = line.split("=")[1].toDouble();
        else if (line.startsWith("comment:"))
        {
            QString comment;
            comment = line.split(":")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);
        }
        ui->doubleSpinBox_3->setValue(kp_);
        ui->doubleSpinBox_4->setValue(kd_);
        ui->doubleSpinBox_5->setValue(x_);
        ui->doubleSpinBox_6->setValue(z_);
        ui->doubleSpinBox_7->setValue(max_vel_);

        file.close();
    }
}
void MainWindow::on_pushButton_37_clicked()
{
    // set3
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
        ui->doubleSpinBox_7->setValue(max_vel_);
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
        else if (line.startsWith("max_vel="))
            max_vel_ = line.split("=")[1].toDouble();
        else if (line.startsWith("comment:"))
        {
            QString comment;
            comment = line.split(":")[1].trimmed();
            ui->plainTextEdit->setPlainText(comment);
        }
        ui->doubleSpinBox_3->setValue(kp_);
        ui->doubleSpinBox_4->setValue(kd_);
        ui->doubleSpinBox_5->setValue(x_);
        ui->doubleSpinBox_6->setValue(z_);
        ui->doubleSpinBox_7->setValue(max_vel_);

        file.close();
    }
}
void MainWindow::on_pushButton_38_clicked()
{
    // save1
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set1.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open fileui->doubleSpinBox_7->setValue(max_vel_);";
        return;
    }
    QString comment;
    comment = ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "max_vel" << max_vel_ << "\n";
    out << "comment:" << comment << "\n";
    file.close();
}
void MainWindow::on_pushButton_39_clicked()
{
    // save2
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set2.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }
    QString comment;
    comment = ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "max_vel" << max_vel_ << "\n";
    out << "comment:" << comment << "\n";
    file.close();
}
void MainWindow::on_pushButton_40_clicked()
{
    // save3
    QFile file("/home/yu/ros2_ws/src/ui_test/tmp/set3.txt");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }
    QString comment;
    comment = ui->plainTextEdit->toPlainText();

    QTextStream out(&file);
    out << "kp=" << kp_ << "\n";
    out << "kd=" << kd_ << "\n";
    out << "l_state_flag=" << l_state_flag_ << "\n";
    out << "l_x=" << l_x_ << "\n";
    out << "l_z=" << l_z_ << "\n";
    out << "max_vel" << max_vel_ << "\n";
    out << "comment:" << comment << "\n";
    file.close();
}

void MainWindow::on_pushButton_42_clicked()
{
    // lane_start
    l_start_flag_ = 1;
    start_flag_ = 0;
    std::cout << "lane-detecting work start" << std::endl;
}

void MainWindow::on_pushButton_43_clicked()
{
    // lane_stop
    l_start_flag_ = 0;
    std::cout << "lane-detecting work stop" << std::endl;
}

// void MainWindow::on_radioButton_3_clicked()
// {
//     // raw_bird1
//     camera_1_state = 2;
// }

// void MainWindow::on_radioButton_clicked()
// {
//     // raw1
//     camera_1_state = 1;
// }

// void MainWindow::on_radioButton_2_clicked()
// {
//     // mask_bird1
//     camera_1_state = 3;
// }

// void MainWindow::on_radioButton_6_clicked()
// {
//     // raw_bird2
//     camera_2_state = 2;
// }

// void MainWindow::on_radioButton_5_clicked()
// {
//     // raw2
//     camera_2_state = 1;
// }

// void MainWindow::on_radioButton_4_clicked()
// {
//     // mask_bird2
//     camera_2_state = 3;
// }

void MainWindow::on_doubleSpinBox_7_valueChanged(double arg1)
{
    // max_vel
    max_vel_ = arg1;
}

void MainWindow::updateImage(const QPixmap &pixmap, int index)
{
    if (pixmap.isNull())
        return;

    if (index >= 0 && index < 11)
    {
        m_img[index] = pixmap;
    }

    // 메인 카메라
    if (index >= 0 && index < 4)
    {
        if (imshow_flag_1 == 1 && camera_1_state == (index + 1))
            ui->label_18->setPixmap(m_img[index].scaled(640, 360, Qt::KeepAspectRatio));

        if (imshow_flag_2 == 1 && camera_2_state == (index + 1))
        {
            // qDebug() << "[Debug] Updating label_19 with image index:" << index;
            ui->label_19->setPixmap(
                m_img[index].scaled(640, 360, Qt::KeepAspectRatio));
        }
    }
    ui->label_wl->setPixmap(m_img[4].scaled(290, 163, Qt::KeepAspectRatio));
    ui->label_yl->setPixmap(m_img[5].scaled(290, 163, Qt::KeepAspectRatio));
    ui->label_rl->setPixmap(m_img[6].scaled(290, 163, Qt::KeepAspectRatio));
    ui->label_rt->setPixmap(m_img[7].scaled(290, 163, Qt::KeepAspectRatio));
    ui->label_yt->setPixmap(m_img[8].scaled(290, 163, Qt::KeepAspectRatio));
    ui->label_gt->setPixmap(m_img[9].scaled(290, 163, Qt::KeepAspectRatio));
    ui->label_bb->setPixmap(m_img[10].scaled(290, 163, Qt::KeepAspectRatio));
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    // hue_high
    switch (vision_hsv_state)
    {
    case 1:
        HSV_high[0] = position;
        break;
    case 2:
        HSV_high[3] = position;
        break;
    case 3:
        HSV_high[6] = position;
        break;
    case 4:
        HSV_high[9] = position;
        break;
    case 5:
        HSV_high[12] = position;
        break;
    case 6:
        HSV_high[15] = position;
        break;
    case 7:
        HSV_high[18] = position;
        break;
    default:
        break;
    }
}

void MainWindow::on_horizontalSlider_2_sliderMoved(int position)
{
    // hue_low
    switch (vision_hsv_state)
    {
    case 1:
        HSV_low[0] = position;
        break;
    case 2:
        HSV_low[3] = position;
        break;
    case 3:
        HSV_low[6] = position;
        break;
    case 4:
        HSV_low[9] = position;
        break;
    case 5:
        HSV_low[12] = position;
        break;
    case 6:
        HSV_low[15] = position;
        break;
    case 7:
        HSV_low[18] = position;
        break;
    default:
        break;
    }
}

void MainWindow::on_horizontalSlider_4_sliderMoved(int position)
{
    // S_high
    switch (vision_hsv_state)
    {
    case 1:
        HSV_high[1] = position;
        break;
    case 2:
        HSV_high[4] = position;
        break;
    case 3:
        HSV_high[7] = position;
        break;
    case 4:
        HSV_high[10] = position;
        break;
    case 5:
        HSV_high[13] = position;
        break;
    case 6:
        HSV_high[16] = position;
        break;
    case 7:
        HSV_high[19] = position;
        break;
    default:
        break;
    }
}

void MainWindow::on_horizontalSlider_3_sliderMoved(int position)
{
    // S_low
    switch (vision_hsv_state)
    {
    case 1:
        HSV_low[1] = position;
        break;
    case 2:
        HSV_low[4] = position;
        break;
    case 3:
        HSV_low[7] = position;
        break;
    case 4:
        HSV_low[10] = position;
        break;
    case 5:
        HSV_low[13] = position;
        break;
    case 6:
        HSV_low[16] = position;
        break;
    case 7:
        HSV_low[19] = position;
        break;
    default:
        break;
    }
}
void MainWindow::on_horizontalSlider_6_sliderMoved(int position)
{
    // V_high
    switch (vision_hsv_state)
    {
    case 1:
        HSV_high[2] = position;
        break;
    case 2:
        HSV_high[5] = position;
        break;
    case 3:
        HSV_high[8] = position;
        break;
    case 4:
        HSV_high[11] = position;
        break;
    case 5:
        HSV_high[14] = position;
        break;
    case 6:
        HSV_high[17] = position;
        break;
    case 7:
        HSV_high[20] = position;
        break;
    default:
        break;
    }
}

void MainWindow::on_horizontalSlider_5_sliderMoved(int position)
{
    // V_low
    switch (vision_hsv_state)
    {
    case 1:
        HSV_low[2] = position;
        break;
    case 2:
        HSV_low[5] = position;
        break;
    case 3:
        HSV_low[8] = position;
        break;
    case 4:
        HSV_low[11] = position;
        break;
    case 5:
        HSV_low[14] = position;
        break;
    case 6:
        HSV_low[17] = position;
        break;
    case 7:
        HSV_low[20] = position;
        break;
    default:
        break;
    }
}

void MainWindow::on_radioButton_7_clicked()
{
    // white_line
    vision_hsv_state = 1;
    ui->horizontalSlider->setValue(HSV_high[0]);
    ui->horizontalSlider_2->setValue(HSV_low[0]);
    ui->horizontalSlider_3->setValue(HSV_low[1]);
    ui->horizontalSlider_4->setValue(HSV_high[1]);
    ui->horizontalSlider_5->setValue(HSV_low[2]);
    ui->horizontalSlider_6->setValue(HSV_high[2]);
    qDebug() << vision_hsv_state;
}
void MainWindow::on_radioButton_8_clicked()
{
    // yellow_line
    vision_hsv_state = 2;
    ui->horizontalSlider->setValue(HSV_high[3]);
    ui->horizontalSlider_2->setValue(HSV_low[3]);
    ui->horizontalSlider_3->setValue(HSV_low[4]);
    ui->horizontalSlider_4->setValue(HSV_high[4]);
    ui->horizontalSlider_5->setValue(HSV_low[5]);
    ui->horizontalSlider_6->setValue(HSV_high[5]);
    qDebug() << vision_hsv_state;
}
void MainWindow::on_radioButton_9_clicked()
{
    // red_line
    vision_hsv_state = 3;
    ui->horizontalSlider->setValue(HSV_high[6]);
    ui->horizontalSlider_2->setValue(HSV_low[6]);
    ui->horizontalSlider_3->setValue(HSV_low[7]);
    ui->horizontalSlider_4->setValue(HSV_high[7]);
    ui->horizontalSlider_5->setValue(HSV_low[8]);
    ui->horizontalSlider_6->setValue(HSV_high[8]);
    qDebug() << vision_hsv_state;
}
void MainWindow::on_radioButton_10_clicked()
{
    // red_traffic
    vision_hsv_state = 4;
    ui->horizontalSlider->setValue(HSV_high[9]);
    ui->horizontalSlider_2->setValue(HSV_low[9]);
    ui->horizontalSlider_3->setValue(HSV_low[10]);
    ui->horizontalSlider_4->setValue(HSV_high[10]);
    ui->horizontalSlider_5->setValue(HSV_low[11]);
    ui->horizontalSlider_6->setValue(HSV_high[11]);
    qDebug() << vision_hsv_state;
}
void MainWindow::on_radioButton_12_clicked()
{
    // yellow_traffic
    vision_hsv_state = 5;
    ui->horizontalSlider->setValue(HSV_high[12]);
    ui->horizontalSlider_2->setValue(HSV_low[12]);
    ui->horizontalSlider_3->setValue(HSV_low[13]);
    ui->horizontalSlider_4->setValue(HSV_high[13]);
    ui->horizontalSlider_5->setValue(HSV_low[14]);
    ui->horizontalSlider_6->setValue(HSV_high[14]);
    qDebug() << vision_hsv_state;
}
void MainWindow::on_radioButton_11_clicked()
{
    // green_traffic
    vision_hsv_state = 6;
    ui->horizontalSlider->setValue(HSV_high[15]);
    ui->horizontalSlider_2->setValue(HSV_low[15]);
    ui->horizontalSlider_3->setValue(HSV_low[16]);
    ui->horizontalSlider_4->setValue(HSV_high[16]);
    ui->horizontalSlider_5->setValue(HSV_low[17]);
    ui->horizontalSlider_6->setValue(HSV_high[17]);
    qDebug() << vision_hsv_state;
}
void MainWindow::on_radioButton_13_clicked()
{
    // brown_bricks
    vision_hsv_state = 7;
    ui->horizontalSlider->setValue(HSV_high[18]);
    ui->horizontalSlider_2->setValue(HSV_low[18]);
    ui->horizontalSlider_3->setValue(HSV_low[19]);
    ui->horizontalSlider_4->setValue(HSV_high[19]);
    ui->horizontalSlider_5->setValue(HSV_low[20]);
    ui->horizontalSlider_6->setValue(HSV_high[20]);
    qDebug() << vision_hsv_state;
}

void MainWindow::on_pushButton_44_clicked()
{

    h_high["lw"] = HSV_high[0];
    h_high["ly"] = HSV_high[3];
    h_high["lr"] = HSV_high[6];
    h_high["tr"] = HSV_high[9];
    h_high["ty"] = HSV_high[12];
    h_high["tg"] = HSV_high[15];
    h_high["bb"] = HSV_high[18];

    s_high["lw"] = HSV_high[1];
    s_high["ly"] = HSV_high[4];
    s_high["lr"] = HSV_high[7];
    s_high["tr"] = HSV_high[10];
    s_high["ty"] = HSV_high[13];
    s_high["tg"] = HSV_high[16];
    s_high["bb"] = HSV_high[19];

    v_high["lw"] = HSV_high[2];
    v_high["ly"] = HSV_high[5];
    v_high["lr"] = HSV_high[8];
    v_high["tr"] = HSV_high[11];
    v_high["ty"] = HSV_high[14];
    v_high["tg"] = HSV_high[17];
    v_high["bb"] = HSV_high[20];

    h_low["lw"] = HSV_low[0];
    h_low["ly"] = HSV_low[3];
    h_low["lr"] = HSV_low[6];
    h_low["tr"] = HSV_low[9];
    h_low["ty"] = HSV_low[12];
    h_low["tg"] = HSV_low[15];
    h_low["bb"] = HSV_low[18];

    s_low["lw"] = HSV_low[1];
    s_low["ly"] = HSV_low[4];
    s_low["lr"] = HSV_low[7];
    s_low["tr"] = HSV_low[10];
    s_low["ty"] = HSV_low[13];
    s_low["tg"] = HSV_low[16];
    s_low["bb"] = HSV_low[19];

    v_low["lw"] = HSV_low[2];
    v_low["ly"] = HSV_low[5];
    v_low["lr"] = HSV_low[8];
    v_low["tr"] = HSV_low[11];
    v_low["ty"] = HSV_low[14];
    v_low["tg"] = HSV_low[17];
    v_low["bb"] = HSV_low[20];

    QJsonArray arr;
    arr.append(h_high);
    arr.append(s_high);
    arr.append(v_high);
    arr.append(h_low);
    arr.append(s_low);
    arr.append(v_low);

    QJsonObject obj;
    obj["visionData"] = arr;

    // to see the JSON output
    QJsonDocument doc(obj);
    qDebug() << doc.toJson(QJsonDocument::Indented);
    QString filePath = QDir::homePath() + "/ros2_ws/src/ui_test/work/visionData.json";
    saveJson(obj, filePath);
}

QJsonObject MainWindow::loadJson(const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly))
    {
        qDebug() << "파일 열기 실패:" << file.errorString();
        return QJsonObject(); // QJsonObject 반환
    }

    QByteArray data = file.readAll();
    QJsonDocument doc(QJsonDocument::fromJson(data));

    if (!doc.isObject())
    {
        qDebug() << "JSON 형식이 올바르지 않습니다.";
        return QJsonObject();
    }

    return doc.object();
}

void MainWindow::on_pushButton_45_clicked()
{
    QString filePath = QDir::homePath() + "/ros2_ws/src/ui_test/work/visionData.json";
    QJsonObject loadedObj = loadJson(filePath);

    // JSON 배열
    QJsonArray arr = loadedObj["visionData"].toArray();
    qDebug() << arr;

    QJsonObject h_high_json = arr[0].toObject();
    QJsonObject s_high_json = arr[1].toObject();
    QJsonObject v_high_json = arr[2].toObject();
    QJsonObject h_low_json = arr[3].toObject();
    QJsonObject s_low_json = arr[4].toObject();
    QJsonObject v_low_json = arr[5].toObject();

    h_high["lw"] = h_high_json["lw"].toInt();
    h_high["ly"] = h_high_json["ly"].toInt();
    h_high["lr"] = h_high_json["lr"].toInt();
    h_high["tr"] = h_high_json["tr"].toInt();
    h_high["ty"] = h_high_json["ty"].toInt();
    h_high["tg"] = h_high_json["tg"].toInt();
    h_high["bb"] = h_high_json["bb"].toInt();

    s_high["lw"] = s_high_json["lw"].toInt();
    s_high["ly"] = s_high_json["ly"].toInt();
    s_high["lr"] = s_high_json["lr"].toInt();
    s_high["tr"] = s_high_json["tr"].toInt();
    s_high["ty"] = s_high_json["ty"].toInt();
    s_high["tg"] = s_high_json["tg"].toInt();
    s_high["bb"] = s_high_json["bb"].toInt();

    v_high["lw"] = v_high_json["lw"].toInt();
    v_high["ly"] = v_high_json["ly"].toInt();
    v_high["lr"] = v_high_json["lr"].toInt();
    v_high["tr"] = v_high_json["tr"].toInt();
    v_high["ty"] = v_high_json["ty"].toInt();
    v_high["tg"] = v_high_json["tg"].toInt();
    v_high["bb"] = v_high_json["bb"].toInt();

    h_low["lw"] = h_low_json["lw"].toInt();
    h_low["ly"] = h_low_json["ly"].toInt();
    h_low["lr"] = h_low_json["lr"].toInt();
    h_low["tr"] = h_low_json["tr"].toInt();
    h_low["ty"] = h_low_json["ty"].toInt();
    h_low["tg"] = h_low_json["tg"].toInt();
    h_low["bb"] = h_low_json["bb"].toInt();

    s_low["lw"] = s_low_json["lw"].toInt();
    s_low["ly"] = s_low_json["ly"].toInt();
    s_low["lr"] = s_low_json["lr"].toInt();
    s_low["tr"] = s_low_json["tr"].toInt();
    s_low["ty"] = s_low_json["ty"].toInt();
    s_low["tg"] = s_low_json["tg"].toInt();
    s_low["bb"] = s_low_json["bb"].toInt();

    v_low["lw"] = v_low_json["lw"].toInt();
    v_low["ly"] = v_low_json["ly"].toInt();
    v_low["lr"] = v_low_json["lr"].toInt();
    v_low["tr"] = v_low_json["tr"].toInt();
    v_low["ty"] = v_low_json["ty"].toInt();
    v_low["tg"] = v_low_json["tg"].toInt();
    v_low["bb"] = v_low_json["bb"].toInt();

    HSV_high[0] = h_high["lw"].toInt();
    HSV_high[3] = h_high["ly"].toInt();
    HSV_high[6] = h_high["lr"].toInt();
    HSV_high[9] = h_high["tr"].toInt();
    HSV_high[12] = h_high["ty"].toInt();
    HSV_high[15] = h_high["tg"].toInt();
    HSV_high[18] = h_high["bb"].toInt();

    // s_high
    HSV_high[1] = s_high["lw"].toInt();
    HSV_high[4] = s_high["ly"].toInt();
    HSV_high[7] = s_high["lr"].toInt();
    HSV_high[10] = s_high["tr"].toInt();
    HSV_high[13] = s_high["ty"].toInt();
    HSV_high[16] = s_high["tg"].toInt();
    HSV_high[19] = s_high["bb"].toInt();

    // v_high
    HSV_high[2] = v_high["lw"].toInt();
    HSV_high[5] = v_high["ly"].toInt();
    HSV_high[8] = v_high["lr"].toInt();
    HSV_high[11] = v_high["tr"].toInt();
    HSV_high[14] = v_high["ty"].toInt();
    HSV_high[17] = v_high["tg"].toInt();
    HSV_high[20] = v_high["bb"].toInt();

    HSV_low[0] = h_low["lw"].toInt();
    HSV_low[3] = h_low["ly"].toInt();
    HSV_low[6] = h_low["lr"].toInt();
    HSV_low[9] = h_low["tr"].toInt();
    HSV_low[12] = h_low["ty"].toInt();
    HSV_low[15] = h_low["tg"].toInt();
    HSV_low[18] = h_low["bb"].toInt();

    HSV_low[1] = s_low["lw"].toInt();
    HSV_low[4] = s_low["ly"].toInt();
    HSV_low[7] = s_low["lr"].toInt();
    HSV_low[10] = s_low["tr"].toInt();
    HSV_low[13] = s_low["ty"].toInt();
    HSV_low[16] = s_low["tg"].toInt();
    HSV_low[19] = s_low["bb"].toInt();

    HSV_low[2] = v_low["lw"].toInt();
    HSV_low[5] = v_low["ly"].toInt();
    HSV_low[8] = v_low["lr"].toInt();
    HSV_low[11] = v_low["tr"].toInt();
    HSV_low[14] = v_low["ty"].toInt();
    HSV_low[17] = v_low["tg"].toInt();
    HSV_low[20] = v_low["bb"].toInt();

    load_vision_data(0);
    ui->radioButton_7->setChecked(true);
}

void MainWindow::load_vision_data(int index)
{
    ui->horizontalSlider->setValue(HSV_high[index]);
    ui->horizontalSlider_2->setValue(HSV_low[index]);
    ui->horizontalSlider_3->setValue(HSV_low[index + 1]);
    ui->horizontalSlider_4->setValue(HSV_high[index + 1]);
    ui->horizontalSlider_5->setValue(HSV_low[index + 2]);
    ui->horizontalSlider_6->setValue(HSV_high[index + 2]);
}

void MainWindow::saveJson(const QJsonObject &obj, const QString &filePath)
{
    QJsonDocument doc(obj);
    QFile file(filePath);

    if (!file.open(QIODevice::WriteOnly))
    {
        qDebug() << "파일 열기 실패:" << file.errorString();
        return;
    }

    file.write(doc.toJson(QJsonDocument::Indented));
    file.close();
    qDebug() << "JSON 파일 저장 완료:" << filePath;
}

void MainWindow::on_pushButton_51_clicked()
{
    // vision_set0
    for (int i = 0; i < 21; i++)
    {
        HSV_high[i] = 0;
        HSV_low[i] = 0;
    }
    vision_hsv_state = 0;
    load_vision_data(0);
    ui->radioButton_7->setChecked(true);
}

void MainWindow::on_pushButton_52_clicked()
{
    // vision_save1
    QString path= QDir::homePath()+"/ros2_ws/src/ui_test/tmp/vision_set1.txt";
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open vision_set1.txt file";
        return;
    }
    QTextStream out(&file);
    int index = (vision_hsv_state - 1) * 3;
    out << "H_h=" << HSV_high[index] << "\n";
    out << "H_l=" << HSV_low[index] << "\n";
    out << "S_h=" << HSV_high[index + 1] << "\n";
    out << "S_l=" << HSV_low[index + 1] << "\n";
    out << "V_h=" << HSV_high[index + 2] << "\n";
    out << "V_l=" << HSV_low[index + 2] << "\n";
    out << "vision_state=" << vision_hsv_state << "\n";
    file.close();
}

void MainWindow::on_pushButton_53_clicked()
{
    // vision_save2
    QString path= QDir::homePath()+"/ros2_ws/src/ui_test/tmp/vision_set2.txt";
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open vision_set2.txt file";
        return;
    }
    QTextStream out(&file);
    int index = (vision_hsv_state - 1) * 3;
    out << "H_h=" << HSV_high[index] << "\n";
    out << "H_l=" << HSV_low[index] << "\n";
    out << "S_h=" << HSV_high[index + 1] << "\n";
    out << "S_l=" << HSV_low[index + 1] << "\n";
    out << "V_h=" << HSV_high[index + 2] << "\n";
    out << "V_l=" << HSV_low[index + 2] << "\n";
    out << "vision_state=" << vision_hsv_state << "\n";
    file.close();
}

void MainWindow::on_pushButton_55_clicked()
{
    // vision_save3
    QString path= QDir::homePath()+"/ros2_ws/src/ui_test/tmp/vision_set3.txt";
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "cannot open vision_set3.txt file";
        return;
    }
    QTextStream out(&file);
    int index = (vision_hsv_state - 1) * 3;
    out << "H_h=" << HSV_high[index] << "\n";
    out << "H_l=" << HSV_low[index] << "\n";
    out << "S_h=" << HSV_high[index + 1] << "\n";
    out << "S_l=" << HSV_low[index + 1] << "\n";
    out << "V_h=" << HSV_high[index + 2] << "\n";
    out << "V_l=" << HSV_low[index + 2] << "\n";
    out << "vision_state=" << vision_hsv_state << "\n";
    file.close();
}

// 이쪽 로드함수 다시 손볼것...
void MainWindow::on_pushButton_50_clicked()
{
    // vision_set1
    QString path= QDir::homePath()+"/ros2_ws/src/ui_test/tmp/vision_set1.txt";
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream in(&file);
    int index = (vision_hsv_state - 1) * 3;
    std::cout << index << std::endl;
    while (!in.atEnd())
    {
        QString line = in.readLine();
        if (line.startsWith("H_h="))
            HSV_high[index] = line.split("=")[1].toInt();
        else if (line.startsWith("H_l="))
            HSV_low[index] = line.split("=")[1].toInt();
        else if (line.startsWith("S_h="))
            HSV_high[index + 1] = line.split("=")[1].toInt();
        else if (line.startsWith("S_l="))
            HSV_low[index + 1] = line.split("=")[1].toInt();
        else if (line.startsWith("V_h="))
            HSV_high[index + 2] = line.split("=")[1].toInt();
        else if (line.startsWith("V_l="))
            HSV_low[index + 2] = line.split("=")[1].toInt();
        // else if (line.startsWith("vision_state="))
        //     vision_hsv_state = line.split("=")[1].toInt();
        file.close();
    }
    file.close();
    load_vision_data(index);
}

void MainWindow::on_pushButton_54_clicked()
{
    // vision_set2
    QString path= QDir::homePath()+"/ros2_ws/src/ui_test/tmp/vision_set2.txt";
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream in(&file);
    int index = (vision_hsv_state - 1) * 3;
    std::cout << index << std::endl;
    while (!in.atEnd())
    {
        QString line = in.readLine();

        if (line.startsWith("H_h="))

            HSV_high[index] = line.split("=")[1].toInt();
        else if (line.startsWith("H_l="))
            HSV_low[index] = line.split("=")[1].toInt();
        else if (line.startsWith("S_h="))
            HSV_high[index + 1] = line.split("=")[1].toInt();
        else if (line.startsWith("S_l="))
            HSV_low[index + 1] = line.split("=")[1].toInt();
        else if (line.startsWith("V_h="))
            HSV_high[index + 2] = line.split("=")[1].toInt();
        else if (line.startsWith("V_l="))
            HSV_low[index + 2] = line.split("=")[1].toInt();
        // else if (line.startsWith("vision_state="))
        //     vision_hsv_state = line.split("=")[1].toInt();
    }
    file.close();
    load_vision_data(index);
}

void MainWindow::on_pushButton_56_clicked()
{
    // vision_set3
    QString path= QDir::homePath()+"/ros2_ws/src/ui_test/tmp/vision_set3.txt";
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "cannot open file";
        return;
    }

    QTextStream in(&file);
    int index = (vision_hsv_state - 1) * 3;
    std::cout << index << std::endl;
    while (!in.atEnd())
    {
        QString line = in.readLine();

        if (line.startsWith("H_h="))
            HSV_high[index] = line.split("=")[1].toInt();
        else if (line.startsWith("H_l="))
            HSV_low[index] = line.split("=")[1].toInt();
        else if (line.startsWith("S_h="))
            HSV_high[index + 1] = line.split("=")[1].toInt();
        else if (line.startsWith("S_l="))
            HSV_low[index + 1] = line.split("=")[1].toInt();
        else if (line.startsWith("V_h="))
            HSV_high[index + 2] = line.split("=")[1].toInt();
        else if (line.startsWith("V_l="))
            HSV_low[index + 2] = line.split("=")[1].toInt();
        // else if (line.startsWith("vision_state="))
        //     vision_hsv_state = line.split("=")[1].toInt();
    }
    file.close();
    load_vision_data(index);
}

void MainWindow::on_pushButton_p90_clicked()
{
    //+90
    imu_yaw += 90;
}

void MainWindow::on_pushButton_m90_clicked()
{
    //-90
    imu_yaw -= 90;
}

void MainWindow::on_pushButton_p180_clicked()
{
    // 180
    imu_yaw += 180;
}

void MainWindow::on_pushButton_setYaw_clicked()
{
    // setyaw
    imu_yaw = 0;
}

void MainWindow::on_pushButton_109_clicked()
{
    // intersection(combine)
    driving_state = 1;
}

void MainWindow::on_pushButton_110_clicked()
{
    // construction(combine)
    driving_state = 4;
}

void MainWindow::on_pushButton_111_clicked()
{
    // parking(combine)
    driving_state = 5;
}

void MainWindow::on_pushButton_112_clicked()
{
    // level_cross(combine)
    driving_state = 6;
}

void MainWindow::on_pushButton_113_clicked()
{
    // default(combine)
    driving_state = 0;
}

void MainWindow::on_pushButton_drive_clicked()
{
    // drive
    l_start_flag_ = 1;
}

void MainWindow::on_pushButton_stop_clicked()
{
    // driving stop
    l_start_flag_ = 0;
}

void MainWindow::on_pushButton_p90_2_clicked()
{
    //+90local
    imu_yaw_local += 90;
}

void MainWindow::on_pushButton_m90_2_clicked()
{
    //-90local
    imu_yaw_local -= 90;
}

void MainWindow::on_pushButton_p180_2_clicked()
{
    //+180local
    imu_yaw_local += 180;
}

void MainWindow::on_pushButton_setYaw_2_clicked()
{
    // set yaw local
    imu_yaw_local = 0;
}

void MainWindow::on_dial_localang_valueChanged(int value)
{
    // value표시?
    ui->label_l_ang->setText(std::to_string(imu_yaw_local).c_str());
}

void MainWindow::on_dial_valueChanged(int value)
{
    // value 표시
    ui->label_g_ang->setText(std::to_string(imu_yaw).c_str());
}

void MainWindow::on_comboBox_camera1_currentIndexChanged(int index)
{
    // camera_choose1
    camera_1_state = index;
}

void MainWindow::on_comboBox_camera2_currentIndexChanged(int index)
{
    // camera_choose2
    camera_2_state = index;
}

// 밑에 키보드 구현-->w s a d home탭에서 이동, space: 정지 enter: pd start
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_W)
    {
        left_right_ = 0;
        forw_back_ = 1;
        start_flag_ = 1;
        ui->tabWidget->setCurrentIndex(0);
        std::cout
            << "w key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_S)
    {
        left_right_ = 0;
        forw_back_ = -1;
        start_flag_ = 1;
        ui->tabWidget->setCurrentIndex(0);
        std::cout
            << "s key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_A)
    {
        left_right_ = 1;
        forw_back_ = 0;
        start_flag_ = 1;
        ui->tabWidget->setCurrentIndex(0);
        std::cout
            << "a key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_D)
    {
        left_right_ = -1;
        forw_back_ = 0;
        start_flag_ = 1;
        ui->tabWidget->setCurrentIndex(0);
        std::cout
            << "d key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_Up)
    {
        currentIndex = ui->tabWidget->currentIndex();
        if (currentIndex == 0)
        {
            x_ += 0.01;
            ui->doubleSpinBox->setValue(x_);
        }
        else if (currentIndex == 1)
        {
            l_x_ += 0.01;
            ui->doubleSpinBox_5->setValue(l_x_);
        }

        std::cout << "up key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_Down)
    {
        currentIndex = ui->tabWidget->currentIndex();
        if (currentIndex == 0)
        {
            x_ -= 0.01;
            ui->doubleSpinBox->setValue(x_);
        }
        else if (currentIndex == 1)
        {
            l_x_ -= 0.01;
            ui->doubleSpinBox_5->setValue(l_x_);
        }
        std::cout << "down key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_Left)
    {
        currentIndex = ui->tabWidget->currentIndex();
        if (currentIndex == 0)
        {
            z_ += 0.01;
            ui->doubleSpinBox_2->setValue(z_);
        }
        else if (currentIndex == 1)
        {
            l_z_ += 0.01;
            ui->doubleSpinBox_6->setValue(l_z_);
        }
        std::cout << "left key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_Right)
    {
        currentIndex = ui->tabWidget->currentIndex();
        if (currentIndex == 0)
        {
            z_ -= 0.01;
            ui->doubleSpinBox_2->setValue(z_);
        }
        else if (currentIndex == 1)
        {
            l_z_ -= 0.01;
            ui->doubleSpinBox_6->setValue(l_z_);
        }
        std::cout << "right key pressed" << std::endl;
    }
    else if (event->key() == Qt::Key_Space)
    {
        start_flag_ = 0;
        l_start_flag_ = 0;
        std::cout << "space key pressed" << std::endl;
    }

    else if (event->key() == Qt::Key_Enter)
    {
        l_start_flag_ = 1;
        ui->tabWidget->setCurrentIndex(1);
        std::cout
            << "enter key pressed" << std::endl;
    }
}