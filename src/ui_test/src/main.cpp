#include <QApplication>
#include <iostream>
#include <QDebug>
#include "../include/ui_test/main_window.hpp"

void myMessageHandler(QtMsgType, const QMessageLogContext &, const QString &) {
    // 아무 것도 하지 않음 → 로그 완전 차단
}

int main(int argc, char *argv[])
{
  qInstallMessageHandler(myMessageHandler);
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
