#include "mainwindow.h"

#include <QApplication>

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.setFixedSize(900, 600);
  w.show();
  return a.exec();
}
