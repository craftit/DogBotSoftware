#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  auto logger = spdlog::stdout_logger_mt("console");
  logger->info("Starting bark");


  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}
