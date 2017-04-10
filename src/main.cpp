#include "pclapp.h"
#include <QApplication>
#include <QMainWindow>

int main (int argc, char *argv[])
{
  QApplication a (argc, argv);
  PCLApp w;
  w.show ();

  return a.exec ();
}
