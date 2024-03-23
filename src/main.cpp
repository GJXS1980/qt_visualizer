#include "pclviewer.h"
#include "container.h"
#include <QApplication>
#include <QMainWindow>

#include <QLabel>
#include <QSurfaceFormat>

#ifndef QT_NO_OPENGL
#include "window.h"
#endif

int main (int argc, char *argv[])
{
  QApplication app(argc, argv);
  PCLViewer w;

  w.show ();

  return app.exec ();
}
