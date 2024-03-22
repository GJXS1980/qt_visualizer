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

  QSurfaceFormat format;
  format.setDepthBufferSize(24);
  QSurfaceFormat::setDefaultFormat(format);

  app.setApplicationName("cube");
  app.setApplicationVersion("0.1");
#ifndef QT_NO_OPENGL
  Window win;
  win.show();
#else
  QLabel note("OpenGL Support required");
  note.show();
#endif


  return app.exec ();
}
