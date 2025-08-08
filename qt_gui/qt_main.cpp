#include <QApplication>
#include "MainWindow.h"

int main(int argc, char *argv[])
{
    // Enable high DPI scaling and high DPI pixmaps for modern displays.
    // These attributes ensure that the application scales correctly on
    // monitors with scaling factors other than 100%.  Qt will
    // automatically adjust widget sizes and fonts on high-density screens.
    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication app(argc, argv);
    MainWindow w;
    w.resize(1200, 600);
    w.show();
    return app.exec();
}