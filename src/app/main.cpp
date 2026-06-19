#include "view/MainWindow.h"

#include <QApplication>
#include <QIcon>
#include <QSurfaceFormat>

namespace
{
constexpr auto kRuntimeIconPath = ":/icons/png/Newell_icon_transparent_1024.png";
}

int main(int argc, char *argv[])
{
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication a(argc, argv);
    a.setWindowIcon(QIcon(QString::fromUtf8(kRuntimeIconPath)));

    MainWindow w;
    w.setWindowIcon(QApplication::windowIcon());
    w.show();
    return QApplication::exec();
}
