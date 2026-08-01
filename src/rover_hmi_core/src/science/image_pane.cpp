#include <rover_hmi_core/science/image_pane.h>
#include <rover_hmi_core/catppuccin.h>

#include <QPainter>
#include <QFileInfo>

ImagePane::ImagePane(const QString& placeholder, QWidget* parent)
    : QWidget(parent), placeholder_(placeholder)
{
    setMinimumHeight(160);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

bool ImagePane::load(const QString& path)
{
    if (!QFileInfo::exists(path)) return false;

    QPixmap pm;
    if (!pm.load(path)) return false;

    pixmap_ = pm;
    update();
    return true;
}

void ImagePane::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.fillRect(rect(), QColor(theme::BgPanel));
    p.setPen(QColor(theme::BorderDim));
    p.drawRect(rect().adjusted(0, 0, -1, -1));

    if (pixmap_.isNull()) {
        p.setPen(QColor(theme::TextDim));
        p.drawText(rect(), Qt::AlignCenter, placeholder_);
        return;
    }

    const QPixmap scaled = pixmap_.scaled(size(), Qt::KeepAspectRatio,
                                          Qt::SmoothTransformation);
    p.drawPixmap((width()  - scaled.width())  / 2,
                 (height() - scaled.height()) / 2, scaled);
}
