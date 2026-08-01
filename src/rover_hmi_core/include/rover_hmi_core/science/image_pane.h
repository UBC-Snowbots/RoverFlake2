// image_pane.h — shows a PNG scaled to fit, keeping its aspect ratio.
// Used for the plots curve_gen.py writes.
#pragma once

#include <QWidget>
#include <QPixmap>
#include <QString>

class ImagePane : public QWidget {
public:
    explicit ImagePane(const QString& placeholder, QWidget* parent = nullptr);

    // Returns false if the file is missing or unreadable, leaving the
    // placeholder in view.
    bool load(const QString& path);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPixmap pixmap_;
    QString placeholder_;
};
