#ifndef NEWELL_VIEW_PROCESSINGPROGRESSWIDGET_H
#define NEWELL_VIEW_PROCESSINGPROGRESSWIDGET_H

#include "model/processing/common/ProcessingProgress.h"

#include <QWidget>

class QLabel;
class QProgressBar;

class ProcessingProgressWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ProcessingProgressWidget(QWidget *parent = nullptr);

    void begin();
    void setProgress(const ProcessingProgress &progress);
    void complete();
    void clear();

private:
    void setPercentage(int percentage);

    QProgressBar *progressBar_ = nullptr;
    QLabel *percentageLabel_ = nullptr;
};

#endif // NEWELL_VIEW_PROCESSINGPROGRESSWIDGET_H
