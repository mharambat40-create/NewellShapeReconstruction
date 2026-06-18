#ifndef NEWELL_VIEW_FLOATINGWORKFLOWMENU_H
#define NEWELL_VIEW_FLOATINGWORKFLOWMENU_H

#include <QWidget>

class QPushButton;

class FloatingWorkflowMenu : public QWidget
{
    Q_OBJECT

public:
    explicit FloatingWorkflowMenu(QWidget *parent = nullptr);

signals:
    void importRequested();
    void placeholderRequested(const QString &featureName);

private:
    QPushButton *importButton_ = nullptr;
    QPushButton *preProcessingButton_ = nullptr;
    QPushButton *surfaceSmoothingButton_ = nullptr;
    QPushButton *parametricFittingButton_ = nullptr;
    QPushButton *exportButton_ = nullptr;
};

#endif // NEWELL_VIEW_FLOATINGWORKFLOWMENU_H
