#ifndef NEWELL_VIEW_FLOATINGWORKFLOWMENU_H
#define NEWELL_VIEW_FLOATINGWORKFLOWMENU_H

#include <QWidget>

class QPushButton;

class FloatingWorkflowMenu : public QWidget
{
    Q_OBJECT

public:
    explicit FloatingWorkflowMenu(QWidget *parent = nullptr);
    void setActiveWorkflowStep(const QString &stepName);

signals:
    void importRequested();
    void workflowStepSelected(const QString &stepName);

private:
    void updateButtonStates();
    QPushButton *buttonForStep(const QString &stepName) const;

    QPushButton *importButton_ = nullptr;
    QPushButton *preProcessingButton_ = nullptr;
    QPushButton *surfaceSmoothingButton_ = nullptr;
    QPushButton *parametricFittingButton_ = nullptr;
    QPushButton *exportButton_ = nullptr;
    QString activeStepName_;
};

#endif // NEWELL_VIEW_FLOATINGWORKFLOWMENU_H
