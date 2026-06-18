#ifndef NEWELL_VIEW_PREPROCESSINGSIDEMENU_H
#define NEWELL_VIEW_PREPROCESSINGSIDEMENU_H

#include <QWidget>

class QStackedLayout;
class QPushButton;

class PreprocessingSideMenu : public QWidget
{
    Q_OBJECT

public:
    explicit PreprocessingSideMenu(QWidget *parent = nullptr);

    void showOperationList();
    void hideMenu();

private:
    void showParameterPlaceholder(const QString &operationName);
    QPushButton *createMenuButton(const QString &label, QWidget *parent) const;

    QStackedLayout *stackedLayout_ = nullptr;
    QWidget *operationListPage_ = nullptr;
    QWidget *parameterPlaceholderPage_ = nullptr;
    QString selectedOperation_;
};

#endif // NEWELL_VIEW_PREPROCESSINGSIDEMENU_H
