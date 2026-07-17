#ifndef NEWELL_VIEW_METHODSELECTIONDIALOG_H
#define NEWELL_VIEW_METHODSELECTIONDIALOG_H

#include <QDialog>
#include <QString>
#include <QStringList>
#include <QVector>

class QListWidget;
class QListWidgetItem;
class QPushButton;
class QTextBrowser;
class QShowEvent;

struct MethodSelectionOption
{
    QString value;
    QString description;
    QStringList advantages;
    QStringList disadvantages;
    QStringList useCases;
    bool enabled = true;
    QString unavailableReason;
};

class MethodSelectionDialog : public QDialog
{
public:
    MethodSelectionDialog(
        const QString &title,
        QVector<MethodSelectionOption> options,
        const QString &currentValue,
        QWidget *centerTarget);

    [[nodiscard]] QString selectedValue() const;

protected:
    void accept() override;
    void showEvent(QShowEvent *event) override;

private:
    void updateExplanation(const QListWidgetItem *item);
    [[nodiscard]] const MethodSelectionOption *optionForItem(const QListWidgetItem *item) const;

    QVector<MethodSelectionOption> options_;
    QString selectedValue_;
    QWidget *centerTarget_ = nullptr;
    QListWidget *optionList_ = nullptr;
    QListWidgetItem *temporarySelection_ = nullptr;
    QTextBrowser *explanationPanel_ = nullptr;
    QPushButton *okButton_ = nullptr;
};

#endif // NEWELL_VIEW_METHODSELECTIONDIALOG_H
