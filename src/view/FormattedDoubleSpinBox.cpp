#include "view/FormattedDoubleSpinBox.h"

#include <QFocusEvent>
#include <QLineEdit>

#include <algorithm>

FormattedDoubleSpinBox::FormattedDoubleSpinBox(QWidget *parent)
    : QDoubleSpinBox(parent)
{
    setDecimals(editingDecimals_);
}

void FormattedDoubleSpinBox::setDisplayDecimals(int decimals)
{
    displayDecimals_ = std::max(0, decimals);
    update();
}

void FormattedDoubleSpinBox::setEditingDecimals(int decimals)
{
    editingDecimals_ = std::max(0, decimals);
    setDecimals(editingDecimals_);
    update();
}

int FormattedDoubleSpinBox::displayDecimals() const
{
    return displayDecimals_;
}

int FormattedDoubleSpinBox::editingDecimals() const
{
    return editingDecimals_;
}

QString FormattedDoubleSpinBox::textFromValue(double value) const
{
    const bool isEditing = hasFocus() || (lineEdit() && lineEdit()->hasFocus());
    return formatValue(
        value,
        isEditing ? editingDecimals_ : displayDecimals_,
        isEditing);
}

void FormattedDoubleSpinBox::focusInEvent(QFocusEvent *event)
{
    QDoubleSpinBox::focusInEvent(event);

    if (QLineEdit *edit = lineEdit()) {
        edit->setText(textFromValue(value()));
        edit->selectAll();
    }
}

void FormattedDoubleSpinBox::focusOutEvent(QFocusEvent *event)
{
    QDoubleSpinBox::focusOutEvent(event);

    if (QLineEdit *edit = lineEdit()) {
        edit->setText(textFromValue(value()));
    }
}

QString FormattedDoubleSpinBox::formatValue(
    double value,
    int decimals,
    bool trimTrailingZeros) const
{
    QString text = locale().toString(value, 'f', decimals);

    if (!trimTrailingZeros) {
        return text;
    }

    const QString decimalPoint = locale().decimalPoint();
    while (text.endsWith(QLatin1Char('0'))) {
        text.chop(1);
    }

    if (text.endsWith(decimalPoint)) {
        text.chop(1);
    }

    return text;
}
