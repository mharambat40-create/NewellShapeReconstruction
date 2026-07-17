#ifndef NEWELL_VIEW_FORMATTEDDOUBLESPINBOX_H
#define NEWELL_VIEW_FORMATTEDDOUBLESPINBOX_H

#include <QDoubleSpinBox>

class FormattedDoubleSpinBox : public QDoubleSpinBox
{
    Q_OBJECT

public:
    explicit FormattedDoubleSpinBox(QWidget *parent = nullptr);

    void setDisplayDecimals(int decimals);
    void setEditingDecimals(int decimals);

    [[nodiscard]] int displayDecimals() const;
    [[nodiscard]] int editingDecimals() const;

protected:
    [[nodiscard]] QString textFromValue(double value) const override;
    void focusInEvent(QFocusEvent *event) override;
    void focusOutEvent(QFocusEvent *event) override;

private:
    [[nodiscard]] QString formatValue(
        double value,
        int decimals,
        bool trimTrailingZeros) const;

    int displayDecimals_ = 3;
    int editingDecimals_ = 6;
};

#endif // NEWELL_VIEW_FORMATTEDDOUBLESPINBOX_H
