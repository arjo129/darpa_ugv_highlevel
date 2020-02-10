#ifndef ESTOP_BUTTON_H
#define ESTOP_BUTTON_H

#include <QPushButton>


class EStopButton : public QPushButton {
    Q_OBJECT
public:
    EStopButton(QWidget* qwidget) : QPushButton(qwidget) { }
    void setType(const bool isEStop, const uint8_t robotNum);
    ~EStopButton();

signals:
    void clicked(bool isEStop, uint8_t robotNum);

private slots:
    void reemitClicked();

private:
    bool isEStop;
    uint8_t robotNum;
};


#endif /* ESTOP_BUTTON_H */

