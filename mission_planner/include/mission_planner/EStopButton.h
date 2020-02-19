#ifndef ESTOP_BUTTON_H
#define ESTOP_BUTTON_H

#include <QPushButton>


class EStopButton : public QPushButton {
    Q_OBJECT
public:
    EStopButton(QWidget* qwidget) : QPushButton(qwidget) { }
    void setType(const bool isEStop, const int robotNum);
    ~EStopButton();

signals:
    void clicked(bool isEStop, int robotNum);

private slots:
    void reemitClicked();

private:
    bool isEStop;
    int robotNum;
};


#endif /* ESTOP_BUTTON_H */

