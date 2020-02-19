#include <mission_planner/EStopButton.h>

EStopButton::~EStopButton() {}

void EStopButton::setType(const bool isEStop, const int robotNum) {
    this->isEStop = isEStop;
    this->robotNum = robotNum;
    connect(this, &QPushButton::clicked, this, &EStopButton::reemitClicked);
}

void EStopButton::reemitClicked() {
    emit clicked(this->isEStop, this->robotNum);
}