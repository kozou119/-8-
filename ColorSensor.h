#ifndef CONTEST_COLOR_SENSOR_H
#define CONTEST_COLOR_SENSOR_H
#include "CompetitionController.h"
bool colorBegin();
bool colorCalibrate();
bool colorRead(contest::Floor &floor);
#endif
