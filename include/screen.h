#ifndef _SCREEN_H_
#define _SCREEN_H_

#include <M5Stack.h>
#include "Arduino.h"

constexpr int MAX_LEN = 120;
// #define X_OFFSET 100
// #define Y_OFFSET 95
constexpr int X_OFFSET = 160;
constexpr int Y_OFFSET = 55;
constexpr int X_SCALE = 3;

class Screen
{

public:
    Screen();
    void draw_waveform(const float& angle);
    void draw_setpoint(const float& angle);

private:
    void update_setpoint_line();

    bool has_angle_point_changed;
    bool angle_point_was_updated;

    float angle_sp;
    float angle_sp_prev;
    SemaphoreHandle_t angle_lock = NULL;
    SemaphoreHandle_t angle_sp_lock = NULL;
};

#endif