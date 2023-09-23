#include "screen.h"

Screen::Screen()
{
  angle_lock = xSemaphoreCreateMutex();

}

void Screen::draw_setpoint(const float& angle)
{

  if(xSemaphoreTake(angle_lock, 1000) == pdFALSE)
    return;

  if(angle == angle_point)
  {
    xSemaphoreGive(angle_lock);
    return;
  }

  angle_point_prev = angle_point;
  angle_point = angle;

  xSemaphoreGive(angle_lock);

  M5.Lcd.fillRect(89, 4, 63, 103, TFT_BLACK);
  M5.Lcd.setTextColor(TFT_RED);
  M5.Lcd.setCursor(95, constrain((int16_t)(angle * X_SCALE), -50, 50) + Y_OFFSET - 5);
  M5.Lcd.printf("%.2f", angle);

}

void Screen::update_setpoint_line()
{
  if(xSemaphoreTake(angle_lock, 1000) == pdFALSE)
    return;
  // draw setpoint
  M5.Lcd.drawFastHLine(X_OFFSET + 1, constrain((int16_t)(angle_point_prev * X_SCALE), -50, 50) + Y_OFFSET, 120, TFT_BLACK);
  M5.Lcd.drawFastHLine(X_OFFSET + 1, constrain((int16_t)(angle_point * X_SCALE), -50, 50) + Y_OFFSET, 120, TFT_RED);
  xSemaphoreGive(angle_lock);
}

void Screen::draw_waveform(const float& angle)
{
  static int16_t val_buf[MAX_LEN] = {0};
  static int16_t pt = MAX_LEN - 1;

  val_buf[pt] = constrain((int16_t)(angle * X_SCALE), -50, 50);

  if (--pt < 0)
  {
    pt = MAX_LEN - 1;
  }

  update_setpoint_line();

  // update plot
  for (int i = 1; i < (MAX_LEN); i++)
  {
    uint16_t now_pt = (pt + i) % (MAX_LEN);
    M5.Lcd.drawLine(i + X_OFFSET, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, i + 1 + X_OFFSET, val_buf[(now_pt + 2) % MAX_LEN] + Y_OFFSET, TFT_BLACK);
    if (i < MAX_LEN - 1)
    {
      M5.Lcd.drawLine(i + X_OFFSET, val_buf[now_pt] + Y_OFFSET, i + 1 + X_OFFSET, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, TFT_GREEN);
    }
  }
}
