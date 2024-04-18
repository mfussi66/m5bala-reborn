#include "screen.h"

Screen::Screen()
{
  angle_lock = xSemaphoreCreateMutex();
  angle_sp_lock = xSemaphoreCreateMutex();

}

void Screen::draw_setpoint(const float& a)
{
  angle_sp_prev = angle_sp;
  if(xSemaphoreTake(angle_sp_lock, 1000) == pdFALSE)
    return;
  angle_sp = a;
  xSemaphoreGive(angle_sp_lock);

  M5.Lcd.fillRect(89, 4, 63, 103, TFT_BLACK);
  M5.Lcd.setTextColor(TFT_RED);
  M5.Lcd.setCursor(95, constrain((int16_t)(angle_sp * X_SCALE), -50, 50) + Y_OFFSET - 5);
  M5.Lcd.printf("%.2f", angle_sp);

}

void Screen::update_setpoint_line()
{
  // draw setpoint
  M5.Lcd.drawFastHLine(X_OFFSET + 1, constrain((int16_t)(angle_sp_prev * X_SCALE), -50, 50) + Y_OFFSET, 120, TFT_BLACK);
  
  if(xSemaphoreTake(angle_sp_lock, 1000) == pdFALSE)
    return;
  M5.Lcd.drawFastHLine(X_OFFSET + 1, constrain((int16_t)(angle_sp * X_SCALE), -50, 50) + Y_OFFSET, 120, TFT_RED);
  xSemaphoreGive(angle_sp_lock);
}

void Screen::draw_waveform(const float& angle)
{
  static int16_t val_buf[MAX_LEN] = {0};
  static int16_t pt = MAX_LEN - 1;

  if(xSemaphoreTake(angle_lock, 1000) == pdFALSE)
    return;

  val_buf[pt] = constrain((int16_t)(angle * X_SCALE), -50, 50);
  xSemaphoreGive(angle_lock);

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
