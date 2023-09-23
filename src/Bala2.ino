#define M5STACK_MPU6886

#include <M5Stack.h>
#include "freertos/FreeRTOS.h"
#include "imu_filter.h"
#include "MadgwickAHRS.h"
#include "bala.h"
#include "pid.h"
#include "calibration.h"
#include "screen.h"

// extern uint8_t bala_img[41056];
static void PIDTask(void *arg);

static float angle_point = 0.0;

float kp = 24.0f, ki = 0.0f, kd = 90.0f;
float s_kp = 15.0f, s_ki = 0.075f, s_kd = 0.0f;

bool calibration_mode = false;

Bala bala;
Screen screen;

PID pid(angle_point, kp, ki, kd);
PID speed_pid(0, s_kp, s_ki, s_kd);

// the setup routine runs once when M5Stack starts up
void setup()
{
  // Initialize the M5Stack object

  M5.begin(true, false, false, false);

  Serial.begin(115200);
  M5.IMU.Init();

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextDatum(MR_DATUM);

  int16_t x_offset, y_offset, z_offset;
  float angle_center;
  calibrationInit();

  if (M5.BtnB.isPressed())
  {
    calibrationGryo();
    calibration_mode = true;
  }

  if (M5.BtnC.isPressed())
  {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.println("Charge mode");
    while (1)
    {
      if (M5.Power.isCharging())
      {
        M5.Lcd.println("Start charging...");
        while (1)
        {
          if (M5.Power.isChargeFull())
            M5.Lcd.println("Charge completed!");
          delay(5000);
        }
      }
      delay(500);
    }
  }

  calibrationGet(&x_offset, &y_offset, &z_offset, &angle_center);
  Serial.printf("x: %d, y: %d, z:%d, angle: %.2f", x_offset, y_offset, z_offset, angle_center);

  angle_point = angle_center;
  pid.SetPoint(angle_point);

  SemaphoreHandle_t i2c_mutex;
  i2c_mutex = xSemaphoreCreateMutex();
  bala.SetMutex(&i2c_mutex);

  ImuTaskStart(x_offset, y_offset, z_offset, &i2c_mutex);
  xTaskCreatePinnedToCore(PIDTask, "pid_task", 4 * 1024, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(ScreenTask, "screen_task", 1024, NULL, 4, NULL, 1);

  // M5.Lcd.drawJpg(bala_img, 41056);
  M5.Lcd.drawRect(160, 4, 122, 103, TFT_GREEN);
  if (calibration_mode)
  {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("calibration mode");
  }
}

// the loop routine runs over and over again forever
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(5));

  M5.update();
  if (M5.BtnA.wasPressed())
  {
    angle_point += 0.25;
    pid.SetPoint(angle_point);
    screen.draw_setpoint(angle_point);
  }

  if (M5.BtnB.wasPressed())
  {
    if (calibration_mode)
    {
      calibrationSaveCenterAngle(angle_point);
    }
  }

  if (M5.BtnC.wasPressed())
  {
    angle_point -= 0.25;
    pid.SetPoint(angle_point);
    screen.draw_setpoint(angle_point);
  }
}

static void PIDTask(void *arg)
{
  float bala_angle;
  float motor_speed = 0;

  int16_t pwm_speed;
  int16_t pwm_output;
  int16_t pwm_angle;

  int32_t encoder = 0;
  int32_t last_encoder = 0;
  uint32_t last_ticks = 0;

  pid.SetOutputLimits(1023, -1023);
  pid.SetDirection(-1);

  speed_pid.SetIntegralLimits(40, -40);
  speed_pid.SetOutputLimits(1023, -1023);
  speed_pid.SetDirection(1);

  for (;;)
  {
    vTaskDelayUntil(&last_ticks, pdMS_TO_TICKS(5));

    // in imu task update, update freq is 200HZ
    bala_angle = getAngle();

    // Get motor encoder value
    bala.UpdateEncoder();
    encoder = bala.wheel_left_encoder + bala.wheel_right_encoder;
    // motor_speed filter
    motor_speed = 0.8 * motor_speed + 0.2 * (encoder - last_encoder);
    last_encoder = encoder;

    if (fabs(bala_angle) < 70)
    {
      pwm_angle = (int16_t)pid.Update(bala_angle);
      pwm_speed = (int16_t)speed_pid.Update(motor_speed);
      pwm_output = pwm_speed + pwm_angle;
      if (pwm_output > 1023)
      {
        pwm_output = 1023;
      }
      if (pwm_output < -1023)
      {
        pwm_output = -1023;
      }
      bala.SetSpeed(pwm_output, pwm_output);
    }
    else
    {
      pwm_angle = 0;
      bala.SetSpeed(0, 0);
      bala.SetEncoder(0, 0);
      speed_pid.SetIntegral(0);
    }
  }
}

static void ScreenTask(void *arg)
{
  TickType_t last_wake_time;
  last_wake_time = xTaskGetTickCount();

  for (;;)
  {
    xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
    screen.draw_waveform(getAngle());
  }
}
