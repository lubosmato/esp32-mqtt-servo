#pragma once

#include "driver/gpio.h"
#include "driver/mcpwm.h"

constexpr int SERVO_MIN_PULSEWIDTH_US = 500;
constexpr int SERVO_MAX_PULSEWIDTH_US = 2500;
constexpr int SERVO_MAX_DEGREE = 120;

class Servo {
  int _angle = 0;

public:
  Servo(gpio_num_t gpio) {
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, gpio);

    mcpwm_config_t pwmConfig{};
    pwmConfig.frequency = 50;
    pwmConfig.cmpr_a = 0;
    pwmConfig.counter_mode = MCPWM_UP_COUNTER;
    pwmConfig.duty_mode = MCPWM_DUTY_MODE_0;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwmConfig);
  }

  void move(int angle) {
    if (angle < 0 || angle > SERVO_MAX_DEGREE) return;

    const uint32_t duty =
      (angle + SERVO_MAX_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (2 * SERVO_MAX_DEGREE) +
      SERVO_MIN_PULSEWIDTH_US;

    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
    vTaskDelay(pdMS_TO_TICKS(static_cast<uint32_t>(200 * (std::abs(_angle - angle) / 60.0f))));

    _angle = angle;
  }
};
