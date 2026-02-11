/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Stand3 - Peltier temperature controller
  *                   MAX31865 RTD sensor, H-bridge Peltier drive,
  *                   SSD1306 OLED menu, PID temperature control.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "u8g2.h"
#include "config_menu.h"
#include "max31865.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS  0x3C

/* MUX control */
#define MUX_EN_ACTIVE_HIGH  0u

/* Buttons */
#define LONG_PRESS_TIME_MS    1000u
#define DEBOUNCE_STABLE_COUNT 1u

/* PID update interval */
#define PID_UPDATE_MS  1000u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define READ_PIN(port,pin)  (HAL_GPIO_ReadPin((port),(pin)) == GPIO_PIN_SET)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* ---- Temperature from MAX31865 ---- */
static float    g_temp_celsius = 0.0f;
static uint16_t g_rtd_raw = 0;
static uint8_t  g_rtd_fault = 0;

/* ---- MAX31865 device handle ---- */
static Max31865_t g_max31865;

/* ---- Bidirectional PID Controller ---- */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_measurement; /* derivative on measurement, not error */
    float output;           /* -95..+95: negative=cool, positive=heat */
    uint8_t first;          /* first cycle flag */
} pid_ctrl_t;

static pid_ctrl_t g_pid = {
    .Kp = 6.0f,
    .Ki = 0.1f,
    .Kd = 2.0f,
    .integral = 0.0f,
    .prev_measurement = 0.0f,
    .output = 0.0f,
    .first = 1,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =======================================================================
   U8G2 I2C callbacks  (identical to Stand1/Stand2)
   ======================================================================= */
uint8_t u8x8_byte_stm32hal_hw_i2c(u8x8_t *u8x8, uint8_t msg,
                                    uint8_t arg_int, void *arg_ptr)
{
  (void)u8x8;
  static uint8_t buffer[32];
  static uint8_t buf_idx;
  uint8_t *data;

  switch (msg)
  {
    case U8X8_MSG_BYTE_SEND:
      data = (uint8_t *)arg_ptr;
      while (arg_int-- > 0) {
        if (buf_idx < sizeof(buffer)) buffer[buf_idx++] = *data;
        data++;
      }
      return 1;

    case U8X8_MSG_BYTE_START_TRANSFER:
      buf_idx = 0;
      return 1;

    case U8X8_MSG_BYTE_END_TRANSFER:
    {
      uint8_t iaddr = I2C_ADDRESS;
      (void)HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)iaddr << 1,
                                     buffer, buf_idx, 50u);
      return 1;
    }
    default:
      return 1;
  }
}

uint8_t psoc_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg,
                                uint8_t arg_int, void *arg_ptr)
{
  (void)u8x8; (void)arg_ptr;
  switch (msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT: return 1;
    case U8X8_MSG_DELAY_NANO:
    {
      volatile uint32_t n = (uint32_t)arg_int * 10u;
      while (n--) ;
      return 1;
    }
    case U8X8_MSG_DELAY_MILLI:
      HAL_Delay((uint32_t)arg_int);
      return 1;
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      return 1;
  }
}

/* =======================================================================
   Buttons  (3 buttons: UP, DOWN, OK)
   ======================================================================= */
typedef enum {
  BTN_NONE = 0,
  BTN_UP   = (1u << 0),
  BTN_DOWN = (1u << 1),
  BTN_OK   = (1u << 2),
} btn_mask_t;

static btn_mask_t buttons_read_raw(void)
{
  btn_mask_t m = BTN_NONE;
  if (READ_PIN(UP_SW_GPIO_Port,   UP_SW_Pin))   m |= BTN_UP;
  if (READ_PIN(DOWN_SW_GPIO_Port, DOWN_SW_Pin))  m |= BTN_DOWN;
  if (READ_PIN(OK_SW_GPIO_Port,   OK_SW_Pin))    m |= BTN_OK;
  return m;
}

static btn_mask_t buttons_get_pressed_edge(void)
{
  static btn_mask_t last_stable = BTN_NONE;
  static btn_mask_t last_raw    = BTN_NONE;
  static uint8_t    stable_cnt  = 0;

  btn_mask_t now_raw = buttons_read_raw();

  if (now_raw == last_raw) {
    if (stable_cnt < DEBOUNCE_STABLE_COUNT) stable_cnt++;
  } else {
    stable_cnt = 0;
    last_raw = now_raw;
  }

  btn_mask_t stable = last_stable;
  if (stable_cnt >= DEBOUNCE_STABLE_COUNT) stable = now_raw;

  btn_mask_t edge = (btn_mask_t)(stable & (btn_mask_t)~last_stable);
  last_stable = stable;
  return edge;
}

static btn_mask_t buttons_get_long_press(void)
{
  static btn_mask_t last_stable = BTN_NONE;
  static btn_mask_t last_raw    = BTN_NONE;
  static uint8_t    stable_cnt  = 0;
  static uint32_t   press_start = 0;

  btn_mask_t now_raw = buttons_read_raw();
  uint32_t now = HAL_GetTick();

  if (now_raw == last_raw) {
    if (stable_cnt < DEBOUNCE_STABLE_COUNT) stable_cnt++;
  } else {
    stable_cnt = 0;
    last_raw = now_raw;
    if (now_raw != BTN_NONE) press_start = now;
  }

  btn_mask_t stable = last_stable;
  if (stable_cnt >= DEBOUNCE_STABLE_COUNT) stable = now_raw;

  btn_mask_t lp = BTN_NONE;
  if (stable != BTN_NONE && (now - press_start) >= LONG_PRESS_TIME_MS) {
    lp = stable;
  }
  last_stable = stable;
  return lp;
}

/* =======================================================================
   Bidirectional PID Controller
   Output range: -95 .. +95
     positive → HEAT  (current flows one direction)
     negative → COOL  (current flows opposite direction)

   Uses "derivative on measurement" to avoid derivative kick
   when the setpoint changes.
   ======================================================================= */
static void pid_reset(pid_ctrl_t *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->output           = 0.0f;
    pid->first            = 1;
}

static float pid_update(pid_ctrl_t *pid, float setpoint, float measured)
{
    float error = setpoint - measured;

    /* Integral with anti-windup */
    pid->integral += error;
    float integral_max = 90.0f / pid->Ki;
    if (pid->integral >  integral_max) pid->integral =  integral_max;
    if (pid->integral < -integral_max) pid->integral = -integral_max;

    /* Derivative on MEASUREMENT (not error) — avoids kick on setpoint change.
       d/dt(error) = d/dt(setpoint - measured) → when setpoint jumps,
       the derivative spikes.  Using -d/dt(measured) instead. */
    float derivative = 0.0f;
    if (pid->first) {
        pid->first = 0;
    } else {
        derivative = -(measured - pid->prev_measurement);
    }
    pid->prev_measurement = measured;

    float out = pid->Kp * error
              + pid->Ki * pid->integral
              + pid->Kd * derivative;

    /* Light smoothing */
    out = pid->output * 0.2f + out * 0.8f;

    /* Clamp to ±95% */
    if (out >  95.0f) out =  95.0f;
    if (out < -95.0f) out = -95.0f;

    pid->output = out;
    return out;
}

/* =======================================================================
   H-Bridge Peltier control (TIM3 CH3/CH4 on PB0/PB1)
   ======================================================================= */
typedef enum { PELTIER_OFF = 0, PELTIER_HEAT, PELTIER_COOL } peltier_mode_t;
static peltier_mode_t g_peltier_mode = PELTIER_OFF;

static uint32_t duty_to_ccr(TIM_HandleTypeDef *htim, uint32_t duty_percent)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
  uint32_t ccr = (arr + 1u) * duty_percent / 100u;
  if (ccr > arr) ccr = arr;
  return ccr;
}

static void peltier_set_enable(uint8_t en)
{
  HAL_GPIO_WritePin(EN_PT_INTERN_GPIO_Port, EN_PT_INTERN_Pin,
                    en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void peltier_set_pwm(uint32_t duty_percent)
{
  uint32_t ccr = duty_to_ccr(&htim3, duty_percent);

  /* SAFE FIRST: PWM=0 */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0u);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0u);

  if (g_peltier_mode == PELTIER_HEAT) {
    peltier_set_enable(1u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0u);
  } else if (g_peltier_mode == PELTIER_COOL) {
    peltier_set_enable(1u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, ccr);
  } else {
    peltier_set_enable(0u);
  }
}

static void peltier_stop(void)
{
  g_peltier_mode = PELTIER_OFF;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0u);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0u);
  peltier_set_enable(0u);
}

/* =======================================================================
   MUX control (internal / external switching)
   ======================================================================= */
typedef enum { PELTIER_CTRL_INTERNAL = 0, PELTIER_CTRL_EXTERNAL = 1 } peltier_ctrl_src_t;
static peltier_ctrl_src_t g_peltier_ctrl_src = PELTIER_CTRL_INTERNAL;

static void mux_enable(uint8_t en)
{
#if (MUX_EN_ACTIVE_HIGH == 1u)
  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin,
                    en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(MUX_EN_GPIO_Port, MUX_EN_Pin,
                    en ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

static void mux_select(peltier_ctrl_src_t src)
{
  HAL_GPIO_WritePin(MUX_SEL_GPIO_Port, MUX_SEL_Pin,
                    (src == PELTIER_CTRL_EXTERNAL) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void peltier_set_source(peltier_ctrl_src_t src)
{
  /* disable mux first */
  mux_enable(0u);

  /* stop peltier */
  peltier_stop();
  HAL_Delay(2);

  if (src == PELTIER_CTRL_EXTERNAL) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_4);
    peltier_set_enable(0u);
  } else {
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    peltier_set_enable(0u);
  }

  mux_select(src);
  HAL_Delay(2);
  mux_enable(1u);

  g_peltier_ctrl_src = src;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* SAFE START: no output */
  peltier_set_enable(0u);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0u);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0u);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  peltier_stop();

  /* Default to internal control */
  peltier_set_source(PELTIER_CTRL_INTERNAL);

  /* ---- MAX31865 init (NimaLTD library) ---- */
  memset(&g_max31865, 0, sizeof(g_max31865));
  Max31865_init(&g_max31865, &hspi2, CS_GPIO_Port, CS_Pin, 2, 50);

  /* ---- I2C probe: check if OLED responds ---- */
  uint8_t i2c_ok = 0;
  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x3C << 1, 3, 100) == HAL_OK) {
    i2c_ok = 1;
  } else if (HAL_I2C_IsDeviceReady(&hi2c1, 0x3D << 1, 3, 100) == HAL_OK) {
    i2c_ok = 2;
  }

  /* ---- OLED init ---- */
  static u8g2_t u8g2;
  u8g2_Setup_ssd1306_i2c_128x32_univision_1(
      &u8g2, U8G2_R0,
      u8x8_byte_stm32hal_hw_i2c,
      psoc_gpio_and_delay_cb);
  HAL_Delay(200);
  u8g2_InitDisplay(&u8g2);
  HAL_Delay(200);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_SetFont(&u8g2, u8g2_font_helvB14_tr);

  /* Menu init */
  config_menu_init();

  config_peltier_state_t prev_peltier_st = CONFIG_PELTIER_STOP;
  config_ctrl_source_t   prev_ctrl_src   = CONFIG_CTRL_INTERNAL;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- 1.  Read MAX31865 temperature ---- */
    /* NimaLTD library: each readTempC call does
       bias-on -> 10ms -> 1shot -> 65ms -> read -> bias-off.
       We call it every 1 second with smoothing filter. */
    {
      static uint32_t last_read_tick = 0;
      static float filtered_temp = 0.0f;
      static uint8_t first_read = 1;
      uint32_t now_t = HAL_GetTick();

      if ((now_t - last_read_tick) >= 1000u) {
        last_read_tick = now_t;

        float t = 0.0f;
        bool ok = Max31865_readTempC(&g_max31865, &t);

        if (ok) {
          if (first_read) {
            filtered_temp = t;
            first_read = 0;
          } else {
            filtered_temp = Max31865_Filter(t, filtered_temp, 0.3f);
          }
          g_temp_celsius = filtered_temp;
          /* Use the raw value already captured by readTempC;
             no need for a second readRTD call. */
          g_rtd_raw = g_max31865.last_rtd;
          g_rtd_fault = 0;
        } else {
          g_rtd_fault = Max31865_readFault(&g_max31865);
        }
      }
    }

    /* ---- 2.  Buttons ---- */
    btn_mask_t press      = buttons_get_pressed_edge();
    btn_mask_t long_press = buttons_get_long_press();

    int8_t cur = ctrl_menu_get_current_idx(&app_menu);

    /* Long press edge detection (one-shot) */
    static uint8_t lp_handled = 0;
    uint8_t lp_edge = 0;
    if (long_press & BTN_DOWN) {
      if (!lp_handled) { lp_edge = 1; lp_handled = 1; }
    } else {
      lp_handled = 0;
    }

    /* ---- 3.  Route buttons to menu ---- */
    if (cur == MENU_TEMP_INC || cur == MENU_TEMP_DEC) {
      /* Temp edit: UP=increase, DOWN=decrease, OK=back */
      if (press & BTN_UP)   config_menu_temp_increase();
      if (press & BTN_DOWN) config_menu_temp_decrease();
      if (press & BTN_OK)   config_menu_process_cmd(CTRL_MENU_CMD_BACK);
    } else if (cur == MENU_RUN) {
      /* State toggle: OK=toggle, long DOWN=back */
      if (lp_edge)        config_menu_process_cmd(CTRL_MENU_CMD_BACK);
      if (press & BTN_OK) config_menu_process_cmd(CTRL_MENU_CMD_ENTER);
    } else if (cur == MENU_SENSOR_VIEW || cur == MENU_INFO_VIEW ||
               cur == MENU_EXT_VIEW) {
      /* Info screens: OK=back, long DOWN=back */
      if (lp_edge)        config_menu_process_cmd(CTRL_MENU_CMD_BACK);
      if (press & BTN_OK) config_menu_process_cmd(CTRL_MENU_CMD_BACK);
    } else {
      /* Normal navigation */
      if (lp_edge) {
        config_menu_process_cmd(CTRL_MENU_CMD_BACK);
      } else {
        if (press & BTN_DOWN) config_menu_process_cmd(CTRL_MENU_CMD_UP);
      }
      if (press & BTN_UP)   config_menu_process_cmd(CTRL_MENU_CMD_DOWN);
      if (press & BTN_OK)   config_menu_process_cmd(CTRL_MENU_CMD_ENTER);
    }

    /* ---- 4.  Control source change (MUX switch) ---- */
    config_ctrl_source_t ctrl_src = config_menu_get_ctrl_source();
    if (ctrl_src != prev_ctrl_src) {
      if (ctrl_src == CONFIG_CTRL_EXTERNAL) {
        peltier_set_source(PELTIER_CTRL_EXTERNAL);
      } else {
        peltier_set_source(PELTIER_CTRL_INTERNAL);
      }
      prev_ctrl_src = ctrl_src;
    }

    /* ---- 5.  Peltier state change ---- */
    config_peltier_state_t peltier_st = config_menu_get_peltier_state();
    if (peltier_st != prev_peltier_st) {
      if (peltier_st == CONFIG_PELTIER_RUN && ctrl_src == CONFIG_CTRL_INTERNAL) {
        pid_reset(&g_pid);
        /* PID will decide direction on first update */
      } else {
        peltier_stop();
      }
      prev_peltier_st = peltier_st;
    }

    /* ---- 6.  Bidirectional PID temperature control ---- */
    if (peltier_st == CONFIG_PELTIER_RUN && ctrl_src == CONFIG_CTRL_INTERNAL) {
      static uint32_t last_pid = 0;
      static peltier_mode_t last_dir = PELTIER_OFF;
      static uint32_t dir_change_tick = 0;
      static float prev_target = -999.0f;
      uint32_t now = HAL_GetTick();

      if ((now - last_pid) >= PID_UPDATE_MS) {
        last_pid = now;
        float target  = (float)config_menu_get_temp_target();
        float measured = g_temp_celsius;

        /* Reset integral when setpoint changes to avoid windup carryover */
        if (target != prev_target) {
          g_pid.integral = 0.0f;
          g_pid.first    = 1;
          prev_target    = target;
        }

        float out = pid_update(&g_pid, target, measured);

        /* Determine desired direction */
        peltier_mode_t desired_dir;
        uint32_t duty;
        if (out > 0.5f) {
          desired_dir = PELTIER_HEAT;
          duty = (uint32_t)out;
          if (duty < 8u) duty = 8u; /* minimum to actually move */
        } else if (out < -0.5f) {
          desired_dir = PELTIER_COOL;
          duty = (uint32_t)(-out);
          if (duty < 8u) duty = 8u;
        } else {
          desired_dir = PELTIER_OFF;
          duty = 0u;
        }

        /* Direction-change lockout: must stay in one direction
           for at least 5 seconds before switching to prevent
           rapid heat/cool toggling that stresses the Peltier. */
        if (desired_dir != last_dir && desired_dir != PELTIER_OFF
            && last_dir != PELTIER_OFF) {
          if ((now - dir_change_tick) < 5000u) {
            /* Too soon to reverse — just turn off for now */
            desired_dir = PELTIER_OFF;
            duty = 0u;
          }
        }

        if (desired_dir != last_dir) {
          dir_change_tick = now;
          last_dir = desired_dir;
        }

        g_peltier_mode = desired_dir;
        peltier_set_pwm(duty);
      }
    }

    /* ---- 7.  Display ---- */
    int8_t idx = ctrl_menu_get_current_idx(&app_menu);

    /* Detect siblings for arrow indicators */
    int8_t par = menu_items[idx].parent_id;
    uint8_t has_prev = 0, has_next = 0;
    if (idx != MENU_TEMP_INC && idx != MENU_TEMP_DEC &&
        idx != MENU_RUN && idx != MENU_SENSOR_VIEW &&
        idx != MENU_INFO_VIEW && idx != MENU_EXT_VIEW)
    {
      for (int8_t i = idx - 1; i >= 0; i--) {
        if (menu_items[i].parent_id == par) { has_prev = 1; break; }
      }
      for (int8_t i = idx + 1; i < MENU_COUNT; i++) {
        if (menu_items[i].parent_id == par) { has_next = 1; break; }
      }
    }

    u8g2_FirstPage(&u8g2);
    do {
      /* Navigation arrows */
      if (has_prev) {
        u8g2_DrawTriangle(&u8g2, 0, 16, 9, 7, 9, 25);
      }
      if (has_next) {
        u8g2_DrawTriangle(&u8g2, 127, 16, 118, 7, 118, 25);
      }

      switch (idx) {

        /* ------ Live Sensor View ------ */
        case MENU_SENSOR_VIEW:
        {
          char line1[24], line2[24];
          int32_t t_int = (int32_t)g_temp_celsius;
          float r_ohm = (float)g_rtd_raw * MAX31865_RREF / 32768.0f;
          int32_t r_x10 = (int32_t)(r_ohm * 10.0f);
          snprintf(line1, sizeof(line1), "T:%ld%cC R:%ld.%ld",
                   (long)t_int, 0xB0, (long)(r_x10/10), (long)(r_x10%10));
          snprintf(line2, sizeof(line2), "ADC:%u F:%02X",
                   (unsigned)g_rtd_raw, g_rtd_fault);
          u8g2_SetFont(&u8g2, u8g2_font_6x10_tr);
          u8g2_DrawStr(&u8g2, 0, 12, line1);
          u8g2_DrawStr(&u8g2, 0, 28, line2);
          break;
        }

        /* ------ Temperature target edit ------ */
        case MENU_TEMP_INC:
        case MENU_TEMP_DEC:
        {
          char buf[16];
          snprintf(buf, sizeof(buf), "%ld \xB0" "C",
                   (long)config_menu_get_temp_target());
          u8g2_SetFont(&u8g2, u8g2_font_helvB24_te);
          u8g2_uint_t w = u8g2_GetStrWidth(&u8g2, buf);
          u8g2_DrawStr(&u8g2, (128 - w) / 2, 30, buf);
          break;
        }

        /* ------ Run / Stop toggle ------ */
        case MENU_RUN:
        {
          const char *txt =
              (config_menu_get_peltier_state() == CONFIG_PELTIER_STOP)
                  ? "RUN" : "STOP";
          u8g2_SetFont(&u8g2, u8g2_font_helvB24_tr);
          u8g2_uint_t w = u8g2_GetStrWidth(&u8g2, txt);
          u8g2_DrawStr(&u8g2, (128 - w) / 2, 30, txt);
          break;
        }

        /* ------ External view ------ */
        case MENU_EXT_VIEW:
        {
          char line1[22];
          int32_t t_int = (int32_t)g_temp_celsius;
          snprintf(line1, sizeof(line1), "T: %ld \xB0" "C",
                   (long)t_int);
          u8g2_SetFont(&u8g2, u8g2_font_helvB14_te);
          u8g2_uint_t w1 = u8g2_GetStrWidth(&u8g2, line1);
          u8g2_DrawStr(&u8g2, (128 - w1) / 2, 14, line1);
          u8g2_DrawStr(&u8g2, 20, 30, "EXTERNAL");
          break;
        }

        /* ------ System info view ------ */
        case MENU_INFO_VIEW:
        {
          char line1[22], line2[22];
          int32_t t_int = (int32_t)g_temp_celsius;
          int32_t pwr_abs = (int32_t)(g_pid.output >= 0 ? g_pid.output : -g_pid.output);
          const char *dir = (g_peltier_mode == PELTIER_HEAT) ? "H" :
                            (g_peltier_mode == PELTIER_COOL) ? "C" : "-";
          snprintf(line1, sizeof(line1), "%s %ld%%",
                   dir, (long)pwr_abs);
          snprintf(line2, sizeof(line2), "T:%ld\xB0" "C S:%ld\xB0" "C",
                   (long)t_int,
                   (long)config_menu_get_temp_target());
          u8g2_SetFont(&u8g2, u8g2_font_helvB12_te);
          u8g2_uint_t w1i = u8g2_GetStrWidth(&u8g2, line1);
          u8g2_uint_t w2i = u8g2_GetStrWidth(&u8g2, line2);
          u8g2_DrawStr(&u8g2, (128 - w1i) / 2, 14, line1);
          u8g2_DrawStr(&u8g2, (128 - w2i) / 2, 30, line2);
          break;
        }

        /* ------ Default: show menu item name ------ */
        default:
        {
          const char *name = menu_items[idx].name;
          u8g2_SetFont(&u8g2, u8g2_font_helvB14_tr);
          u8g2_uint_t w = u8g2_GetStrWidth(&u8g2, name);
          u8g2_DrawStr(&u8g2, (128 - w) / 2, 24, name);
          break;
        }
      }
    } while (u8g2_NextPage(&u8g2));

    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
