#ifndef CONFIG_MENU_H
#define CONFIG_MENU_H

#include "ctrl_menu.h"

typedef enum {
    MENU_INTERNAL = 0,   /* root: INTERNAL control */
    MENU_EXTERNAL,       /* root: EXTERNAL control */
    MENU_TEMP_SET,       /* child of INTERNAL */
    MENU_STATE,          /* child of INTERNAL */
    MENU_SENSOR,         /* child of INTERNAL */
    MENU_INFO,           /* child of INTERNAL */
    MENU_TEMP_INC,       /* child of TEMP_SET */
    MENU_TEMP_DEC,       /* child of TEMP_SET */
    MENU_RUN,            /* child of STATE   (toggle) */
    MENU_SENSOR_VIEW,    /* child of SENSOR  (live temp) */
    MENU_INFO_VIEW,      /* child of INFO    (system info) */
    MENU_EXT_VIEW,       /* child of EXTERNAL (live temp, external) */
    MENU_COUNT
} config_menu_idx_t;

typedef enum {
    CONFIG_CTRL_INTERNAL = 0,
    CONFIG_CTRL_EXTERNAL
} config_ctrl_source_t;

typedef enum {
    CONFIG_PELTIER_STOP = 0,
    CONFIG_PELTIER_RUN
} config_peltier_state_t;

extern ctrl_menu_item_t menu_items[MENU_COUNT];
extern ctrl_menu_t app_menu;

void config_menu_init(void);
void config_menu_process_cmd(ctrl_menu_cmd_t cmd);

config_ctrl_source_t   config_menu_get_ctrl_source(void);
config_peltier_state_t config_menu_get_peltier_state(void);
int32_t                config_menu_get_temp_target(void);
void                   config_menu_temp_increase(void);
void                   config_menu_temp_decrease(void);

#endif
