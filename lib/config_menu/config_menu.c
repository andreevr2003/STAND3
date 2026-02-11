#include "config_menu.h"
#include <stddef.h>

static config_ctrl_source_t   ctrl_source   = CONFIG_CTRL_INTERNAL;
static config_peltier_state_t peltier_state = CONFIG_PELTIER_STOP;
static uint32_t temp_target = 50u;   /* default: 50 C */

static void temp_inc_callback(void);
static void temp_dec_callback(void);
static void run_toggle_callback(void);

ctrl_menu_item_t menu_items[MENU_COUNT] = {
    /* id, parent, name,        type,                   value, min, max, callback */
    {0,    -1,     "INTERNAL",  CTRL_MENU_ITEM_SUBMENU, 0,     0,   0,  NULL},
    {1,    -1,     "EXTERNAL",  CTRL_MENU_ITEM_SUBMENU, 0,     0,   0,  NULL},
    {2,     0,     "TEMP SET",  CTRL_MENU_ITEM_SUBMENU, 0,     0,   0,  NULL},
    {3,     0,     "STATE",     CTRL_MENU_ITEM_SUBMENU, 0,     0,   0,  NULL},
    {4,     0,     "SENSOR",    CTRL_MENU_ITEM_SUBMENU, 0,     0,   0,  NULL},
    {5,     0,     "INFO",      CTRL_MENU_ITEM_SUBMENU, 0,     0,   0,  NULL},
    {6,     2,     "TMP +",     CTRL_MENU_ITEM_ACTION,  0,     0,   0,  temp_inc_callback},
    {7,     2,     "TMP -",     CTRL_MENU_ITEM_ACTION,  0,     0,   0,  temp_dec_callback},
    {8,     3,     "RUN",       CTRL_MENU_ITEM_ACTION,  0,     0,   0,  run_toggle_callback},
    {9,     4,     "SENSOR",    CTRL_MENU_ITEM_ACTION,  0,     0,   0,  NULL},
    {10,    5,     "INFO",      CTRL_MENU_ITEM_ACTION,  0,     0,   0,  NULL},
    {11,    1,     "EXTERNAL",  CTRL_MENU_ITEM_ACTION,  0,     0,   0,  NULL},
};

ctrl_menu_t app_menu;

static void temp_inc_callback(void)
{
    if (temp_target < 150u) {
        temp_target += 5u;
        if (temp_target > 150u) temp_target = 150u;
    }
}

static void temp_dec_callback(void)
{
    if (temp_target > 0u) {
        if (temp_target >= 5u)
            temp_target -= 5u;
        else
            temp_target = 0u;
    }
}

static void run_toggle_callback(void)
{
    if (peltier_state == CONFIG_PELTIER_STOP) {
        peltier_state = CONFIG_PELTIER_RUN;
    } else {
        peltier_state = CONFIG_PELTIER_STOP;
    }
}

void config_menu_init(void)
{
    ctrl_menu_init(&app_menu, menu_items, MENU_COUNT);
}

void config_menu_process_cmd(ctrl_menu_cmd_t cmd)
{
    ctrl_menu_process(&app_menu, cmd);

    int8_t idx = ctrl_menu_get_current_idx(&app_menu);
    int8_t parent = menu_items[idx].parent_id;

    /* Stop peltier when returning to root level */
    if (parent == -1) {
        peltier_state = CONFIG_PELTIER_STOP;
    }

    /* Auto-set internal source when inside INTERNAL subtree */
    if (parent == menu_items[MENU_INTERNAL].id ||
        parent == menu_items[MENU_TEMP_SET].id ||
        parent == menu_items[MENU_STATE].id ||
        parent == menu_items[MENU_SENSOR].id ||
        parent == menu_items[MENU_INFO].id) {
        ctrl_source = CONFIG_CTRL_INTERNAL;
    }

    /* Auto-set external source when inside EXTERNAL subtree */
    if (parent == menu_items[MENU_EXTERNAL].id) {
        ctrl_source = CONFIG_CTRL_EXTERNAL;
    }
}

config_ctrl_source_t config_menu_get_ctrl_source(void)
{
    return ctrl_source;
}

config_peltier_state_t config_menu_get_peltier_state(void)
{
    return peltier_state;
}

uint32_t config_menu_get_temp_target(void)
{
    return temp_target;
}

void config_menu_temp_increase(void)
{
    temp_inc_callback();
}

void config_menu_temp_decrease(void)
{
    temp_dec_callback();
}
