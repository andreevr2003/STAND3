#ifndef CTRL_MENU_H
#define CTRL_MENU_H

#include <stdint.h>
#include <stdbool.h>

#define CTRL_MENU_MAX_NAME_LEN    12
#define CTRL_MENU_MAX_ITEMS       16
#define CTRL_MENU_MAX_ROWS        4

typedef enum {
    CTRL_MENU_CMD_NONE = 0,
    CTRL_MENU_CMD_UP,
    CTRL_MENU_CMD_DOWN,
    CTRL_MENU_CMD_ENTER,
    CTRL_MENU_CMD_BACK
} ctrl_menu_cmd_t;

typedef enum {
    CTRL_MENU_ITEM_SUBMENU = 0,
    CTRL_MENU_ITEM_PARAM,
    CTRL_MENU_ITEM_ACTION
} ctrl_menu_item_type_t;

typedef struct {
    int8_t id;
    int8_t parent_id;
    char name[CTRL_MENU_MAX_NAME_LEN];
    ctrl_menu_item_type_t type;
    int16_t value;
    int16_t min;
    int16_t max;
    void (*callback)(void);
} ctrl_menu_item_t;

typedef struct {
    ctrl_menu_item_t *items;
    uint8_t item_count;
    int8_t current_idx;
    int8_t edit_mode;
    int16_t tmp_value;
    uint8_t visible_start;
} ctrl_menu_t;

void ctrl_menu_init(ctrl_menu_t *menu, ctrl_menu_item_t *items, uint8_t count);
void ctrl_menu_process(ctrl_menu_t *menu, ctrl_menu_cmd_t cmd);

int8_t ctrl_menu_get_current_idx(ctrl_menu_t *menu);
bool ctrl_menu_is_editing(ctrl_menu_t *menu);
int16_t ctrl_menu_get_value(ctrl_menu_t *menu, uint8_t idx);
void ctrl_menu_set_value(ctrl_menu_t *menu, uint8_t idx, int16_t value);

uint8_t ctrl_menu_get_visible_count(ctrl_menu_t *menu);
int8_t ctrl_menu_get_visible_idx(ctrl_menu_t *menu, uint8_t row);
uint8_t ctrl_menu_get_cursor_row(ctrl_menu_t *menu);

#endif
