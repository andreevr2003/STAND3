#include "ctrl_menu.h"
#include <string.h>

static int8_t find_first_child(ctrl_menu_t *menu, int8_t parent_id)
{
    for (uint8_t i = 0; i < menu->item_count; i++) {
        if (menu->items[i].parent_id == parent_id) {
            return i;
        }
    }
    return -1;
}

static int8_t find_parent_idx(ctrl_menu_t *menu, int8_t parent_id)
{
    for (uint8_t i = 0; i < menu->item_count; i++) {
        if (menu->items[i].id == parent_id) {
            return i;
        }
    }
    return -1;
}

static int8_t find_sibling(ctrl_menu_t *menu, int8_t current_idx, int8_t direction)
{
    int8_t parent_id = menu->items[current_idx].parent_id;
    int8_t idx = current_idx + direction;

    while (idx >= 0 && idx < menu->item_count) {
        if (menu->items[idx].parent_id == parent_id) {
            return idx;
        }
        idx += direction;
    }
    return current_idx;
}

static uint8_t count_siblings(ctrl_menu_t *menu, int8_t parent_id)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < menu->item_count; i++) {
        if (menu->items[i].parent_id == parent_id) {
            count++;
        }
    }
    return count;
}

static void update_visible_window(ctrl_menu_t *menu)
{
    int8_t parent_id = menu->items[menu->current_idx].parent_id;
    uint8_t pos_in_submenu = 0;

    for (uint8_t i = 0; i < menu->item_count; i++) {
        if (menu->items[i].parent_id == parent_id) {
            if (i == menu->current_idx) break;
            pos_in_submenu++;
        }
    }

    if (pos_in_submenu < menu->visible_start) {
        menu->visible_start = pos_in_submenu;
    } else if (pos_in_submenu >= menu->visible_start + CTRL_MENU_MAX_ROWS) {
        menu->visible_start = pos_in_submenu - CTRL_MENU_MAX_ROWS + 1;
    }
}

void ctrl_menu_init(ctrl_menu_t *menu, ctrl_menu_item_t *items, uint8_t count)
{
    menu->items = items;
    menu->item_count = count;
    menu->current_idx = find_first_child(menu, -1);
    menu->edit_mode = 0;
    menu->tmp_value = 0;
    menu->visible_start = 0;
}

void ctrl_menu_process(ctrl_menu_t *menu, ctrl_menu_cmd_t cmd)
{
    if (cmd == CTRL_MENU_CMD_NONE) return;

    ctrl_menu_item_t *current = &menu->items[menu->current_idx];

    switch (cmd) {
        case CTRL_MENU_CMD_UP:
            if (menu->edit_mode) {
                menu->tmp_value++;
                if (menu->tmp_value > current->max) {
                    menu->tmp_value = current->max;
                }
                if (current->type == CTRL_MENU_ITEM_PARAM && current->callback != NULL) {
                    int16_t old_value = current->value;
                    current->value = menu->tmp_value;
                    current->callback();
                    current->value = old_value;
                }
            } else {
                menu->current_idx = find_sibling(menu, menu->current_idx, -1);
                update_visible_window(menu);
            }
            break;

        case CTRL_MENU_CMD_DOWN:
            if (menu->edit_mode) {
                menu->tmp_value--;
                if (menu->tmp_value < current->min) {
                    menu->tmp_value = current->min;
                }
                if (current->type == CTRL_MENU_ITEM_PARAM && current->callback != NULL) {
                    int16_t old_value = current->value;
                    current->value = menu->tmp_value;
                    current->callback();
                    current->value = old_value;
                }
            } else {
                menu->current_idx = find_sibling(menu, menu->current_idx, 1);
                update_visible_window(menu);
            }
            break;

        case CTRL_MENU_CMD_ENTER:
            if (menu->edit_mode) {
                current->value = menu->tmp_value;
                menu->edit_mode = 0;
                if (current->type == CTRL_MENU_ITEM_PARAM && current->callback != NULL) {
                    current->callback();
                }
            } else {
                switch (current->type) {
                    case CTRL_MENU_ITEM_PARAM:
                        menu->edit_mode = 1;
                        menu->tmp_value = current->value;
                        break;
                    case CTRL_MENU_ITEM_SUBMENU:
                        {
                            int8_t child = find_first_child(menu, current->id);
                            if (child >= 0) {
                                menu->current_idx = child;
                                menu->visible_start = 0;
                            }
                        }
                        break;
                    case CTRL_MENU_ITEM_ACTION:
                        if (current->callback != NULL) {
                            current->callback();
                        }
                        break;
                }
            }
            break;

        case CTRL_MENU_CMD_BACK:
            if (menu->edit_mode) {
                menu->edit_mode = 0;
                if (current->type == CTRL_MENU_ITEM_PARAM && current->callback != NULL) {
                    current->callback();
                }
            } else {
                int8_t parent_id = current->parent_id;
                if (parent_id >= 0) {
                    int8_t parent_idx = find_parent_idx(menu, parent_id);
                    if (parent_idx >= 0) {
                        menu->current_idx = parent_idx;
                        menu->visible_start = 0;
                        update_visible_window(menu);
                    }
                }
            }
            break;

        default:
            break;
    }
}

int8_t ctrl_menu_get_current_idx(ctrl_menu_t *menu)
{
    return menu->current_idx;
}

bool ctrl_menu_is_editing(ctrl_menu_t *menu)
{
    return menu->edit_mode != 0;
}

int16_t ctrl_menu_get_value(ctrl_menu_t *menu, uint8_t idx)
{
    if (idx >= menu->item_count) return 0;

    if (menu->edit_mode && idx == menu->current_idx) {
        return menu->tmp_value;
    }
    return menu->items[idx].value;
}

void ctrl_menu_set_value(ctrl_menu_t *menu, uint8_t idx, int16_t value)
{
    if (idx >= menu->item_count) return;
    menu->items[idx].value = value;
}

uint8_t ctrl_menu_get_visible_count(ctrl_menu_t *menu)
{
    int8_t parent_id = menu->items[menu->current_idx].parent_id;
    uint8_t total = count_siblings(menu, parent_id);
    return (total > CTRL_MENU_MAX_ROWS) ? CTRL_MENU_MAX_ROWS : total;
}

int8_t ctrl_menu_get_visible_idx(ctrl_menu_t *menu, uint8_t row)
{
    int8_t parent_id = menu->items[menu->current_idx].parent_id;
    uint8_t target = menu->visible_start + row;
    uint8_t count = 0;

    for (uint8_t i = 0; i < menu->item_count; i++) {
        if (menu->items[i].parent_id == parent_id) {
            if (count == target) {
                return i;
            }
            count++;
        }
    }
    return -1;
}

uint8_t ctrl_menu_get_cursor_row(ctrl_menu_t *menu)
{
    int8_t parent_id = menu->items[menu->current_idx].parent_id;
    uint8_t pos = 0;

    for (uint8_t i = 0; i < menu->item_count; i++) {
        if (menu->items[i].parent_id == parent_id) {
            if (i == menu->current_idx) {
                return pos - menu->visible_start;
            }
            pos++;
        }
    }
    return 0;
}
