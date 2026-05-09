//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_number_t *ui_g_Ungroup_Pitch_Number;
extern ui_interface_ellipse_t *ui_g_Ungroup_vision;
extern ui_interface_number_t *ui_g_Ungroup_vision_trice_id;
extern ui_interface_number_t *ui_g_Ungroup_shoot_mode;
extern ui_interface_number_t *ui_g_Ungroup_pitch_temperature;
extern ui_interface_string_t *ui_g_Ungroup_Pitch;

void ui_init_g_Ungroup(void);
void ui_update_g_Ungroup(void);
void ui_remove_g_Ungroup(void);


#endif // UI_g_H
