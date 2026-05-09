//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_5_frame_t ui_g_Ungroup_0;

ui_interface_number_t *ui_g_Ungroup_Pitch_Number = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_ellipse_t *ui_g_Ungroup_vision = (ui_interface_ellipse_t*)&(ui_g_Ungroup_0.data[1]);
ui_interface_number_t *ui_g_Ungroup_vision_trice_id = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[2]);
ui_interface_number_t *ui_g_Ungroup_shoot_mode = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[3]);
ui_interface_number_t *ui_g_Ungroup_pitch_temperature = (ui_interface_number_t*)&(ui_g_Ungroup_0.data[4]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 5; i < 5; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_Pitch_Number->figure_type = 6;
    ui_g_Ungroup_Pitch_Number->operate_type = 1;
    ui_g_Ungroup_Pitch_Number->layer = 0;
    ui_g_Ungroup_Pitch_Number->color = 2;
    ui_g_Ungroup_Pitch_Number->start_x = 454;
    ui_g_Ungroup_Pitch_Number->start_y = 717;
    ui_g_Ungroup_Pitch_Number->width = 2;
    ui_g_Ungroup_Pitch_Number->font_size = 20;
    ui_g_Ungroup_Pitch_Number->number = 12345;

    ui_g_Ungroup_vision->figure_type = 3;
    ui_g_Ungroup_vision->operate_type = 1;
    ui_g_Ungroup_vision->layer = 0;
    ui_g_Ungroup_vision->color = 0;
    ui_g_Ungroup_vision->start_x = 958;
    ui_g_Ungroup_vision->start_y = 538;
    ui_g_Ungroup_vision->width = 4;
    ui_g_Ungroup_vision->rx = 50;
    ui_g_Ungroup_vision->ry = 50;

    ui_g_Ungroup_vision_trice_id->figure_type = 6;
    ui_g_Ungroup_vision_trice_id->operate_type = 1;
    ui_g_Ungroup_vision_trice_id->layer = 0;
    ui_g_Ungroup_vision_trice_id->color = 4;
    ui_g_Ungroup_vision_trice_id->start_x = 909;
    ui_g_Ungroup_vision_trice_id->start_y = 720;
    ui_g_Ungroup_vision_trice_id->width = 2;
    ui_g_Ungroup_vision_trice_id->font_size = 20;
    ui_g_Ungroup_vision_trice_id->number = 12345;

    ui_g_Ungroup_shoot_mode->figure_type = 6;
    ui_g_Ungroup_shoot_mode->operate_type = 1;
    ui_g_Ungroup_shoot_mode->layer = 0;
    ui_g_Ungroup_shoot_mode->color = 8;
    ui_g_Ungroup_shoot_mode->start_x = 1271;
    ui_g_Ungroup_shoot_mode->start_y = 547;
    ui_g_Ungroup_shoot_mode->width = 2;
    ui_g_Ungroup_shoot_mode->font_size = 20;
    ui_g_Ungroup_shoot_mode->number = 12345;

    ui_g_Ungroup_pitch_temperature->figure_type = 6;
    ui_g_Ungroup_pitch_temperature->operate_type = 1;
    ui_g_Ungroup_pitch_temperature->layer = 0;
    ui_g_Ungroup_pitch_temperature->color = 0;
    ui_g_Ungroup_pitch_temperature->start_x = 429;
    ui_g_Ungroup_pitch_temperature->start_y = 665;
    ui_g_Ungroup_pitch_temperature->width = 2;
    ui_g_Ungroup_pitch_temperature->font_size = 20;
    ui_g_Ungroup_pitch_temperature->number = 12345;


    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 5; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

ui_string_frame_t ui_g_Ungroup_1;
ui_interface_string_t* ui_g_Ungroup_Pitch = &(ui_g_Ungroup_1.option);

void _ui_init_g_Ungroup_1() {
    ui_g_Ungroup_1.option.figure_name[0] = 0;
    ui_g_Ungroup_1.option.figure_name[1] = 0;
    ui_g_Ungroup_1.option.figure_name[2] = 5;
    ui_g_Ungroup_1.option.operate_type = 1;

    ui_g_Ungroup_Pitch->figure_type = 7;
    ui_g_Ungroup_Pitch->operate_type = 1;
    ui_g_Ungroup_Pitch->layer = 0;
    ui_g_Ungroup_Pitch->color = 2;
    ui_g_Ungroup_Pitch->start_x = 317;
    ui_g_Ungroup_Pitch->start_y = 720;
    ui_g_Ungroup_Pitch->width = 2;
    ui_g_Ungroup_Pitch->font_size = 20;
    ui_g_Ungroup_Pitch->str_length = 5;
    strcpy(ui_g_Ungroup_Pitch->string, "Pitch");


    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_update_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_remove_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
    _ui_init_g_Ungroup_1();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
    _ui_update_g_Ungroup_1();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
    _ui_remove_g_Ungroup_1();
}

