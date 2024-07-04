/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   fontworker.h
 * Author: root
 *
 * Created on 09 April 2019, 8:32 AM
 */

#ifndef FONTWORKER_H
#define FONTWORKER_H

#include "ili9481.h"





/******************************************************FontSettings "Object" **/
//typedef struct FontSettings_t FontSettings_t, *pFS;


typedef enum {
    TYPE_CHAR,
    TYPE_UCHAR,
    TYPE_SHORT,
    TYPE_USHORT,
    TYPE_INT,
    TYPE_UINT,
    TYPE_LONG,
    TYPE_ULONG,
    TYPE_DOUBLE,
    TYPE_FLOAT,
    TYPE_CHARARR
} NumType;

typedef struct BoundingBox {
    ui16 x;
    ui16 y;
    ui16 w;
    ui16 h;
} BoundingBox;

typedef struct Point {
    ui16 x;
    ui16 y;
} Point;

struct FontSettings_t {
    ui8 font_size;
    ui8 font_pixel_w;
    ui8 font_pixel_h;
    ui8 font_pixel_hpad;
    ui8 font_pixel_vpad;
    Point position;
    Point cursor;
    ui16 fg_colour;
    ui16 bg_colour;

    enum Alignment {
        ALIGN_LEFT,
        ALIGN_CENTER,
        ALIGN_RIGHT
    } alignment;

    enum Effect {
        FX_NONE,
        FX_UNDERLINE,
        FX_BOXED,
        FX_INVERT,
    } effect;

    BoundingBox bbox;

} FontSettings_t;

struct FontSettings_t *pFS = &FontSettings_t;

void initFS(ui8 size, ui8 font_px_w, ui8 font_px_h, ui8 font_px_pad_h, ui8 font_px_pad_v, ui16 xpos, ui16 ypos, ui16 colour);
void calc_bbox(const char *);
void apply_effect(void);

void print_char(char c);
void print_str(const char * c);
void print_pgm_str(PGM_P arr_name);
void printf_str(const char * c, void *p, NumType type);
void printfi_str(const char * c, int i);
void print_int(ui16 n);
void print_float(float flt);
const ui16 ** get_char(char c);
//external
extern void fill_rect(ui16 x, ui16 y, ui32 w, ui32 h, ui16 colour);
extern void draw_rect(ui16 x, ui16 y, ui16 w, ui16 h, ui16 colour);
extern void draw_line(ui16 x1, ui16 y1, ui16 x2, ui16 y2, ui8 thickness, ui16 colour);

/************************************************************** INIT */

void initFS(ui8 size, ui8 font_px_w, ui8 font_px_h, ui8 font_px_pad_h, ui8 font_px_pad_v, ui16 xpos, ui16 ypos, ui16 colour)
{
    pFS->alignment = FontSettings_t::ALIGN_LEFT;
    pFS->effect = FontSettings_t::FX_NONE;
    pFS->font_size = size;
    pFS->font_pixel_w = font_px_w;
    pFS->font_pixel_h = font_px_h;
    pFS->font_pixel_hpad = font_px_pad_h;
    pFS->font_pixel_vpad = font_px_pad_v;
    pFS->position.x = xpos;
    pFS->position.y = ypos;
    pFS->cursor.y = ypos;
    pFS->cursor.x = xpos;
    pFS->fg_colour = colour;
    pFS->bg_colour = 0;
}

void calc_bbox(const char * c)
{
    pFS->bbox.w = strlen(c) * pFS->font_size * (pFS->font_pixel_w + pFS->font_pixel_hpad);
    pFS->bbox.h = pFS->font_size * (pFS->font_pixel_h + pFS->font_pixel_vpad);
    switch (pFS->alignment) {
    case FontSettings_t::ALIGN_LEFT:
        pFS->bbox.x = pFS->position.x;
        pFS->cursor.x = pFS->bbox.x;
        pFS->bbox.y = pFS->position.y;
        break;

    case FontSettings_t::ALIGN_CENTER:
        pFS->bbox.x = pFS->position.x - (pFS->bbox.w / 2);
        pFS->cursor.x = pFS->bbox.x;
        pFS->bbox.y = pFS->position.y;
        break;

    case FontSettings_t::ALIGN_RIGHT:
        pFS->bbox.x = pFS->position.x - pFS->bbox.w;
        pFS->cursor.x = pFS->bbox.x;
        pFS->bbox.y = pFS->position.y;
        break;

    }
}

void apply_effect(void)
{
    switch (pFS->effect) {
    case FontSettings_t::FX_NONE:
        break;

    case FontSettings_t::FX_BOXED:
        draw_rect(pFS->bbox.x - (2 * pFS->font_pixel_hpad),
                pFS->bbox.y - (1 * pFS->font_pixel_vpad),
                pFS->bbox.w + (1 * pFS->font_pixel_hpad),
                pFS->bbox.h + (0 * pFS->font_pixel_vpad),
                pFS->fg_colour);
        break;

    case FontSettings_t::FX_INVERT:
        fill_rect(pFS->bbox.x - (2 * pFS->font_pixel_hpad),
                pFS->bbox.y - (1 * pFS->font_pixel_vpad),
                pFS->bbox.w + (1 * pFS->font_pixel_hpad),
                pFS->bbox.h + (0 * pFS->font_pixel_vpad),
                pFS->fg_colour);
        break;

    case FontSettings_t::FX_UNDERLINE:
        draw_line(pFS->bbox.x,
                pFS->bbox.y + pFS->bbox.h - pFS->font_pixel_vpad,
                pFS->bbox.x + pFS->bbox.w - pFS->font_pixel_hpad,
                pFS->bbox.y + pFS->bbox.h - pFS->font_pixel_vpad,
                1,
                pFS->fg_colour);
        break;

    }
    return;
}

/************************************************************** Font Functions */

void print_char(char c)
{
    if (c == '\n') {
        pFS->position.y += (pFS->font_pixel_h + pFS->font_pixel_vpad) * pFS->font_size;
        pFS->position.x = 0;
        pFS->cursor.x = 0;
    }
    if (pFS->position.y > DISPLAY_HEIGHT - pFS->font_pixel_h) {
        pFS->position.y = 0;
        clear_screen();
        _delay_ms(750);
    }
    if (c == '\r') {
        pFS->position.x = 0;
        pFS->cursor.x = 0;
    } else {
        const ui16 ** chararr = get_char(c);
        ui8 idx;
        ui16 chardata;
        for (ui8 i = 1; i < pgm_read_word(&(chararr[0])) + 1; i++) {
            idx = i;
            chardata = pgm_read_word(&(chararr[idx]));
            fill_rect(pFS->cursor.x + (pFS->font_size * (0x0F & (chardata >> 12))),
                    pFS->position.y + (pFS->font_size * (0x0F & (chardata >> 8))),
                    pFS->font_size * (0x0F & (chardata >> 4)),
                    pFS->font_size * (0x0F & (chardata >> 0)),
                    (pFS->effect == FontSettings_t::FX_INVERT) ? pFS->bg_colour : pFS->fg_colour);
        }
    }
    return;
}

void print_str(const char * c)
{
    ui8 step = pFS->font_size * (pFS->font_pixel_w + pFS->font_pixel_hpad);
    pFS->cursor.x = pFS->position.x;
    calc_bbox(c);
    apply_effect();
    while (*c - '\0') {
        print_char(*c);
        pFS->cursor.x += step;
        c++;
    }
    pFS->position.x = pFS->cursor.x;
    return;
}

void print_pgm_str(const char* arr_name)
{
    char buffer[26];
    strcpy_P(buffer, arr_name);
    print_str(buffer);
}

void print_float(float flt)
{
    char str[8];
    print_str(dtostrf(flt, 5, 3, str));
    return;
}

void print_int(ui16 n)
{

    char str[5];
    print_str(itoa(n, str, 10));
    return;
}

void printf_str(const char * c, void *p, NumType type)
{
    ui8 numlen = 0x0A;
    ui8 str_len = strlen(c);
    char fmtstr[numlen + 1 + str_len];
    switch (type) {
    case TYPE_CHAR:
        sprintf(fmtstr, c, *(char*) p);
        break;
    case TYPE_UCHAR:
        sprintf(fmtstr, c, *(unsigned char*) p);
        break;

    case TYPE_SHORT:
        sprintf(fmtstr, c, *(short*) p);
        break;


    case TYPE_USHORT:
        sprintf(fmtstr, c, *(unsigned short*) p);
        break;

    case TYPE_INT:
        sprintf(fmtstr, c, *(int*) p);
        break;

    case TYPE_UINT:
        sprintf(fmtstr, c, *(unsigned int*) p);
        break;

    case TYPE_LONG:
        sprintf(fmtstr, c, *(long*) p);
        break;


    case TYPE_ULONG:
        sprintf(fmtstr, c, *(unsigned long*) p);
        break;


    case TYPE_FLOAT:
        sprintf(fmtstr, c, *(float*) p);
        break;

    case TYPE_DOUBLE:
        sprintf(fmtstr, c, *(double*) p);
        break;
        break;

    case TYPE_CHARARR:
        sprintf(fmtstr, c, *(char*) p);
        break;
    }

    print_str(fmtstr);
    return;
}

void printfi_str(const char * c, int i)
{
    ui8 numlen = 0x0A;
    ui8 str_len = strlen(c);
    char fmtstr[numlen + 1 + str_len];
    sprintf(fmtstr, c, i);
    print_str(fmtstr);
    return;
}

#endif /* FONTWORKER_H */

