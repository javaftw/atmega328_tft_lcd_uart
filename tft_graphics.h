/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   tft_graphics.h
 * Author: root
 *
 * Created on 11 April 2019, 11:14 PM
 */

#ifndef TFT_GRAPHICS_H
#define TFT_GRAPHICS_H

uint16_t rgb_565(uint8_t r, uint8_t g, uint8_t b);
uint16_t rgb888_to_565(uint8_t r, uint8_t g, uint8_t b);
//general 
void clear_screen(void);
void fill_rect(uint16_t x1, uint16_t y1, ui32 w, ui32 h, uint16_t colour);
void draw_rect(uint16_t x, uint16_t y, ui16 w, ui16 h, uint16_t colour);
void plot_pixel(uint16_t x, uint16_t y, uint16_t colour);
void plot_2D_graph(void);
void draw_v_line(uint16_t x, uint16_t y, uint16_t len, uint16_t colour);
void draw_h_line(uint16_t x, uint16_t y, uint16_t len, uint16_t colour);
void draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, ui8 thickness, uint16_t colour);
void draw_shape(uint16_t coords[], uint8_t size, bool closed, uint8_t thickness, uint16_t colour);
void draw_circle(uint16_t cx, uint16_t cy, uint16_t radius, uint16_t colour);


extern void write_command(uint8_t);
extern void write_command(uint8_t cmd, uint8_t dat);
extern void write_command(uint8_t cmd, uint8_t dat[], uint8_t size);
extern void write_data(uint8_t dat[], uint8_t size);
extern void put_bus_byte(uint8_t);
extern uint8_t read_command(uint8_t);

uint16_t rgb_565(uint8_t r, uint8_t g, uint8_t b)
{
    r &= 0x1F; //0-31
    g &= 0x3F; //0-63
    b &= 0x1F; //0-31
    return ((0xFFFF & (r << 11)) | (g << 5)) | b;
}

uint16_t rgb888_to_565(uint8_t r, uint8_t g, uint8_t b)
{
    return rgb_565((r >> 3), (g >> 2), (b >> 3));
}

/******************************************************************************
 *                                                          GRAPHICS FUNCTIONS*/
void clear_screen(void)
{
    uint8_t coladdr[] = {0x00, 0x00, 0x01, 0x3F};
    uint8_t pageaddr[] = {0x00, 0x00, 0x01, 0xDF};
    write_command(OPC_WRITE_COL_ADDR, coladdr, 4);
    write_command(OPC_WRITE_PAGE_ADDR, pageaddr, 4);
    uint8_t black[] = {0x00, 0x00};
    write_command(OPC_WRITE_MEM_START, black, 2);
    write_command(OPC_WRITE_MEM_CONTINUE, black, 2);
    active_CS;
    put_bus_byte(0x00);
    put_bus_byte(0x00);
    uint32_t tempw = DISPLAY_WIDTH, temph = DISPLAY_HEIGHT;
    uint32_t area = tempw*temph;
    for (uint32_t i = 0; i < area; i++) {
        strobe_WR;
        strobe_WR;
    }
    idle_CS;
    _IDLE;
    return;
}

void fill_rect(uint16_t x, uint16_t y, ui32 w, ui32 h, uint16_t colour)
{

    //uint32_t tempw = w, temph = h;
    ui32 area = w*h;
    w += (x - 1);
    h += (y - 1);
    uint8_t cxhi = 0xFF & (x >> 8);
    uint8_t cxlo = 0xFF & x;
    uint8_t cyhi = 0xFF & (w >> 8);
    uint8_t cylo = 0xFF & w;
    uint8_t pxhi = 0xFF & (y >> 8);
    uint8_t pxlo = 0xFF & y;
    uint8_t pyhi = 0xFF & (h >> 8);
    uint8_t pylo = 0xFF & h;
    uint8_t colhi = 0xFF & (colour >> 8);
    uint8_t collo = 0xFF & colour;
    uint8_t caddr[] = {cxhi, cxlo, cyhi, cylo};
    uint8_t paddr[] = {pxhi, pxlo, pyhi, pylo};
    write_command(OPC_WRITE_COL_ADDR, caddr, 4);
    write_command(OPC_WRITE_PAGE_ADDR, paddr, 4);
    uint8_t col[] = {colhi, collo};
    write_command(OPC_WRITE_MEM_START, col, 2);
    write_command(OPC_WRITE_MEM_CONTINUE, col, 2);
    active_CS;
    for (uint32_t i = 1; i < area; i++) {
        put_bus_byte(col[0]);
        put_bus_byte(col[1]);
    }
    idle_CS;
    _IDLE;
    return;
}

void plot_pixel(uint16_t x, uint16_t y, uint16_t colour)
{
    fill_rect(x, y, 1, 1, colour);
    return;
}

void draw_h_line(uint16_t x, uint16_t y, uint16_t len, uint16_t colour)
{
    fill_rect(x, y, len, 1, colour);
    return;
}

void draw_v_line(uint16_t x, uint16_t y, uint16_t len, uint16_t colour)
{
    fill_rect(x, y, 1, len, colour);
    return;
}

void draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint8_t thickness, uint16_t colour)
{
    //housekeeping
    if (x1 > x2) {
        uint16_t tempx = x1;
        x1 = x2;
        x2 = tempx;
        uint16_t tempy = y1;
        y1 = y2;
        y2 = tempy;
    }
    //vertical line
    if (x1 == x2) {
        draw_v_line(x1, y1, y2 - y1, colour);
        return;
    }
    //horizontal line
    if (y1 == y2) {
        draw_h_line(x1, y1, x2 - x1, colour);
        return;
    }
    uint16_t i;
    double LY = y1, X1 = x1, X2 = x2, Y1 = y1, Y2 = y2;
    double step = (Y2 - Y1) / (X2 - X1);
    for (i = 0; i < (X2 - X1); i++) {
        LY += step;
        if (step > 1.0) {
            fill_rect(x1 + i, (uint16_t) (LY - step), thickness, (uint16_t) (thickness + step), colour);
        } else {
            fill_rect(x1 + i, (uint16_t) LY, thickness, thickness, colour);
        }
    }
    return;
}

void draw_shape(uint16_t coords[], uint8_t size, bool closed, uint8_t thickness, uint16_t colour)
{
    uint8_t i;
    for (i = 0; i < size - 2; i += 2) {
        draw_line(coords[i + 0], coords[i + 1], coords[i + 2], coords[i + 3], thickness, colour);
    }
    if (closed) {
        draw_line(coords[size - 2], coords[size - 1], coords[0], coords[1], thickness, colour);
    }
}

void draw_rect(uint16_t x, uint16_t y, ui16 w, ui16 h, uint16_t colour)
{
    draw_h_line(x, y, w, colour);
    draw_h_line(x, y + h, w, colour);
    draw_v_line(x, y, h, colour);
    draw_v_line(x + w, y, h, colour);
    return;
}

void draw_circle(uint16_t cx, uint16_t cy, uint16_t radius, uint16_t colour)
{
    /* Bresenham Circle - http://rosettacode.org/wiki/Bitmap/Midpoint_circle_algorithm#C*/
    int f = 1 - radius;
    int ddF_x = 0;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    plot_pixel(cx, cy + radius, colour);
    plot_pixel(cx, cy - radius, colour);
    plot_pixel(cx + radius, cy, colour);
    plot_pixel(cx - radius, cy, colour);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x + 1;
        plot_pixel(cx + x, cy + y, colour);
        plot_pixel(cx - x, cy + y, colour);
        plot_pixel(cx + x, cy - y, colour);
        plot_pixel(cx - x, cy - y, colour);
        plot_pixel(cx + y, cy + x, colour);
        plot_pixel(cx - y, cy + x, colour);
        plot_pixel(cx + y, cy - x, colour);
        plot_pixel(cx - y, cy - x, colour);
    }
}

void plot_2D_graph(void)
{

    return;
}


#endif /* TFT_GRAPHICS_H */

