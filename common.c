#include "common.h"

int trans_vertices = 0;
int drawn_tris = 0;
int patches_drawn = 0;
int enable_debug = 0;

// Camera definition
camera_t camera = {
    .x       = MAP_SIZE/2.0f,
    .y       = MAP_SIZE/2.0f,
    .height  = 70.0f,
    .pitch_ang = 0.0f,
    .roll_ang = 0.0f,
    .zfar    = 500.0f,
    .angle   = 0.0f //1.5 * 3.141592 // (= 270 deg)
};
