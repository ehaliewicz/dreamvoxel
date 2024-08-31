#ifndef COMMON_H
#define COMMON_H

extern int trans_vertices;
extern int drawn_tris;
extern int patches_drawn;

extern int enable_debug;

#define TXR_SIZE 1024
#define BUMP_SIZE 512
#define MAP_SIZE 1024

#define NUM_MIPS 5

#define PATCH_SIZE 128
#define NUM_PATCHES_DIR  (MAP_SIZE/PATCH_SIZE)
#define NUM_PATCH_VERTS_DIR (NUM_PATCHES_DIR+1)
#define NUM_TOTAL_PATCHES (NUM_PATCHES_DIR*NUM_PATCHES_DIR)
#define NUM_TOTAL_PATCH_VERTS (NUM_PATCH_VERTS_DIR*NUM_PATCH_VERTS_DIR)

typedef struct {
    float x;         // x position on the map
    float y;         // y position on the map
    float height;    // height of the camera
    float pitch_ang;  // look up down angle
    float roll_ang;   // roll left right angle
    float zfar;      // distance of the camera looking forward
    float angle;     // camera angle (radians, clockwise)
} camera_t;

extern camera_t camera;

#endif