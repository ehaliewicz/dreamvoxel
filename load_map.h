#ifndef load_map_h
#define load_map_h

#include <stdlib.h>
#include <stdint.h>
#include <kos.h>
#include "stb_image.h"

uint16_t* loadmap_get_texture(char* filename);
uint8_t* loadmap_get_heights(char* filename);
void swizzle_texture_map();
void swizzle_height_map();

#endif