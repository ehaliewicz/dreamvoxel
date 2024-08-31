#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_TGA
#include "load_map.h"

#define PACK_PIXEL(r, g, b) ( ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3) )

typedef struct {
    char texture[30];
    char heights[30];
    char bump[30];
} surface_info_t;

surface_info_t files[] = {{
    .texture = "/rd/images/texture/C1W.tga",
    .heights  = "/rd/images/height/D1.tga",
    .bump = "/rd/images/bump/bump.dt"
}};

uint16_t* separated_RGB_to_RGB565(uint8_t *rgb, int image_width, int image_height){
    uint16_t *result = (uint16_t *) malloc(image_width * image_height * sizeof(uint16_t));
    for (int i = 0; i < image_width * image_height; i++){
        result[i] = PACK_PIXEL(rgb[i * 3], rgb[i * 3 + 1], rgb[i * 3 + 2]);
    }
    return result;
}

uint16_t* loadmap_get_texture(char* filename){
    int image_width, image_height, comp;

    uint8_t *separated_RGB = stbi_load(filename, &image_width, &image_height, &comp, STBI_rgb);
    uint16_t *texture_map = separated_RGB_to_RGB565(separated_RGB, image_width, image_height);

    return texture_map;
}

uint8_t* loadmap_get_heights(char* filename){
    int image_width, image_height, comp;

    uint8_t *height_map = stbi_load(filename, &image_width, &image_height, &comp, STBI_grey);

    return height_map;
}



uint16_t* loadmap_get_bump(uint8_t index) {
    FILE* f = fopen(files[index].bump, "rb");

    
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    fseek(f, 0, SEEK_SET);
    uint16_t* buf = memalign(2, 128*1024*2);
    (int) fread((uint8_t*)buf,1,fsize,f);
    fclose(f);
    return (uint16_t*)buf;

    //FILE *f = stbi__fopen(files[index].bump, "rb");
    //uint8_t *height_map = stbi_load(files[index].heights, &image_width, &image_height, &comp, STBI_grey);

    //return height_map;
}
