/* KallistiOS ##version##

   examples/dreamcast/pvr/bumpmap/bump.c
   Copyright (C) 2014 Lawrence Sebald

   This example demonstrates the use of bumpmaps on a surface.
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <dc/pvr.h>
#include <dc/maple.h>
#include <dc/fmath.h>
#include <dc/matrix3d.h>
#include <dc/matrix.h>
#include <dc/pvr.h>
#include <dc/vec3f.h>
#include <dc/maple/controller.h>

#include <kos/init.h>

#include <kmg/kmg.h>

#include "load_map.h"

#include "common.h"

//static pvr_ver

static pvr_poly_hdr_t norm_map_hdr;
static pvr_poly_hdr_t strip_hdr;
static pvr_sprite_hdr_t line_hdr, red_line_hdr;
static float bumpiness = 0.5f;
static pvr_ptr_t bump, txr;

uint8_t* height_map = NULL;


// 512, 256, 128, 64, 32
int mip_sizes[5] = {MAP_SIZE, MAP_SIZE/2, MAP_SIZE/4, MAP_SIZE/8, MAP_SIZE/16};
uint8_t* height_map_mips[] = {NULL,NULL,NULL,NULL,NULL,NULL};

//512


int enable_gouraud_lighting = 1;
int enable_bump = 1;
int enable_texture = 1;

#define CLAMP(low, high, value) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

static matrix_t _perspective_mtrx __attribute__((aligned(32)));

void mat_perspective_fov(float fov, float aspect, float zNear, float zFar) {
  float s = 1.0f / ftan(fov * 0.5f * F_PI / 180.0f);
  //float local_mat[4][4] __attribute__((aligned(32))) = {
  //    {s / aspect, 0.0f, 0.0f, 0.0f},
  //    {0.0f, s, 0.0f, 0.0f},
  //    {0.0f, 0.0f, (zFar + zNear) / (zNear - zFar), (2.0f * zFar * zNear) / (zNear - zFar)},
  //    {0.0f, 0.0f, -1.0f, 0.0f}};
  float local_mat[4][4] __attribute__((aligned(32))) = {
      {s, 0.0f, 0.0f, 0.0f},
      {0.0f, s, 0.0f, 0.0f},
      {0.0f, 0.0f, zFar / (zFar - zNear), (zFar * zNear) / (zFar - zNear)},
      {0.0f, 0.0f, -1.0f, 0.0f}};
  memcpy(_perspective_mtrx, local_mat, sizeof(matrix_t));
  mat_load(&_perspective_mtrx);
}


uint32_t get_map_offset(int x, int y) {
    x &= MAP_SIZE-1;
    y &= MAP_SIZE-1;
    return (y << 9) | x;

    int low_x = x & 0b11;
    int low_y = y & 0b11;
    int high_x = (x&0b1111111100)>>2;
    int high_y = (y&0b1111111100)>>2;

    // YYYYYYYY XXXXXXXX yy xx
    return (high_y << 12) | (high_x << 4) | (low_y << 2) | (low_x);
}

static pvr_ptr_t load_bump(const char fn[]) {
    FILE *fp;
    pvr_ptr_t rv;

    if(!(rv = pvr_mem_malloc(BUMP_SIZE * BUMP_SIZE * 2))) {
        printf("Failed to allocate PVR memory!\n");
        return NULL;
    }

    if(!(fp = fopen(fn, "rb"))) {
        printf("Couldn't open file: %s\n", strerror(errno));
        pvr_mem_free(rv);
        return NULL;
    }


    if(fread(rv, 1, BUMP_SIZE * BUMP_SIZE * 2, fp) != BUMP_SIZE * BUMP_SIZE * 2) {
        printf("Failed to read texture from file!\n");
        fclose(fp);
        pvr_mem_free(rv);
        return NULL;
    }
    fclose(fp);

    return rv;
}

static pvr_ptr_t load_kmg(const char fn[]) {
    kos_img_t img;
    pvr_ptr_t rv;

    if(kmg_to_img(fn, &img)) {
        printf("Failed to load image file: %s\n", fn);
        return NULL;
    }

    if(!(rv = pvr_mem_malloc(img.byte_count))) {
        printf("Couldn't allocate memory for texture!\n");
        kos_img_free(&img, 0);
        return NULL;
    }

    pvr_txr_load_kimg(&img, rv, 0);
    kos_img_free(&img, 0);

    return rv;
}

#define min(x,y) ((x) < (y) ? (x) : (y))
#define max(x,y) ((x) > (y) ? (x) : (y))

float u8ToF(uint8_t val) {
	return (float)val/ 255.0f;
}

// 16 patches



static void setup(void) {
    pvr_poly_cxt_t cxt;
    pvr_sprite_cxt_t line_cxt, red_line_cxt;
    
    /* Load the textures. */
    if(!(bump = load_bump("rd/D1_DEPTH_DOUBLED_NRM_512.dt"))) { 
        exit(EXIT_FAILURE);
    }
    
    height_map_mips[0] = loadmap_get_heights("/rd/D1.tga");

    int prev_mip_size = MAP_SIZE;
    for(int i = 1; i < NUM_MIPS; i++) {
        int mip_size = prev_mip_size>>1;
        height_map_mips[i] = malloc(mip_size*mip_size);
        for(int y = 0; y < mip_size; y++) {
            int prev_mip_y = y<<1;
            int prev_mip_dy = prev_mip_y+1;
            for(int x = 0; x < mip_size; x++) {
                int prev_mip_x = x<<1;
                int prev_mip_rx = prev_mip_x+1;
                int sum_vals = height_map_mips[i-1][prev_mip_y*prev_mip_size+prev_mip_x];
                sum_vals += height_map_mips[i-1][prev_mip_y*prev_mip_size+prev_mip_rx];
                sum_vals += height_map_mips[i-1][prev_mip_dy*prev_mip_size+prev_mip_x];
                sum_vals += height_map_mips[i-1][prev_mip_dy*prev_mip_size+prev_mip_rx];
                height_map_mips[i][y*mip_size+x] = sum_vals>>2;
            }
        } 
    }

    /*
    // 200kB normal map!!
    avg_normals = malloc(NUM_STEPS*NUM_STEPS*sizeof(vec3f_t));
    for(int y = 0; y < NUM_STEPS; y++) {
        for(int x = 0; x < NUM_STEPS; x++) {
            int map_x = x * PX_STEP;
            int map_y = y * PX_STEP;
            int radius = PX_STEP/2;

            float avg_x = 0.0f, avg_y = 0.0f, avg_z = 0.0f;
            
            //int off = get_map_offset(map_x,map_y); //ty*512+tx;
            //float left = ((float)height_map[get_map_offset(map_x-1,map_y)]);
            //float right = ((float)height_map[get_map_offset(map_x+1,map_y)]);
            //float top = ((float)height_map[get_map_offset(map_x,map_y-1)]);
            //float bottom = ((float)height_map[get_map_offset(map_x,map_y+1)]);
            //float nx = 2.0f*(right-left);
            //float ny = -4.0f;
            //float nz = 2.0f*(bottom-top); 
            //avg_x = nx;
            //avg_y = ny;
            //avg_z = nz;
            

            
            for(int ty = map_y-radius; ty < map_y+radius; ty++) {
                for(int tx = map_x-radius; tx < map_x+radius; tx++) {
                    float left = ((float)height_map[get_map_offset(tx-1,ty)]);
                    float right = ((float)height_map[get_map_offset(tx+1,ty)]);
                    float top = ((float)height_map[get_map_offset(tx,ty-1)]);
                    float bottom = ((float)height_map[get_map_offset(tx,ty+1)]);
                    float x = 2.0f*(right-left);
                    float y = -4.0f;
                    float z = 2.0f*(bottom-top); 
                    //vec3f_normalize(avg_x, avg_y, avg_z);
                    float l;
                    vec3f_length(x,y,z,l);
                    x /= l;
                    y /= l;
                    z /= l;
                    avg_x += x;
                    avg_y += y;
                    avg_z += z;
                }
            }
            
            float l;
            vec3f_length(avg_x,avg_y,avg_z,l);
            avg_x /= l;
            avg_y /= l;
            avg_z /= l;
            avg_normals[y*NUM_STEPS+x].x = avg_x;
            avg_normals[y*NUM_STEPS+x].y = avg_y;
            avg_normals[y*NUM_STEPS+x].z = avg_z;


            //normals[off].x = x;
            //normals[off].y = y;
            //normals[off].z = z;

            //normals[off].x = u8ToF(height_map[get_map_offset(lx,y)]) - u8ToF(height_map[get_map_offset(rx,y)]);
            //normals[off].y = u8ToF(height_map[get_map_offset(x,dy)]) - u8ToF(height_map[get_map_offset(x,uy)]);
            //normals[off].z = sqrtf(1 - normals[off].x*normals[off].x + normals[off].y*normals[off].y);
        }
    }
    */

    init_stats();
    


    uint16_t *txr_buf = loadmap_get_texture("rd/C1W.tga"); //rd/C1W_512.tga");
    txr = pvr_mem_malloc(TXR_SIZE * TXR_SIZE * 2);
    pvr_txr_load_ex(txr_buf, txr, TXR_SIZE, TXR_SIZE, PVR_TXRLOAD_16BPP);
   

   pvr_poly_cxt_txr(&cxt, PVR_LIST_OP_POLY,
                    PVR_TXRFMT_BUMP | PVR_TXRFMT_TWIDDLED, BUMP_SIZE, BUMP_SIZE, bump,
                    PVR_FILTER_NONE);
   cxt.gen.specular = PVR_SPECULAR_ENABLE;
   cxt.txr.env = PVR_TXRENV_DECAL;
   cxt.gen.culling = PVR_CULLING_CW;
   pvr_poly_compile(&norm_map_hdr, &cxt);

    
    // textured header
    pvr_poly_cxt_txr(&cxt, PVR_LIST_PT_POLY,
                     PVR_TXRFMT_RGB565 | PVR_TXRFMT_TWIDDLED,
                     TXR_SIZE, TXR_SIZE, txr, PVR_FILTER_BILINEAR);
    //cxt.blend.src = PVR_BLEND_DESTCOLOR;
    //cxt.blend.dst = PVR_BLEND_ZERO;
    // ^ for bump map blending
    cxt.blend.src = PVR_BLEND_ONE;
    cxt.blend.dst = PVR_BLEND_ZERO;

    cxt.gen.shading = PVR_SHADE_GOURAUD;
    cxt.gen.culling = PVR_CULLING_CW;
    pvr_poly_compile(&strip_hdr, &cxt);

    // solid color header
    pvr_sprite_cxt_col(&line_cxt, PVR_LIST_PT_POLY);
    //cxt.blend.src = PVR_BLEND_DESTCOLOR;
    //cxt.blend.dst = PVR_BLEND_ZERO;
    // ^ for bump map blending
    cxt.blend.src = PVR_BLEND_ONE;
    cxt.blend.dst = PVR_BLEND_ZERO;
    cxt.gen.culling = PVR_CULLING_NONE;
    cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;

    line_hdr.argb = 0xFFFFFFFF;
    line_hdr.oargb = 0;
    pvr_sprite_compile(&line_hdr, &line_cxt);


    pvr_sprite_cxt_col(&red_line_cxt, PVR_LIST_PT_POLY);
    //cxt.blend.src = PVR_BLEND_DESTCOLOR;
    //cxt.blend.dst = PVR_BLEND_ZERO;
    // ^ for bump map blending
    cxt.blend.src = PVR_BLEND_ONE;
    cxt.blend.dst = PVR_BLEND_ZERO;
    cxt.gen.culling = PVR_CULLING_NONE;
    cxt.depth.comparison = PVR_DEPTHCMP_ALWAYS;


    red_line_hdr.argb = 0xFFFF0000;
    red_line_hdr.oargb = 0;
    pvr_sprite_compile(&red_line_hdr, &red_line_cxt);
    red_line_hdr.argb = 0xFFFF0000;
    red_line_hdr.oargb = 0;




}




float lerp(float a, float b, float f) {
    return a * (1.0 - f) + (b * f);
}

vec3f_t vec3_add(vec3f_t a, vec3f_t b) {
    vec3f_t c;
    c.x = a.x + b.y;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

vec3f_t vec3_div_scalar(vec3f_t a, float b) {
    vec3f_t c;
    c.x = a.x / b;
    c.y = a.y / b;
    c.z = a.z / b;
    return c;
}

int check_start(void) {
    MAPLE_FOREACH_BEGIN(MAPLE_FUNC_CONTROLLER, cont_state_t, st)

    if(st->buttons & CONT_START) {
        return 1;
    }

    MAPLE_FOREACH_END()
    return 0;
}

float speed = 0.2f;

void move_forward(float scalar) {
    float dx = cos(camera.angle);
    float dz = -sin(camera.angle);
    float dy = -sin(camera.pitch_ang);

    camera.x += dx * scalar;
    camera.y += dz * scalar;
    camera.height += dy * scalar;
}

static int process_input(void) {
    maple_device_t *cont;
    cont_state_t *state;
    
    cont = maple_enum_type(0, MAPLE_FUNC_CONTROLLER);

    // Check key status
    state = (cont_state_t *)maple_dev_status(cont);

    if(!state) {
        printf("Error reading controller\n");
        return -1;
    }
    static int last_start = 0;
    if(state->buttons & CONT_START) {
        if(!last_start) {
            toggle_stats();
            last_start = 1;
        }
    } else {
        last_start = 0;
    }
    if(state->buttons & CONT_A){
        camera.pitch_ang += 0.02f;
    }
    if(state->buttons & CONT_B){
        camera.pitch_ang -= 0.02f;
    }
    if((state->buttons & CONT_X)) {
        camera.height++;
    }
    if((state->buttons & CONT_Y)) {
            camera.height--;
    }
    if(state->buttons & CONT_DPAD_UP){
        speed = min(speed+0.01f, 1.0f);

        //float dx = cos(camera.angle);
        //float dz = -sin(camera.angle);
        //float dy = -sin(camera.pitch_ang);

        //camera.x += dx;
        //camera.y += dz;
        //camera.height += dy;
        
        //camera.x += cos(camera.angle);
        //camera.y -= sin(camera.angle);
    }
    if(state->buttons & CONT_DPAD_DOWN){
        speed = max(0.0f, speed-0.01f);
        
        //float dx = cos(camera.angle);
        //float dz = -sin(camera.angle);
        //float dy = -sin(camera.pitch_ang);

        //camera.x -= dx;
        //camera.y -= dz;
        //camera.height -= dy;


        //camera.x -= cos(camera.angle);
        //camera.y += sin(camera.angle);
    }
    if(state->buttons & CONT_DPAD_RIGHT){
        camera.angle -= 0.02;
        camera.angle = camera.angle >= 0.0 ? fmod(camera.angle, 6.28) : 6.28 - fabs(fmod(camera.angle, 6.28));
    }
    if(state->buttons & CONT_DPAD_LEFT){
        camera.angle = fmod((camera.angle + 0.02), (6.28));
    }
    //static int last_l = 0;
    if(state->ltrig) { //} && !last_l){
        //~ return 0;
        camera.roll_ang -= 0.02f;
        //enable_gouraud_lighting = !enable_gouraud_lighting;
    }
    //last_l = state->ltrig;
    
    //static int last_r = 0;
    if(state->rtrig) { //} && !last_r){
        //enable_bump = !enable_bump;
        camera.roll_ang += 0.02f;
    }
    //last_r = state->rtrig;

    

    // joystick is 0-256
    // lets remap it
    if(state->joyx < 0) {
        float droll = lerp(0.0, 0.03, (float)(state->joyx/-128.0f));

        camera.roll_ang += droll;

    } else if (state->joyx > 0) {
        float droll = lerp(0.0, 0.03, (float)(state->joyx/128.0f));

        camera.roll_ang -= droll;
    }

    if(state->joyy < 0) {
        float dpull = lerp(0.0, 0.02, (float)(state->joyy/-128.0f));
        float dpitch = dpull * cos(camera.roll_ang);
        float dang = dpull * sin(camera.roll_ang);

        camera.pitch_ang -= dpitch;
        camera.angle -= dang;

    } else if (state->joyy > 0) {
        float dpull = lerp(0.0, 0.02, (float)(state->joyy/128.0f));
        float dpitch = dpull * cos(camera.roll_ang);
        float dang = dpull * sin(camera.roll_ang);

        camera.pitch_ang += dpitch;
        camera.angle += dang;
    }

    //if(state->joyy < -64) {
    //    bumpiness = CLAMP(0.0f, 1.0f, bumpiness - 0.01f);
    //} else if(state->joyy > 64) {
    //    bumpiness = CLAMP(0.0f, 1.0f, bumpiness + 0.01f);
    //}
    return 1;


}



static inline float min_float(float a, float b) { return a < b ? a : b; }

static inline float max_float(float a, float b) { return a > b ? a : b; }




#define SCALE_FACTOR (70.0)
#define CONST1 320.0f
#define ASPECT_RATIO (640.0f/480.0f)
#define CONST2 (-1.619f * 1 * 320.0f)

#define CONST3 240.0f
#define CONST4 (ASPECT_RATIO * 1 * 240.0f)

#define MAX_POLYS_PER_PATCH 32
#define MAX_VERTS_PER_PATCH 33


int num_polys_lod[5] = {
    32,16,16,8,8
};


uint32_t lod_cols[5] = {
    0xff00ced1,
    0xffffa500,
    0xff00ff00,
    0xff0000ff,
    0xffff1493
};

float xs[MAX_VERTS_PER_PATCH*MAX_VERTS_PER_PATCH];
float ys[MAX_VERTS_PER_PATCH*MAX_VERTS_PER_PATCH];
float us[MAX_VERTS_PER_PATCH*MAX_VERTS_PER_PATCH];
float vs[MAX_VERTS_PER_PATCH*MAX_VERTS_PER_PATCH];
//float zs[NUM_STEPS*NUM_STEPS];
float ws[MAX_VERTS_PER_PATCH*MAX_VERTS_PER_PATCH];
uint32_t argbs[MAX_VERTS_PER_PATCH*MAX_VERTS_PER_PATCH];
uint32_t bumpy[MAX_VERTS_PER_PATCH*MAX_VERTS_PER_PATCH];




void submit_tri(pvr_vertex_t* vert, pvr_dr_state_t dr_state,
                 float ax, float ay, float aw, float au, float av, uint32_t acol, uint32_t aocol,
                 float bx, float by, float bw, float bu, float bv, uint32_t bcol, uint32_t bocol,
                 float cx, float cy, float cw, float cu, float cv, uint32_t ccol, uint32_t cocol) {
    
    if((ax < 0.0f && bx < 0.0f && cx < 0.0f) || 
        (ax >= 640.0f && bx >= 640.0f && cx >= 640.0f) ||
        (ay < 0.0f && by < 0.0f && cy < 0.0f) || 
        (ay >= 480.0f && by >= 480.0f && cy >= 480.0f) ||
        (aw >= 1.0f || bw >= 1.0f|| cw >= 1.0f || aw <= 0.0f || bw <= 0.0f  || cw <= 0.0f)) {
    } else {
        vert = pvr_dr_target(dr_state);
        vert->flags = PVR_CMD_VERTEX;
        vert->x = ax;
        vert->y = ay;
        vert->z = aw;
        vert->u = au;
        vert->v = av;
        vert->argb = acol;
        vert->oargb = aocol;
        pvr_dr_commit(vert);
        vert = pvr_dr_target(dr_state);
        vert->flags = PVR_CMD_VERTEX;
        vert->x = bx;
        vert->y = by;
        vert->z = bw;
        vert->u = bu;
        vert->v = bv;
        vert->argb = bcol;
        vert->oargb = bocol;
        pvr_dr_commit(vert);
        vert = pvr_dr_target(dr_state);
        vert->flags = PVR_CMD_VERTEX_EOL;
        vert->x = cx;
        vert->y = cy;
        vert->z = cw;
        vert->u = cu;
        vert->v = cv;
        vert->argb = ccol;
        vert->oargb = cocol;
        pvr_dr_commit(vert);
        drawn_tris++;
    }
}


void submit_quad(pvr_vertex_t* vert, pvr_dr_state_t dr_state,
                 float ax, float ay, float aw, float au, float av, uint32_t acol, uint32_t aocol,
                 float bx, float by, float bw, float bu, float bv, uint32_t bcol, uint32_t bocol,
                 float cx, float cy, float cw, float cu, float cv, uint32_t ccol, uint32_t cocol,
                 float dx, float dy, float dw, float du, float dv, uint32_t dcol, uint32_t docol) {
    
    if((ax < 0.0f && bx < 0.0f && dx < 0.0f && cx < 0.0f) || 
        (ax >= 640.0f && bx >= 640.0f && dx >= 640.0f && cx >= 640.0f) ||
        (ay < 0.0f && by < 0.0f && dy < 0.0f && cy < 0.0f) || 
        (ay >= 480.0f && by >= 480.0f && dy >= 480.0f && cy >= 480.0f) ||
        (aw >= 1.0f || bw >= 1.0f || dw >= 1.0f || cw >= 1.0f || aw <= 0.0f || bw <= 0.0f  || dw <= 0.0f || cw <= 0.0f)) {
    } else {
        vert = pvr_dr_target(dr_state);
        vert->flags = PVR_CMD_VERTEX;
        vert->x = ax;
        vert->y = ay;
        vert->z = aw;
        vert->u = au;
        vert->v = av;
        vert->argb = acol;
        vert->oargb = aocol;
        pvr_dr_commit(vert);
        vert = pvr_dr_target(dr_state);
        vert->flags = PVR_CMD_VERTEX;
        vert->x = bx;
        vert->y = by;
        vert->z = bw;
        vert->u = bu;
        vert->v = bv;
        vert->argb = bcol;
        vert->oargb = bocol;
        pvr_dr_commit(vert);
        vert = pvr_dr_target(dr_state);
        vert->flags = PVR_CMD_VERTEX;
        vert->x = cx;
        vert->y = cy;
        vert->z = cw;
        vert->u = cu;
        vert->v = cv;
        vert->argb = ccol;
        vert->oargb = cocol;
        pvr_dr_commit(vert);
        vert = pvr_dr_target(dr_state);
        vert->flags = PVR_CMD_VERTEX_EOL;
        vert->x = dx;
        vert->y = dy;
        vert->z = dw;
        vert->u = du;
        vert->v = dv;
        vert->argb = dcol;
        vert->oargb = docol;
        pvr_dr_commit(vert);

        drawn_tris += 2;
    }
}


void submit_line(float ax, float ay, float aw,
                 float bx, float by, float bw) {
    if((aw >= 1.0f || bw >= 1.0f || aw <= 0.0f || bw <= 0.0f)) {
        return;
    }
    float dy = by-ay;
    float dx = bx-ax;

    float inv_len = frsqrt(dx*dx+dy*dy);

    float norm_y = 1*dx * inv_len;
    float norm_x = -1*dy* inv_len;


    pvr_sprite_col_t line_spr = {
        .flags = PVR_CMD_VERTEX_EOL,
        .ax = ax, .ay = ay, .az = 1.0f,
        .bx = bx, .by = by, .bz = 1.0f,
        .cx = bx+norm_x, .cy = by+norm_y, .cz = 1.0f,
        .dx = ax+norm_x, .dy = ay+norm_y
    };

    pvr_prim(&line_spr, sizeof(line_spr));
}


int patch_lods[NUM_TOTAL_PATCHES];
int patch_left_side_touches_higher_lod[NUM_TOTAL_PATCHES];
int patch_right_side_touches_higher_lod[NUM_TOTAL_PATCHES];
int patch_top_side_touches_higher_lod[NUM_TOTAL_PATCHES];
int patch_bot_side_touches_higher_lod[NUM_TOTAL_PATCHES];

uint8_t patch_max_height[NUM_PATCHES_DIR*NUM_PATCHES_DIR];

void setup_patches() {
    for(int py = 0; py < NUM_PATCHES_DIR; py++) {


        for(int px = 0; px < NUM_PATCHES_DIR; px++) {

            uint8_t max_height = 0;
            for(int my = py*PATCH_SIZE; my < (py+1)*PATCH_SIZE; my++) {
                for(int mx = px*PATCH_SIZE; mx < (px+1)*PATCH_SIZE; mx++) {
                    max_height = max(max_height, height_map_mips[0][my*MAP_SIZE+mx]);
                }
            }
            patch_max_height[py*NUM_PATCHES_DIR+px] = max_height;
        }
    }
}


void generate_patches() {
    pvr_dr_state_t dr_state;
    pvr_vertex_t* vert = NULL;
    patches_drawn = 0;
    trans_vertices = 0;

    for(int py = 0; py < NUM_PATCHES_DIR; py++) {
        int my = py*PATCH_SIZE;
        int mcy = my+(PATCH_SIZE/2);

        float map_dy = fabs((float)mcy - camera.y);
        int patch_dy = map_dy / PATCH_SIZE;

        for(int px = 0; px < NUM_PATCHES_DIR; px++) {

            int mx = px*PATCH_SIZE;
            int mcx = mx+(PATCH_SIZE/2);

            float map_dx = fabs((float)mcx - camera.x);

            //int actual_len = sqrt(map_dx*map_dx + map_dy*map_dy);
            //int len_in_patches = actual_len / PATCH_SIZE;

            int patch_dx = map_dx / PATCH_SIZE;
            //int lod = len_in_patches/2;
            //if(lod > 4) { lod = 4; }
            int lod = (patch_dy + patch_dx);///2;
            if(lod > 4) { lod = 4; }
            patch_lods[py*NUM_PATCHES_DIR + px] = lod;

        }
    }

    for(int py = 0; py < NUM_PATCHES_DIR; py++) {
        for(int px = 0; px < NUM_PATCHES_DIR; px++) {
            int p_idx = py*NUM_PATCHES_DIR+px;
            int p_lod = patch_lods[p_idx];

            patch_top_side_touches_higher_lod[p_idx] = (py > 0) ? (patch_lods[p_idx-NUM_PATCHES_DIR] > p_lod) : 0;
            patch_bot_side_touches_higher_lod[p_idx] = (py < NUM_PATCHES_DIR-1) ? (patch_lods[p_idx+NUM_PATCHES_DIR] > p_lod) : 0;
            patch_left_side_touches_higher_lod[p_idx] = (px > 0) ? (patch_lods[p_idx-1] > p_lod) : 0;
            patch_right_side_touches_higher_lod[p_idx] = (px < NUM_PATCHES_DIR-1) ? (patch_lods[p_idx+1] > p_lod) : 0;
        }
    }


    //mat_perspective_fov(45.0f, 640.0f / 480.0f, 1.0f, 1024.0f);

    matrix_t patch_culling_matrix, real_matrix;

    /* REAL CAMERA MATRIX */
    mat_identity();
    mat_rotate(0.0f, 0.0f, camera.roll_ang);
    mat_rotate(camera.pitch_ang, 0.0f, 0.0f);
    mat_rotate(0.0f, (camera.angle-1.57f)-3.14159f, 0.0f);
    mat_translate(-camera.x, -camera.height, -camera.y);

    float bbox_coords[8][3];


    float patch_uv_step = 1.0f / (NUM_PATCHES_DIR);

    for(int py = 0; py < NUM_PATCHES_DIR; py++) {
        int my = py*PATCH_SIZE;
        int mdy = my+PATCH_SIZE;


        float init_v = py * patch_uv_step;


        for(int px = 0; px < NUM_PATCHES_DIR; px++) {

            int mx = px*PATCH_SIZE;
            int mrx = mx+PATCH_SIZE;
            
            
            float max_height = 255.0f - (float)(patch_max_height[py*NUM_PATCHES_DIR+px]);

            // translate patch bounding box
            // x and z bounds, ideally we'd have min and max height, but we're going with 0 and 255
            bbox_coords[0][0] = mx;
            bbox_coords[0][1] = max_height;
            bbox_coords[0][2] = my;

            bbox_coords[1][0] = mx;
            bbox_coords[1][1] = max_height;
            bbox_coords[1][2] = mdy;

            bbox_coords[2][0] = mrx;
            bbox_coords[2][1] = max_height;
            bbox_coords[2][2] = my;

            bbox_coords[3][0] = mrx;
            bbox_coords[3][1] = max_height;
            bbox_coords[3][2] = mdy;
            
            bbox_coords[4][0] = mx;
            bbox_coords[4][1] = 255.0f;
            bbox_coords[4][2] = my;

            bbox_coords[5][0] = mx;
            bbox_coords[5][1] = 255.0f;
            bbox_coords[5][2] = mdy;

            bbox_coords[6][0] = mrx;
            bbox_coords[6][1] = 255.0f;
            bbox_coords[6][2] = my;

            bbox_coords[7][0] = mrx;
            bbox_coords[7][1] = 255.0f;
            bbox_coords[7][2] = mdy;
            
            //mat_load(&patch_culling_matrix);

            for(int i = 0; i < 8; i++) {
                float trans_x = bbox_coords[i][0];
                float trans_y = bbox_coords[i][1];
                float trans_z = bbox_coords[i][2];
                mat_trans_single3_nodiv(trans_x, trans_y, trans_z);
                bbox_coords[i][0] = trans_x;
                bbox_coords[i][1] = trans_y;
                bbox_coords[i][2] = trans_z;
            }
            
            int behind_camera = 0;
            for(int i = 0; i < 8; i++) {
                behind_camera += (bbox_coords[i][2] <= 1.0f) ? 1 : 0;
            }


            if(behind_camera == 8) {
                //continue;
                goto after_draw_quads;
            }

            for(int i = 0; i < 8; i++) {
                float trans_z = bbox_coords[i][2];
                //float inv_z = 1.0f / trans_z;
                float inv_z = frsqrt(trans_z*trans_z);
                float trans_x = bbox_coords[i][0];
                float trans_y = bbox_coords[i][1];
                float proj_x = CONST1 + CONST2 * trans_x * inv_z;
                float proj_y = CONST3 + CONST4 * trans_y * inv_z;

                bbox_coords[i][0] = proj_x;
                bbox_coords[i][1] = proj_y;
                bbox_coords[i][2] = inv_z;
            }

            
            int above_screen = 0;
            int left_of_screen = 0;
            int below_screen = 0;
            int right_of_screen = 0;
            for(int i = 0; i < 8; i++) {
                left_of_screen += (bbox_coords[i][0] < 0.0f) ? 1 : 0;
                right_of_screen += (bbox_coords[i][0] >= 640.0f) ? 1 : 0;
                above_screen += (bbox_coords[i][1] < 0.0f) ? 1 : 0;
                below_screen += (bbox_coords[i][1] >= 480.0f) ? 1 : 0;
            }







            if(above_screen == 8 || below_screen == 8  || left_of_screen == 8  || right_of_screen == 8) {
                goto after_draw_quads;
                //continue;
            }
            

            

            float init_u = px * patch_uv_step;
            int patch_idx = py*NUM_PATCHES_DIR+px;
            int lod = patch_lods[patch_idx];
            uint8_t* lod_height_map = height_map_mips[lod];
            int lod_mip_size = mip_sizes[lod];

            int num_polys_one_dir = num_polys_lod[lod];
            int num_verts_one_dir = num_polys_one_dir+1;
            int total_polys = num_polys_one_dir * num_polys_one_dir;

            patches_drawn += 1;
            //float uv_step = patch_uv_step / num_polys_one_dir;

            float texels_per_poly = PATCH_SIZE/num_polys_one_dir;
            float uv_step = (((float)texels_per_poly)/TXR_SIZE);
            
            int idx = 0;
            // texels per poly 4 
            // patch width = 128
            // num polys per dir = 32
            // 0 to 33, that's to produce vertexes, it's fine


            int top_touches_higher_lod = patch_top_side_touches_higher_lod[patch_idx];
            int bot_touches_higher_lod = patch_bot_side_touches_higher_lod[patch_idx];
            int left_touches_higher_lod = patch_left_side_touches_higher_lod[patch_idx];
            int right_touches_higher_lod = patch_right_side_touches_higher_lod[patch_idx];
            
            //mat_load(&real_matrix);

            // TRANSFORM HEIGHTMAP SAMPLES INTO VERTEXES
            for(int y = 0; y < num_verts_one_dir; y++) {
                int map_y = my + y*texels_per_poly;
                int lod_map_y = map_y>>lod;
                //int higher_lod_map_y = lod_map_y>>(lod+1);
                int higher_lod_map_y = map_y>>(lod+1);
                
                float v = init_v + y*uv_step;

                int top_side_higher_lod = (y == 0) && top_touches_higher_lod;
                int bot_side_higher_lod = (y == num_verts_one_dir-1) && bot_touches_higher_lod;

                for(int x = 0; x < num_verts_one_dir; x++) {

                    int left_side_higher_lod = (x==0) && left_touches_higher_lod;
                    int right_side_higher_lod = (x == num_verts_one_dir-1) && right_touches_higher_lod;

                    int map_x = mx + x*texels_per_poly;
                    int lod_map_x = map_x>>lod;
                    int higher_lod_map_x = map_x>>(lod+1);
                    float u = init_u + x*uv_step;
                    
                    //float tlx = (float)map_x - camera.x;
                    //float tly = (float)map_y - camera.y;
                    //float trans_x = (tlx * sinang) - (tly * cosang);
                    //float trans_z = (tlx * cosang) + (tly * sinang);

                    uint8_t map_height_val;
                    if((left_side_higher_lod || right_side_higher_lod || top_side_higher_lod || bot_side_higher_lod)) {
                        int higher_lod_mip_size = mip_sizes[lod+1];
                        uint8_t* higher_lod_height_map = height_map_mips[lod+1];
                        map_height_val = higher_lod_height_map[higher_lod_map_y*higher_lod_mip_size + higher_lod_map_x];
                    } else {
                        map_height_val = lod_height_map[lod_map_y * lod_mip_size + lod_map_x];
                    }

                    map_height_val = height_map_mips[0][map_y*MAP_SIZE + map_x];

                    //float height = (float)(height_map[get_map_offset(map_x, map_y)]) * 2.0f;
                    float height = 255.0f - map_height_val;
                    float trans_x = map_x;
                    float trans_y = height;
                    float trans_z = map_y;
                    mat_trans_single3_nodiv(trans_x, trans_y, trans_z);

                    if(trans_z <= 1.0f) {
                        ws[idx++] = 0.0f;
                        continue;
                    }
                    float invz = frsqrt(trans_z*trans_z);
                    //float invz = 1.0 / trans_z;

                    //float proj_y = trans_y * invz * (640.0/480.0) * SCALE_FACTOR;
                    float proj_x = (CONST1 + CONST2 * trans_x * invz);
                    float proj_y = CONST3 + CONST4 * trans_y * invz;
                    

                    xs[idx] = proj_x; //proj_x;
                    ys[idx] = proj_y; //proj_height;
                    us[idx] = u;
                    vs[idx] = v;
                    //argbs[idx] = lod_cols[lod];
                    //ws[idx++] = min_float(65535.0f,
                    //                    max_float(0.0f, (proj_w + 1.0f) / 2.0f * 65535.0f));
                    ws[idx++] = invz; 
                    //ws[idx++] = (proj_w*(1.0f/1023.0f))+(1.0f/1.0f); 
                    //ws[idx++] = (proj_w*(1023.0f))+(1.0f); 

                }
            }
            trans_vertices += idx;


            // generate quads
            uint32_t bump_disabled_oargb = pvr_pack_bump(0.0f, F_PI / 4.0f, 5.0f * F_PI / 6.0f);
            
            pvr_prim(&strip_hdr, sizeof(pvr_poly_hdr_t));
            pvr_dr_init(&dr_state);

            float ax,ay,aw,au,av;
            float bx,by,bw,bu,bv;
            float cx,cy,cw,cu,cv;
            float dx,dy,dw,du,dv;
            uint32_t lod_col = lod_cols[lod];
            uint32_t higher_lod_col = lod_cols[lod+1];
            uint32_t acol,bcol,ccol,dcol;


            // DRAW LANDSCAPE QUADS HERE
            for(int y = 0; y < num_polys_one_dir; y++) {
                int y_off = num_verts_one_dir * y;
                int ny_off = num_verts_one_dir * (y+1);
                int nny_off = num_verts_one_dir * (y+2);

                int top_side_higher_lod = (y == 0) && top_touches_higher_lod;
                int bot_side_higher_lod = (y == num_polys_one_dir-1) && bot_touches_higher_lod;

                for(int x = 0; x < num_polys_one_dir; x++) {
                    int left_side_higher_lod = (x==0) && left_touches_higher_lod;
                    int right_side_higher_lod = (x == num_polys_one_dir-1) && right_touches_higher_lod;
                    
                    int rx = x+1;
                    int rrx = x+2;

                    acol = lod_col; //argbs[y_off+x];
                    bcol = lod_col; //argbs[ny+x];
                    dcol = lod_col; //argbs[y_off+rx];
                    ccol = lod_col; //argbs[ny+rx];

                    ax = xs[y_off+x];
                    ax = xs[y_off+x];
                    ay = ys[y_off+x];
                    aw = ws[y_off+x];
                    au = us[y_off+x];
                    av = vs[y_off+x];

                    bx = xs[ny_off+x];
                    by = ys[ny_off+x];
                    bw = ws[ny_off+x];
                    bu = us[ny_off+x];
                    bv = vs[ny_off+x];
                    
                    cx = xs[y_off+rx];
                    cy = ys[y_off+rx];
                    cw = ws[y_off+rx];
                    cu = us[y_off+rx];
                    cv = vs[y_off+rx];

                    dx = xs[ny_off+rx];
                    dy = ys[ny_off+rx];
                    dw = ws[ny_off+rx];
                    du = us[ny_off+rx];
                    dv = vs[ny_off+rx];

                    if(0) { //left_side_higher_lod && top_side_higher_lod) {
                        continue;
                        // TODO handle correctly
                    }
                    if(0) { //bot_side_higher_lod && right_side_higher_lod) {
                        // already drawn, overlapping
                        continue;
                    }

                    if(0) { //top_side_higher_lod && left_side_higher_lod) {
                        // needs to be handled specially
                    }

                    if(0) { //top_side_higher_lod) {
                        acol = higher_lod_col;
                        // on the top or bottom, and touching a higher (lower res) lod
                        if(x & 1) {
                            // skip odd vertexes
                            continue;
                        } else {
                            continue;
                            /*
                                A   E 
                                B D F
                            */
                            
                            // d becomes e
                            float ex = xs[y_off+rrx];
                            float ey = ys[y_off+rrx];
                            float ew = ws[y_off+rrx];
                            float eu = us[y_off+rrx];
                            float ev = vs[y_off+rrx];
                            float fx = xs[ny_off+rrx];
                            float fy = ys[ny_off+rrx];
                            float fw = ws[ny_off+rrx];
                            float fu = us[ny_off+rrx];
                            float fv = vs[ny_off+rrx];
                            //uint32_t ecol = higher_lod_col;
                            //uint32_t fcol = lod_col;
                            submit_quad(
                                vert, dr_state,
                                ax, ay, aw, au, av, 0xFFFFFFFF, bump_disabled_oargb,
                                dx, dy, dw, du, dv, 0xFFFFFFFF, bump_disabled_oargb,
                                ex, ey, ew, eu, ev, 0xFFFFFFFF, bump_disabled_oargb,
                                fx, fy, fw, fu, fv, 0xFFFFFFFF, bump_disabled_oargb
                            );
                            continue;
                        }
                    }

                    if(0) { //left_side_higher_lod) {
                        acol = higher_lod_col;
                        // on the left or right, and touching a higher (lower res) lod
                        if(y & 1) {
                            // skip odd vertexes
                            continue;
                        } else {
                            // BROKEN?!
                            continue;
                            float ex = xs[nny_off+x];
                            float ey = ys[nny_off+x];
                            float ew = ws[nny_off+x];
                            float eu = us[nny_off+x];
                            float ev = vs[nny_off+x];
                            uint32_t ecol = higher_lod_col;
                            float fx = xs[nny_off+rx];
                            float fy = ys[nny_off+rx];
                            float fw = ws[nny_off+rx];
                            float fu = us[nny_off+rx];
                            float fv = vs[nny_off+rx];
                            uint32_t fcol = lod_col;
                            
                            /*
                            submit_tri(
                                vert, dr_state,
                                cx, cy, cw, cu, cv, 0xFFFFFFFF, bump_disabled_oargb,
                                ex, ey, ew, eu, ev, 0xFFFFFFFF, bump_disabled_oargb,
                                fx, fy, fw, fu, fv, 0xFFFFFFFF, bump_disabled_oargb
                            );
                            continue;
                            */

                        }
                    }

                    if(0) { //bot_side_higher_lod) {
                        bcol = higher_lod_col;
                        if(x & 1) {
                            continue;
                        } else {
                           continue;
                            float ex = xs[y_off+rrx];
                            float ey = ys[y_off+rrx];
                            float ew = ws[y_off+rrx];
                            float eu = us[y_off+rrx];
                            float ev = vs[y_off+rrx];
                            float fx = xs[ny_off+rrx];
                            float fy = ys[ny_off+rrx];
                            float fw = ws[ny_off+rrx];
                            float fu = us[ny_off+rrx];
                            float fv = vs[ny_off+rrx];
                            uint32_t ecol = lod_col;
                            uint32_t fcol = higher_lod_col;

                            submit_quad(
                                vert, dr_state,
                                ax, ay, aw, au, av, 0xFFFFFFFF, bump_disabled_oargb,
                                bx, by, bw, bu, bv, 0xFFFFFFFF, bump_disabled_oargb,
                                dx, dy, dw, du, dv, 0xFFFFFFFF, bump_disabled_oargb,
                                fx, fy, fw, fu, fv, 0xFFFFFFFF, bump_disabled_oargb
                            );
                            submit_tri(
                                vert, dr_state,
                                dx, dy, dw, du, dv, 0xFFFFFFFF, bump_disabled_oargb,
                                fx, fy, fw, fu, fv, 0xFFFFFFFF, bump_disabled_oargb,
                                ex, ey, ew, eu, ev, 0xFFFFFFFF, bump_disabled_oargb
                            );
                            continue;
                        }
                    }

                    if(0) { //right_side_higher_lod) {
                        dcol = higher_lod_col;
                        // on the left or right, and touching a higher (lower res) lod
                        if(y & 1) {
                            // skip odd vertexes
                            continue;
                        } else {
                            continue;
                            // BROKEN!!
                            float ex = xs[nny_off+x];
                            float ey = ys[nny_off+x];
                            float ew = ws[nny_off+x];
                            float eu = us[nny_off+x];
                            float ev = vs[nny_off+x];
                            //uint32_t ecol = lod_col;
                            float fx = xs[nny_off+rx];
                            float fy = ys[nny_off+rx];
                            float fw = ws[nny_off+rx];
                            float fu = us[nny_off+rx];
                            float fv = vs[nny_off+rx];
                            //uint32_t fcol = higher_lod_col;

                           
                            submit_quad(
                                vert, dr_state,
                                dx, dy, dw, du, dv, 0xFFFFFFFF, bump_disabled_oargb,
                                bx, by, bw, bu, bv, 0xFFFFFFFF, bump_disabled_oargb,
                                fx, fy, fw, fu, fv, 0xFFFFFFFF, bump_disabled_oargb,
                                ex, ey, ew, eu, ev, 0xFFFFFFFF, bump_disabled_oargb
                            );
                            submit_tri(
                                vert, dr_state,
                                dx, dy, dw, du, dv, 0xFFFFFFFF, bump_disabled_oargb,
                                ax, ay, aw, au, av, 0xFFFFFFFF, bump_disabled_oargb,
                                bx, by, bw, bu, bv, 0xFFFFFFFF, bump_disabled_oargb
                            );
                            continue;

                        }
                    }

                    
                    submit_quad(vert, dr_state,
                                ax, ay, aw, au, av, 0xFFFFFFFF, bump_disabled_oargb,
                                bx, by, bw, bu, bv, 0xFFFFFFFF, bump_disabled_oargb,
                                cx, cy, cw, cu, cv, 0xFFFFFFFF, bump_disabled_oargb,
                                dx, dy, dw, du, dv, 0xFFFFFFFF, bump_disabled_oargb);


                }
            }
            
            pvr_dr_finish();

            if(enable_debug) {
                if(behind_camera || left_of_screen || right_of_screen || above_screen || below_screen) {
                    pvr_prim(&red_line_hdr, sizeof(red_line_hdr));
                } else {
                    pvr_prim(&line_hdr, sizeof(line_hdr));
                }
                
                submit_line(
                    bbox_coords[0][0], bbox_coords[0][1], bbox_coords[0][2], 
                    bbox_coords[1][0], bbox_coords[1][1], bbox_coords[1][2]
                );
                submit_line(
                    bbox_coords[1][0], bbox_coords[1][1], bbox_coords[1][2], 
                    bbox_coords[3][0], bbox_coords[3][1], bbox_coords[3][2]
                );
                submit_line(
                    bbox_coords[3][0], bbox_coords[3][1], bbox_coords[3][2], 
                    bbox_coords[2][0], bbox_coords[2][1], bbox_coords[2][2]
                );
                submit_line(
                    bbox_coords[2][0], bbox_coords[2][1], bbox_coords[2][2], 
                    bbox_coords[0][0], bbox_coords[0][1], bbox_coords[0][2]
                );
                
                submit_line(
                    bbox_coords[4+0][0], bbox_coords[4+0][1], bbox_coords[4+0][2], 
                    bbox_coords[4+1][0], bbox_coords[4+1][1], bbox_coords[4+1][2]
                );
                submit_line(
                    bbox_coords[4+1][0], bbox_coords[4+1][1], bbox_coords[4+1][2], 
                    bbox_coords[4+3][0], bbox_coords[4+3][1], bbox_coords[4+3][2]
                );
                submit_line(
                    bbox_coords[4+3][0], bbox_coords[4+3][1], bbox_coords[4+3][2], 
                    bbox_coords[4+2][0], bbox_coords[4+2][1], bbox_coords[4+2][2]
                );
                submit_line(
                    bbox_coords[4+2][0], bbox_coords[4+2][1], bbox_coords[4+2][2], 
                    bbox_coords[4+0][0], bbox_coords[4+0][1], bbox_coords[4+0][2]
                );

                
                
                submit_line(
                    bbox_coords[0][0], bbox_coords[0][1], bbox_coords[0][2], 
                    bbox_coords[4+0][0], bbox_coords[4+0][1], bbox_coords[4+0][2]
                );
                submit_line(
                    bbox_coords[1][0], bbox_coords[1][1], bbox_coords[1][2], 
                    bbox_coords[4+1][0], bbox_coords[4+1][1], bbox_coords[4+1][2]
                );
                submit_line(
                    bbox_coords[2][0], bbox_coords[2][1], bbox_coords[2][2], 
                    bbox_coords[4+2][0], bbox_coords[4+2][1], bbox_coords[4+2][2]
                );
                submit_line(
                    bbox_coords[3][0], bbox_coords[3][1], bbox_coords[3][2], 
                    bbox_coords[4+3][0], bbox_coords[4+3][1], bbox_coords[4+3][2]
                );
            }
        after_draw_quads:
            
        }
    }
}


void calculate_lighting(vec3f_t light_vector, float ambient) {

    /*
    for(int y = 0; y < NUM_STEPS; y++) {
        int map_y = y*PX_STEP;
        

        for(int x = 0; x < NUM_STEPS; x++) {
            int map_x = x*PX_STEP;


            int off = y*NUM_STEPS+x;
            //float z = zs[off];
            float w = ws[off];
            if (w >= 1.0f) {

            } else {

                vec3f_t norm = avg_normals[off];
                float intensity = fipr(light_vector.x, light_vector.y, light_vector.z, 0.0f, norm.x, norm.y, norm.z, 0.0f);
                intensity = min(max(0.0f, intensity+ambient), 1.0f);
                uint8_t scaled_intensity = 255.0f * intensity;
                argbs[off] = 0xFFFFFFFF;
                //argbs[off] = (0xFF << 24) | (scaled_intensity << 16) | (scaled_intensity << 8) | (scaled_intensity << 0);
            
                //float avg_dist = z;//(az + bz + cz + dz) / 4.0f;
                float bumpiness = 0.5f; //lerp(0.0f, 1.0f, min(z,256.0f)/256.0f);
                uint32_t bump_oargb = pvr_pack_bump(bumpiness, F_PI / 4.0f, 5.0f * F_PI / 6.0f);
                bumpy[off] = bump_oargb;
            }
        }
    }
    */
}


vector_t vector_cross(vector_t v1, vector_t v2) {
    vector_t c;
    c.x = v1.y * v2.z - v1.z * v2.y;
    c.y = v1.x * v2.z - v1.z * v2.x;
    c.z = v1.x * v2.y - v1.y * v2.x;
    c.w = 0;
    return c;
}


float sun_angle = 1.57f;


static void do_frame(int frame_cnt) {
    drawn_tris = 0;
    float light_x = 0.0f;//sin(camera.angle);
    float light_y = sin(sun_angle);
    float light_z = cos(sun_angle);

    vec3f_normalize(light_x, light_y, light_z);
    vec3f_t light_vector = {.x = -light_x, .y = -light_y, .z = -light_z};


    pvr_wait_ready();
    pvr_scene_begin();


    pvr_list_begin(PVR_LIST_PT_POLY);


    generate_patches();



    //pvr_list_begin(PVR_LIST_OP_POLY);
    //pvr_prim(&norm_map_hdr, sizeof(pvr_poly_hdr_t));
    //draw_landscape_quads(1);
    //pvr_list_finish();

    //draw_landscape_quads(0);

    draw_stats();



    pvr_list_finish();

    pvr_scene_finish();

    
}

static pvr_init_params_t pvr_params = {
    /* Enable only opaque and punchthru polygons. */
    {
        // opaque       opaque modifier, transparent,   transparent_modifier, punch-thru
        PVR_BINSIZE_16, PVR_BINSIZE_0,   PVR_BINSIZE_0, PVR_BINSIZE_0,        PVR_BINSIZE_16
    },
    512 * 1024
};

extern uint8 romdisk_boot[];
KOS_INIT_ROMDISK(romdisk_boot);

int num_frames = 0;

int main(int argc, char *argv[]) {


    srand(time(NULL));

    pvr_init(&pvr_params);

    setup();
    setup_patches();


    /* Go as long as the user hasn't pressed start on controller 1. */
    while(1) {    
        //FPS Counter

        update_stats();


        //Process controller input
        process_input();
        
        move_forward(speed);

        //sun_angle += 0.004f;
        if(sun_angle >= 6.283f) {
            sun_angle -= 6.283f;
        }

        float rads_from_direct_vertical = min(3.14159f, fabs(sun_angle - 1.57f));
        float sky_blue_r = 53.0f/255.0f;
        float sky_blue_g = 81.0f/255.0f;
        float sky_blue_b = 92.0f/255.0f;
        float midnight_blue_r = 1.0f/255.0f;
        float midnight_blue_g = 0.0f/255.0f;
        float midnight_blue_b = 26.0f/255.0f;

        float sky_r = lerp(midnight_blue_r, sky_blue_r, 1.0f-(rads_from_direct_vertical/3.14159f));
        float sky_g = lerp(midnight_blue_g, sky_blue_g, 1.0f-(rads_from_direct_vertical/3.14159f));
        float sky_b = lerp(midnight_blue_b, sky_blue_b, 1.0f-(rads_from_direct_vertical/3.14159f));

        pvr_set_bg_color(sky_r, sky_g, sky_b);

        do_frame(num_frames++);
    }


    return 0;
}
