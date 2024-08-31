
#include "common.h"
#include <dc/pvr.h>

#define STATS_TXR_WIDTH 256
#define STATS_TXR_HEIGHT 128

static pvr_sprite_hdr_t stats_hdr;

static uint16_t* stats_txr_buf;
static pvr_ptr_t stats_txr;

uint64_t start_time;
static float fps = 0.0f;


void init_stats() {
    pvr_sprite_cxt_t spr_cxt;

    stats_txr_buf = malloc(2 * STATS_TXR_WIDTH * STATS_TXR_HEIGHT);
    bfont_set_foreground_color(0xFFFFFFFF);

    stats_txr = pvr_mem_malloc(STATS_TXR_WIDTH * STATS_TXR_HEIGHT * 2);

    
    pvr_sprite_cxt_txr(&spr_cxt, PVR_LIST_PT_POLY,
                     PVR_TXRFMT_RGB565 | PVR_TXRFMT_NONTWIDDLED,
                     STATS_TXR_WIDTH, STATS_TXR_HEIGHT, stats_txr, PVR_FILTER_BILINEAR);
    //spr_cxt.blend.src_enable = PVR_BLEND_ENABLE;
    //spr_cxt.blend.dst_enable = PVR_BLEND_ENABLE;
    //spr_cxt.blend.src = PVR_BLEND_SRCALPHA;
    //spr_cxt.blend.dst = PVR_BLEND_INVSRCALPHA;
    spr_cxt.gen.culling = PVR_CULLING_NONE;
    pvr_sprite_compile(&stats_hdr, &spr_cxt);

    start_time = timer_ms_gettime64();
}

void toggle_stats() {
    enable_debug++;
    if(enable_debug > 2) { enable_debug = 0; }
}

static char buf[32];

static int fps_frame_counter = 0;
static int num_frames = 0;
static uint32 current_time = 0;

void update_stats(pvr_ptr_t stats_txr, uint16_t* stats_txr_buf) {
    current_time = timer_ms_gettime64();
    if((current_time - start_time) > 1000) {
        fps = 1000.0 * (float)fps_frame_counter / (float)(current_time - start_time);
        fps_frame_counter = 0;
        start_time = current_time;
    }
    fps_frame_counter++;
    num_frames++;

    if(enable_debug == 1) {
        
        memset2(stats_txr_buf, 0x00000000, STATS_TXR_WIDTH*STATS_TXR_HEIGHT*2);

        int y = 0;
        sprintf(buf, "fps: %.2f", fps);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "vram avail: %li", pvr_mem_available());
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "patches culled: %i/%i", NUM_TOTAL_PATCHES-patches_drawn, NUM_TOTAL_PATCHES);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "transf. verts: %i", trans_vertices);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y+= 20;

        sprintf(buf, "drawn tris: %i", drawn_tris);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        pvr_txr_load(stats_txr_buf, stats_txr, STATS_TXR_WIDTH*STATS_TXR_WIDTH*2);

    } else if (enable_debug == 2) {

        memset2(stats_txr_buf, 0x00000000, STATS_TXR_WIDTH*STATS_TXR_HEIGHT*2);
        int y = 0;
        sprintf(buf, "x: %f", camera.x);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "y: %f", camera.height);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "z: %f", camera.y);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "y: %f", camera.angle);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "p: %f", camera.roll_ang);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;

        sprintf(buf, "r: %f", camera.roll_ang);
        bfont_draw_str(stats_txr_buf+y*STATS_TXR_WIDTH, STATS_TXR_WIDTH, 0, buf);
        y += 20;


        pvr_txr_load(stats_txr_buf, stats_txr, STATS_TXR_WIDTH*STATS_TXR_WIDTH*2);

    }
}

void draw_stats() {
    if(enable_debug) {

        pvr_prim(&stats_hdr, sizeof(stats_hdr));
        pvr_sprite_txr_t spr = {
            .flags = PVR_CMD_VERTEX_EOL,
            .ax = 0.0f, .ay = 0.0f, .az = 1.0f,
            .bx = STATS_TXR_WIDTH, .by = 0.0f, .bz = 1.0f,
            .cx = STATS_TXR_WIDTH, .cy = STATS_TXR_HEIGHT, .cz = 1.0f,
            .dx = 0.0f, .dy = STATS_TXR_HEIGHT,
            .auv = PVR_PACK_16BIT_UV(0.0f, 0.0f),
            .buv = PVR_PACK_16BIT_UV(1.0f, 0.0f),
            .cuv = PVR_PACK_16BIT_UV(1.0f, 1.0f),
        };
        pvr_prim(&spr, sizeof(spr));
    }
}