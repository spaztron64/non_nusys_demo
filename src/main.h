#pragma once

//Defines amount of memory for local variables in a thread
#define MAIN_STACK_SIZE 0x2000
#define IDLE_STACK_SIZE 0x800
#define GFX_STACK_SIZE 0x2000

//Define thread priorities
//These define which threads are most critical to run
//These should all be less than 127 since anything greater is reserved
#define MAIN_PRIORITY 10
#define GFX_PRIORITY 50
#define AUDIO_PRIORITY 70
#define SCHED_PRIORITY 120

#define NUM_GFX_MSGS 8
#define NUM_PI_MSGS 16
#define NUM_SI_MSGS 4

//Define resolution modes
typedef enum {
	RES_320x240,
	RES_640x480,
	RES_MODE_COUNT
} ResolutionMode;

//Resolution parameters structure
typedef struct {
	u16 width;
	u16 height;
	u16 vi_mode;
} ResolutionParams;

//Maximum resolution for buffer allocation
#define MAX_SCREEN_WD 640
#define MAX_SCREEN_HT 480

//Place framebuffers at end of RAM
#define ZBUF_ADDR (0x80800000-(MAX_SCREEN_WD*MAX_SCREEN_HT*2*3))
#define CFB1_ADDR (0x80800000-(MAX_SCREEN_WD*MAX_SCREEN_HT*2*2))
#define CFB2_ADDR (0x80800000-(MAX_SCREEN_WD*MAX_SCREEN_HT*2))

//Define background buffer size
#define BG_BUF_SIZE	600000
#define BG_BUF_ADDR (ZBUF_ADDR-BG_BUF_SIZE)

//Size of display list buffer
#define GLIST_SIZE 16384

//Size of RSP FIFO for F3DEX2
#define FIFO_SIZE 8192

//Square Render Properties
#define SQUARE_SIZE 32
#define SQUARE_BORDER_W 2

//Define background stride
#define     BACK_WD       	640
#define     BACK_HT       	440
#define     ROWS          	2
#define     DELTA         	1

//Audio heap definitions
//Placed immediately before framebuffers in this demo
#define AUDIO_HEAP_SIZE 0x60000
#define AUDIO_HEAP_ADDR (BG_BUF_ADDR-AUDIO_HEAP_SIZE)

//Define Size of audio data buffers
#define PTR_BUF_SIZE 8192
#define TUNE_BUF_SIZE 16384

//Statically allocate sound buffers due to... I guess codesegment size overruns?
#define PTR_BUF_ADDR (AUDIO_HEAP_ADDR-PTR_BUF_SIZE)
#define TUNE_BUF_ADDR (PTR_BUF_ADDR-TUNE_BUF_SIZE)

//Define maximum chunk size for DMAs
//Used to prevent clashes with other DMAs
//Since the PI only supports one DMA at a time
#define DMA_BLOCK_SIZE 16384

//Square movement parameters
#define STICK_DEADZONE 10
#define SQUARE_VELOCITY_SCALE (1/24.0f)

//Game state structure shared between threads
typedef struct {
	float square_pos_x;
	float square_pos_y;
	ResolutionMode resolution_mode;
	u8 resolution_changed;
} GameState;

//Expose audio data pointer to program
extern u8 pbank_start[];
extern u8 pbank_end[];
extern u8 wbank_start[];
extern u8 wbank_end[];
extern u8 sng_menu_start[];
extern u8 sng_menu_end[];

//Background data segments
extern u8 _background_dataSegmentRomStart[];
extern u8 _background_dataSegmentRomEnd[];

//Model data segments
extern u8 _model_dataSegmentRomStart[];
extern u8 _model_dataSegmentRomEnd[];