#include <ultra64.h>
#include <PR/sched.h>
#include <PR/libmus.h>
#include <math.h>
#include "main.h"
#include "resources.h"

//Define stacks
//Main stack must not be static to export to entrypoint code
u64 main_stack[MAIN_STACK_SIZE/8];
static u64 idle_stack[IDLE_STACK_SIZE/8];
static u64 gfx_stack[GFX_STACK_SIZE/8];
static u64 sched_stack[OS_SC_STACKSIZE/8];

//Define OSSched scheduler instance
static OSSched scheduler;

//Define threads
static OSThread idle_thread;
static OSThread main_thread;
static OSThread gfx_thread;

//Define message queues and message buffers
static OSMesgQueue pi_msg_queue;
static OSMesg pi_msgs[NUM_PI_MSGS];
static OSMesgQueue gfx_msg_queue;
static OSMesg gfx_msgs[NUM_GFX_MSGS];
static OSMesgQueue si_msg_queue;
static OSMesg si_msgs[NUM_SI_MSGS];
static OSMesgQueue game_msg_queue;
static OSMesg game_msgs[8];

//Define PI Handle
//The handle is used to read from ROM with the EPI interface
static OSPiHandle *cart_handle;

//Define display list buffer
static Gfx glist[GLIST_SIZE];

//Define 16-Byte aligned F3DEX2 buffers
//Define F3DEX2 matrix stack variable
static u64 dram_stack[SP_DRAM_STACK_SIZE64] __attribute__((aligned(16)));
//Buffer to hold results of F3DEX2 display list processing
static u64 fifo_buf[FIFO_SIZE/8] __attribute__((aligned(16)));
//Buffer to allow interruption of F3DEX2 by audio microcode
static u64 yield_buf[OS_YIELD_DATA_SIZE/8] __attribute__((aligned(16)));

//static u64 bgbuf[PTR_BUF_SIZE] __attribute__((aligned(16)));
u64* bgbuf = BG_BUF_ADDR;
u64* model_buf = 0x80400000;

//Define libmus audio data buffers
//static u8 ptr_buf[PTR_BUF_SIZE*2] __attribute__((aligned(4)));
//static u8 tune_buf[TUNE_BUF_SIZE] __attribute__((aligned(4)));
//But do it statically this time because they crash now. Code segment overrun?
static u8* ptr_buf = PTR_BUF_ADDR;
static u8* tune_buf = TUNE_BUF_ADDR;

//Store controller data
static OSContStatus cont_statuses[MAXCONTROLLERS];
static OSContPad cont_data[MAXCONTROLLERS];

//Shared game state
static GameState game_state;

//Resolution lookup table
static const ResolutionParams resolution_table[RES_MODE_COUNT] = {
	{ 320, 240, OS_VI_NTSC_LAN1 },
	{ 640, 480, OS_VI_NTSC_HAN1 }
};

u16 screen_wd, screen_ht;

static void idle(void *arg);
static void main(void *arg);
static void graphics(void *arg);

extern void Debug_Print(Gfx **glistp_ptr, u16 x, u16 y, char* str);

// To use it, call print_fps(x,y); every frame.
#define FRAMETIME_COUNT 30

OSTime frameTimes[FRAMETIME_COUNT];
u8 curFrameTimeIndex = 0;
u32 sCPU = 0;
f32 gFPS = 0;

// Call once per frame
f32 calculate_and_update_fps() {
    OSTime newTime = osGetTime();
    OSTime oldTime = frameTimes[curFrameTimeIndex];
    frameTimes[curFrameTimeIndex] = newTime;

    curFrameTimeIndex++;
    if (curFrameTimeIndex >= FRAMETIME_COUNT) {
        curFrameTimeIndex = 0;
    }
    gFPS = ((f32)FRAMETIME_COUNT * 1000000.0f) / (s32)OS_CYCLES_TO_USEC(newTime - oldTime);
}

u32 rsp_taskstart, rsp_taskend, rsp_tasktime = 0;
char fpsbuf[40];

//Start up game
//This function must not be static to be seen by entrypoint code
void boot()
{
	//Start up OS
	osInitialize();
	//Start up an idle thread
	osCreateThread(&idle_thread, 1, idle, NULL, &idle_stack[IDLE_STACK_SIZE/8], MAIN_PRIORITY);
	osStartThread(&idle_thread);
}

static void idle(void *arg)
{
	//Initialize PI to read cart
	osCreatePiManager(OS_PRIORITY_PIMGR, &pi_msg_queue, pi_msgs, NUM_PI_MSGS);
	cart_handle = osCartRomInit();
	//Start up main thread
	osCreateThread(&main_thread, 3, main, NULL, &main_stack[MAIN_STACK_SIZE/8], MAIN_PRIORITY);
	osStartThread(&main_thread);
	//Make this the idle thread
	osSetThreadPri(NULL, 0);
	//Busy wait
	while(1);
}

static void ReadRom(u32 src, void *dst, u32 size)
{
	OSIoMesg io_mesg;
    OSMesgQueue dma_msg_queue;
    OSMesg dma_msg;
    u32 read_size;
	//Use alternative variable for dst
	u8 *dma_dest = (u8 *)dst;
	
	//Initialize DMA message queue for waiting for DMA to be done
    osCreateMesgQueue(&dma_msg_queue, &dma_msg, 1);
	
	//Setup static parameters in OSIoMesg for ROM read
    io_mesg.hdr.pri = OS_MESG_PRI_NORMAL;
    io_mesg.hdr.retQueue = &dma_msg_queue;
	
	//Invalidate data cache DMA will write to
    osInvalDCache(dma_dest, size);
	
	//DMA in chunks
    while(size){
		//Determine read size based off the size left to read
		if(size > DMA_BLOCK_SIZE){
			//Cap read size if size is greater than block size
			read_size = DMA_BLOCK_SIZE;
		} else {
			//Read size is identical to size remaining
			read_size = size;
		}
		//Setup dynamic parameters in OSIoMesg for ROM read
		io_mesg.dramAddr = dma_dest;
		io_mesg.devAddr = src;
		io_mesg.size = read_size;
		
		//Start reading from ROM
		osEPiStartDma(cart_handle, &io_mesg, OS_READ);
		//Wait for ROM Read to be done
		osRecvMesg(&dma_msg_queue, &dma_msg, OS_MESG_BLOCK);
		
		//Advance pointers for next ROM read
		src += read_size;
		dma_dest += read_size;
		size -= read_size;
    }
}

static void InitAudio()
{
	musConfig config;
	config.control_flag = 0; //Use ROM wavetables
	config.channels = 32; //Decent default for channel count
	config.sched = &scheduler; //Point to existing OSSched scheduler instance
	config.thread_priority = AUDIO_PRIORITY;
	//Setup audio heap
	config.heap = (u8 *)AUDIO_HEAP_ADDR;
	config.heap_length = AUDIO_HEAP_SIZE;
	//Set FIFO length to minimum
	config.fifo_length = 64;
	//Assign no initial audio or FX bank
	config.ptr = NULL;
	config.wbk = NULL;
	config.default_fxbank = NULL;
	//Set audio default frequency
	config.syn_output_rate = 44100;
	//Set synthesizer parameters to sane defaults from nualstl3
	config.syn_updates = 256;
	config.syn_rsp_cmds = 2048;
	config.syn_num_dma_bufs = 64;
	config.syn_dma_buf_size = 1024;
	config.syn_retraceCount = 1; //Must be same as last parameter to osCreateScheduler
	//Initialize libmus
	MusInitialize(&config);
	//Read and initialize song sample bank
	ReadRom((u32)pbank_start, ptr_buf, pbank_end-pbank_start);
    MusPtrBankInitialize(ptr_buf, wbank_start);
	//Read and play back the song
	ReadRom((u32)sng_menu_start, tune_buf, sng_menu_end-sng_menu_start);
	MusStartSong(tune_buf);
}

static void InitController()
{
	//Variable to hold bit pattern of controllers plugged in
	u8 pattern;
	//Initialize SI message queue
	osCreateMesgQueue(&si_msg_queue, si_msgs, NUM_SI_MSGS);
	//Register SI message queue with SI events
	osSetEventMesg(OS_EVENT_SI, &si_msg_queue, NULL);
	//Initialize SI devices
	osContInit(&si_msg_queue, &pattern, cont_statuses);
}

static void ReadController()
{
	//Start controller read
	osContStartReadData(&si_msg_queue);
	//Wait for controller read to finish
	osRecvMesg(&si_msg_queue, NULL, OS_MESG_BLOCK);
	//Get controller read data
	osContGetReadData(cont_data);
}

static void Load_Background(){
	u32 bgseg_start = (u32)_background_dataSegmentRomStart;
	u32 bgseg_size = _background_dataSegmentRomEnd - _background_dataSegmentRomStart;
	ReadRom(bgseg_start, bgbuf, bgseg_size);
}

static void Load_Model(){
	u32 seg_start = (u32)_model_dataSegmentRomStart;
	u32 seg_size = _model_dataSegmentRomEnd - _model_dataSegmentRomStart;
	ReadRom(seg_start, model_buf, seg_size);
}

static void Draw_Background(Gfx **glistp_ptr, u16 scr_w, u16 scr_h)
{

  Gfx *glistp = *glistp_ptr;
  
  int           i;
  static int    posx = 0, delta = DELTA;
  
  gSPSegment(glistp++, 0x02, OS_K0_TO_PHYSICAL(bgbuf));
  if(game_state.resolution_mode == RES_640x480){
	gDPSetCycleType(glistp++, G_CYC_COPY);
  }
  else{
	gDPSetCycleType(glistp++, G_CYC_1CYCLE);
  }
  gDPSetCombineMode(glistp++, G_CC_DECALRGBA, G_CC_DECALRGBA);
  gDPSetRenderMode(glistp++, G_RM_NOOP, G_RM_NOOP2);
  gDPSetTexturePersp(glistp++, G_TP_NONE);
  
  for(i = 20; i < (BACK_HT / ROWS); i++)
  {
    gDPLoadTextureTile(glistp++,
                       higu,
                       G_IM_FMT_RGBA,
                       G_IM_SIZ_16b,
                       BACK_WD,
                       BACK_HT,
                       posx,
                       i * ROWS,
                       posx + (640 - 1),
                       i * ROWS + (ROWS - 1),
                       0,
                       G_TX_WRAP, G_TX_WRAP,
                       0, 0,
                       G_TX_NOLOD, G_TX_NOLOD);
	if(game_state.resolution_mode == RES_640x480){
    gSPTextureRectangle(glistp++,
                        0 << 2,
                        (i * ROWS) << 2,
                        (scr_w - 1) << 2,
                        (i * ROWS + (ROWS - 1)) << 2,
                        G_TX_RENDERTILE,
                        posx << 5, (i * ROWS) << 5,
                        4 << 10, 1 << 10);
	}
	else{
	gSPTextureRectangle(glistp++,
                        0 << 2,
                        (i * ROWS) << 1,
                        (scr_w - 1) << 2,
                        (i * ROWS + (ROWS - 1)) << 1,
                        G_TX_RENDERTILE,
                        posx << 5, (i * ROWS) << 5,
                        2 << 10, 1 << 10);
	}
  }
  posx += delta;
  int view_width = (game_state.resolution_mode == RES_640x480) ? screen_wd : screen_wd * 2;

  if(posx + view_width >= BACK_WD)
  {
    posx = BACK_WD - view_width;
    delta = -DELTA;
  }
  else if(posx <= 0)
  {
    posx = 0;
    delta = DELTA;
  }
  gDPPipeSync(glistp++);
  
  *glistp_ptr = glistp;
}

static void DrawRect(Gfx **glistp_ptr, s32 x, s32 y, s32 w, s32 h, u16 color, u16 screen_wd, u16 screen_ht)
{
	Gfx *glistp = *glistp_ptr;
	
	//Don't draw fully offscreen rectangles
	if(x < -w || y < -h) {
		return;
	}
	//Clamp partially offscreen rectangles to screen
	if(x < 0) {
		w += x;
		x = 0;
	}
	if(y < 0) {
		h += y;
		y = 0;
	}
	//Set rectangle color
	gDPSetFillColor(glistp++, (color << 16)|color);
	//Draw rectangle
	gDPFillRectangle(glistp++, x, y, x+w-1, y+h-1);
	//Wait for rectangle to draw
	gDPPipeSync(glistp++);
	
	*glistp_ptr = glistp;
}

static void graphics(void *arg)
{
	//Information to keep track of using two framebuffers
	int cfb_idx = 0;
	void *cfb_table[2] = { (void *)CFB1_ADDR, (void *)CFB2_ADDR };
	void *depth_buf = (void *)ZBUF_ADDR;
	OSScMsg done_msg; //Hold message for rendering being done
	OSScMsg *wait_msg; //Point to received messages
	OSScClient client; //Used for frame synchronization
	OSScTask sc_task; //Graphics task information
	Gfx *glistp;
	ResolutionMode current_res_mode;
	
	//Wait for scheduler to be initialized
	osRecvMesg(&game_msg_queue, NULL, OS_MESG_BLOCK);
	
	done_msg.type = OS_SC_DONE_MSG; //Register done message
	//Initialize graphics messages
	osCreateViManager(OS_PRIORITY_VIMGR);
	osCreateMesgQueue(&gfx_msg_queue, gfx_msgs, NUM_GFX_MSGS);
	osScAddClient(&scheduler, &client, &gfx_msg_queue);
	
	//Initialize static part of task
	sc_task.list.t.type = M_GFXTASK; //This task is a graphics task
	sc_task.list.t.flags = 0; //Set to zero for FIFO graphics tasks
	//Set boot microcode to rspboot
	sc_task.list.t.ucode_boot = (u64 *)rspbootTextStart;
	sc_task.list.t.ucode_boot_size = ((u32)rspbootTextEnd - (u32)rspbootTextStart);
	sc_task.list.t.ucode = (u64 *)gspF3DEX3_BrZ_NOCTextStart;
	//Set microcode to FIFO F3DEX2
	sc_task.list.t.ucode_size = SP_UCODE_SIZE;
	sc_task.list.t.ucode_data = (u64 *)gspF3DEX3_BrZ_NOCDataStart;
	sc_task.list.t.ucode_data_size = SP_UCODE_DATA_SIZE;
	//Initialize DRAM stack parameters
	sc_task.list.t.dram_stack = dram_stack;
	sc_task.list.t.dram_stack_size = SP_DRAM_STACK_SIZE8;
	//Initialize output buffer parameters
	sc_task.list.t.output_buff = fifo_buf;
	sc_task.list.t.output_buff_size = &fifo_buf[FIFO_SIZE/8]; //Points to end of FIFO buffer for FIFO F3DEX2
	//Initialize yield buffer parameters
	sc_task.list.t.yield_data_ptr = yield_buf;
	sc_task.list.t.yield_data_size = OS_YIELD_DATA_SIZE;
	sc_task.next = NULL; //Unneeded
	//Set flags for this being the only task
	sc_task.flags = OS_SC_NEEDS_RSP | OS_SC_NEEDS_RDP | OS_SC_LAST_TASK | OS_SC_SWAPBUFFER;
	//Parameters for task done message
	sc_task.msgQ = &gfx_msg_queue;
	sc_task.msg = (OSMesg)&done_msg;
	
	//Get initial resolution
	current_res_mode = game_state.resolution_mode;
	screen_wd = resolution_table[current_res_mode].width;
	screen_ht = resolution_table[current_res_mode].height;
	
	while(1) {
		//Check if resolution changed
		if(game_state.resolution_changed) {
			current_res_mode = game_state.resolution_mode;
			screen_wd = resolution_table[current_res_mode].width;
			screen_ht = resolution_table[current_res_mode].height;
			osViSetMode(&osViModeTable[resolution_table[current_res_mode].vi_mode]);
			game_state.resolution_changed = 0;
		}
		
		//Reset display list pointer
		glistp = glist;
		//Set up direct RAM mapping for segment zero
		gSPSegment(glistp++, 0, 0);		
		//Set rendering framebuffer
		gDPSetColorImage(glistp++, G_IM_FMT_RGBA, G_IM_SIZ_16b, screen_wd, cfb_table[cfb_idx]);
		gDPSetDepthImage(glistp++, ZBUF_ADDR);
		//Enable drawing to whole screen
		gDPSetScissor(glistp++, G_SC_NON_INTERLACE, 0, 0, screen_wd, screen_ht);
		//Set up filling rectangles
		gDPSetCycleType(glistp++, G_CYC_FILL);
		//Draw background color solid
		DrawRect(&glistp, 0, 0, screen_wd, screen_ht, GPACK_RGBA5551(0, 64, 0, 1), screen_wd, screen_ht);
		//Draw background bitmap
		Draw_Background(&glistp, screen_wd, screen_ht);
		//Draw model (currently disabled, broken)
		gSPTexture(glistp++,0x8000, 0x8000, 0, 0, G_ON);
		gDPSetCycleType(glistp++, G_CYC_1CYCLE);
		gDPSetCombineMode(glistp++,G_CC_DECALRGBA, G_CC_DECALRGBA);
		gDPSetTextureFilter(glistp++, G_TF_BILERP);
		gSPClearGeometryMode(glistp++,0xFFFFFFFF);
		gSPSegment(glistp++, 0x03, OS_K0_TO_PHYSICAL(model_buf));
		//gSPDisplayList(glistp++, sc1scene_Cube_mesh);
		gDPSetCycleType(glistp++, G_CYC_FILL);
		//Draw square
		DrawRect(&glistp, game_state.square_pos_x, game_state.square_pos_y, SQUARE_SIZE, SQUARE_SIZE, GPACK_RGBA5551(255, 255, 255, 1), screen_wd, screen_ht);
		//Draw square borders
		DrawRect(&glistp, game_state.square_pos_x, game_state.square_pos_y, SQUARE_SIZE, SQUARE_BORDER_W, GPACK_RGBA5551(0, 0, 0, 1), screen_wd, screen_ht);
		DrawRect(&glistp, game_state.square_pos_x, game_state.square_pos_y, SQUARE_BORDER_W, SQUARE_SIZE, GPACK_RGBA5551(0, 0, 0, 1), screen_wd, screen_ht);
		DrawRect(&glistp, game_state.square_pos_x+SQUARE_SIZE-SQUARE_BORDER_W, game_state.square_pos_y, SQUARE_BORDER_W, SQUARE_SIZE, GPACK_RGBA5551(0, 0, 0, 1), screen_wd, screen_ht);
		DrawRect(&glistp, game_state.square_pos_x, game_state.square_pos_y+SQUARE_SIZE-SQUARE_BORDER_W, SQUARE_SIZE, SQUARE_BORDER_W, GPACK_RGBA5551(0, 0, 0, 1), screen_wd, screen_ht);
		//End frame, now print debug
		Debug_Print(&glistp, 160, 7, "WELL FUCK ME SIDEWAYS");
		
		rsp_taskend = osGetCount();
		rsp_tasktime = rsp_taskend - rsp_taskstart;
	
		u32 sCPU_start = osGetCount();
		// Fazana's performance profiler
		u32 tmem = IO_READ(DPC_TMEM_REG);
		u32 cmd =  MAX(IO_READ(DPC_BUFBUSY_REG), tmem);
		u32 pipe = MAX(IO_READ(DPC_PIPEBUSY_REG), cmd);
		calculate_and_update_fps();

		sprintf(fpsbuf, "CPU: %05.2fMS", OS_CYCLES_TO_USEC(sCPU) / 1000.0f);
		Debug_Print(&glistp, 32, 32, fpsbuf);
		sprintf(fpsbuf, "RSP: %05.2fMS", OS_CYCLES_TO_USEC(rsp_tasktime) / 1000.0f);
		Debug_Print(&glistp, 32, 48, fpsbuf);
		sprintf(fpsbuf, "RDP: %05.2fMS", (u32) (pipe * 10) / 625.0f);
		Debug_Print(&glistp, 32, 64, fpsbuf);
		sprintf(fpsbuf, "FPS: %2.2f  ", gFPS);
		Debug_Print(&glistp, 32, 80, fpsbuf);
		IO_WRITE(DPC_STATUS_REG, (DPC_CLR_CLOCK_CTR | DPC_CLR_CMD_CTR | DPC_CLR_PIPE_CTR | DPC_CLR_TMEM_CTR));	
		u32 sCPU_end = osGetCount();
		sCPU = sCPU_end - sCPU_start;
		if(game_state.resolution_mode == RES_640x480){
			Debug_Print(&glistp, resolution_table[game_state.resolution_mode].width - 100, resolution_table[game_state.resolution_mode].height - 40, "640X480");
		}
		else{
			Debug_Print(&glistp, resolution_table[game_state.resolution_mode].width - 75, resolution_table[game_state.resolution_mode].height - 30, "320X240");
		}
		
		//End display list
		gDPFullSync(glistp++);
		gSPEndDisplayList(glistp++);
		//Set task display list
		sc_task.list.t.data_ptr = (u64 *)glist;
		sc_task.list.t.data_size = (glistp-glist)*sizeof(Gfx); //In bytes
		//Set task framebuffer
		sc_task.framebuffer = cfb_table[cfb_idx];
		//Writeback whole data cache to let RCP see task properly
		osWritebackDCacheAll();
		//Send task to scheduler
		osSendMesg(osScGetCmdQ(&scheduler), (OSMesg)&sc_task, OS_MESG_BLOCK);
		//Wait for rendering to finish
		//Happens when done message is received
		do
		{
			osRecvMesg(&gfx_msg_queue, (OSMesg *)&wait_msg, OS_MESG_BLOCK);
		} while (wait_msg->type != OS_SC_DONE_MSG);
		rsp_taskstart = osGetCount();
		//Swap framebuffer
		cfb_idx ^= 1;
	}
}

static void main(void *arg)
{
	char start_pressed = 0;
	//Initialize game state with default resolution
	game_state.resolution_mode = RES_640x480;
	game_state.resolution_changed = 0;
	screen_wd = resolution_table[game_state.resolution_mode].width;
	screen_ht = resolution_table[game_state.resolution_mode].height;
	
	//Center Square
	game_state.square_pos_x = (screen_wd/2)-(SQUARE_SIZE/2);
	game_state.square_pos_y = (screen_ht/2)-(SQUARE_SIZE/2);

	//Initialize scheduler
	osCreateScheduler(&scheduler, &sched_stack[OS_SC_STACKSIZE/8], SCHED_PRIORITY, 
		resolution_table[game_state.resolution_mode].vi_mode, 1);
	
	//Initialize other system components
	InitController();
	InitAudio();
	
	//Prepare background data
	Load_Background();
	
	//Prepare model data
	Load_Model();
	
	//Create message queue for synchronization with graphics thread
	osCreateMesgQueue(&game_msg_queue, game_msgs, 8);
	
	//Start graphics thread
	osCreateThread(&gfx_thread, 4, graphics, NULL, &gfx_stack[GFX_STACK_SIZE/8], GFX_PRIORITY);
	osStartThread(&gfx_thread);
	
	//Signal graphics thread that scheduler is ready
	osSendMesg(&game_msg_queue, NULL, OS_MESG_NOBLOCK);
	
	while(1) {
		//Update controller
		ReadController();
		
		//Get current screen dimensions
		screen_wd = resolution_table[game_state.resolution_mode].width;
		screen_ht = resolution_table[game_state.resolution_mode].height;
		
		//Toggle resolution with L button
		if(cont_data[0].button & START_BUTTON && !start_pressed) {
			game_state.resolution_mode = (game_state.resolution_mode + 1) % RES_MODE_COUNT;
			game_state.resolution_changed = 1;
			//Wait a bit to debounce button
		}
		else{
			start_pressed = 0;
		}
		
		//Move square with analog stick
		if(cont_data[0].stick_x > STICK_DEADZONE || cont_data[0].stick_x < -STICK_DEADZONE) {
			game_state.square_pos_x += cont_data[0].stick_x*SQUARE_VELOCITY_SCALE;
		}
		if(cont_data[0].stick_y > STICK_DEADZONE || cont_data[0].stick_y < -STICK_DEADZONE) {
			game_state.square_pos_y -= cont_data[0].stick_y*SQUARE_VELOCITY_SCALE;
		}
		//Keep square inside screen
		if(game_state.square_pos_x < 0) {
			game_state.square_pos_x = 0;
		}
		if(game_state.square_pos_x >= screen_wd-SQUARE_SIZE) {
			game_state.square_pos_x = screen_wd-SQUARE_SIZE-1;
		}
		if(game_state.square_pos_y < 0) {
			game_state.square_pos_y = 0;
		}
		if(game_state.square_pos_y >= screen_ht-SQUARE_SIZE) {
			game_state.square_pos_y = screen_ht-SQUARE_SIZE-1;
		}
		
	}
}