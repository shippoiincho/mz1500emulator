//  SHARP MZ-1500 emulator
//
//  GP0: HSYNC
//  GP1: VSYNC
//  GP2: Blue
//  GP3: Red
//  GP4: Green
//  GP6: Audio

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/dma.h"
#include "hardware/uart.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"

#include "tusb.h"
#include "bsp/board.h"

#include "vga16_graphics.h"

#include "mzkeymap.h"
#include "z80.h"
#include "mzmisc.h"

#include "lfs.h"

#define USE_NEWMON
//#define USE_KANJI

#ifdef USE_KANJI
#include "mzkanji.h"
#endif


#ifdef USE_NEWMON
#include "mznewrom.h"
#else
#include "mzrom.h"
#endif

// VGAout configuration

#define DOTCLOCK 25000
#define CLOCKMUL 9
// Pico does not work at CLOCKMUL=7 (175MHz).

#define VGA_PIXELS_X 320
#define VGA_PIXELS_Y 400

#define VGA_CHARS_X 40
#define VGA_CHARS_Y 25

#define VRAM_PAGE_SIZE (VGA_PIXELS_X*VGA_PIXELS_Y/8)

extern unsigned char vga_data_array[];
volatile uint8_t fbcolor,cursor_x,cursor_y,video_mode;

volatile uint32_t video_hsync,video_vsync,scanline,vsync_scanline;
volatile uint32_t redraw_command=0;
volatile uint32_t scroll_flag=0;

struct repeating_timer timer,timer2;

// MZ configuration

static Z80Context cpu;
uint32_t cpu_clocks=0;
uint32_t cpu_ei=0;
int exec_reti;

uint8_t mainram[0x10000];
uint8_t vram[0x1000];
uint8_t pcg[0x6000];

uint8_t ioport[0x100];
uint8_t memioport[0x100];

uint8_t ramfile[0x10000];

uint32_t rombank=0;
uint32_t vrambank=0;
uint32_t pcgbank=0;

#ifdef USE_KANJI
uint32_t kanji_ptr=0;
uint32_t jisho_ptr=0;
#endif
uint32_t ramfile_ptr=0;

uint8_t i8253[3];
uint8_t i8253_access[3];
uint16_t i8253_preload[3];
uint16_t i8253_counter[3];
uint16_t i8253_latch[3];
uint32_t i8253_latch_flag=0;
volatile uint32_t i8253_enable_irq=0;
uint32_t beep_flag=0;
uint32_t beep_mute=0;

#define I8253CLOCK 894000 // NTSC
//#define I8253CLOCK 110800 // PAL

uint8_t sioa[8],siob[8];
uint8_t sioa_access,siob_access;

uint8_t pioa[4],piob[4];
volatile uint32_t pioa_enable_irq=0;
volatile uint32_t piob_enable_irq=1;
uint32_t pioa_next_mask,pioa_next_iocontrol;
uint32_t piob_next_mask,piob_next_iocontrol;
uint32_t pio_irq_processing=0;

volatile uint32_t tempo_timer=0;
volatile uint32_t cursor_timer=0;

uint8_t palette[8];
uint8_t color_palette[0x80];

volatile uint8_t keypressed=0;  //last pressed usbkeycode

uint32_t lastmodifier=0; 

// BEEP & PSG

uint32_t pwm_slice_num;
volatile uint32_t sound_tick=0;

#define PSG_NUMBERS 2

uint16_t psg_register[8 * PSG_NUMBERS];
uint32_t psg_osc_interval[3 * PSG_NUMBERS];
uint32_t psg_osc_counter[3 * PSG_NUMBERS];

uint32_t psg_noise_interval[PSG_NUMBERS];
uint32_t psg_noise_counter[PSG_NUMBERS];
uint8_t psg_noise_output[PSG_NUMBERS];
uint32_t psg_noise_seed[PSG_NUMBERS];
uint32_t psg_freq_write[PSG_NUMBERS];
// uint32_t psg_envelope_interval[PSG_NUMBERS];
// uint32_t psg_envelope_counter[PSG_NUMBERS];
//uint32_t psg_master_clock = PSG_CLOCK2;
//uint32_t psg_master_clock = (3579545/2);
uint32_t psg_master_clock = 3579545;
uint16_t psg_master_volume = 0;

// TESUTO
uint32_t psg_note_count;

//const uint16_t psg_volume[] = { 0x00, 0x00, 0x01, 0x01, 0x02, 0x02, 0x03, 0x04,
//        0x05, 0x06, 0x07, 0x08, 0x09, 0x0b, 0x0d, 0x10, 0x13, 0x17, 0x1b, 0x20,
//        0x26, 0x2d, 0x36, 0x40, 0x4c, 0x5a, 0x6b, 0x80, 0x98, 0xb4, 0xd6, 0xff };

const uint16_t psg_volume[] = { 0xFF,0xCB,0xA1,0x80,0x66,0x51,0x40,0x33,0x28,0x20,0x1A,0x14,0x10,0x0D,0x0A,0x00};

#define SAMPLING_FREQ 48000
//#define SAMPLING_FREQ 22050
#define TIME_UNIT 100000000                           // Oscillator calculation resolution = 10nsec
#define SAMPLING_INTERVAL (TIME_UNIT/SAMPLING_FREQ) 

// Tape

uint32_t tape_ready=0;
uint32_t tape_ptr=0;
uint32_t tape_phase=0;
uint32_t tape_count=0;

#define TAPE_THRESHOLD 200000

uint8_t uart_rx[32];
uint8_t uart_nibble=0;
uint8_t uart_count=0;
volatile uint8_t uart_write_ptr=0;
volatile uint8_t uart_read_ptr=0;
uint32_t uart_cycle;

#define TAPE1 245
#define TAPE0 123

// UI

uint32_t menumode=0;
uint32_t menuitem=0;

// USB

hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
extern void hid_app_task(void);

uint32_t usbcheck_count=0;
uint32_t kbhit=0;            // 4:Key pressed (timer stop)/3&2:Key depressed (timer running)/1:no key irq triggerd
uint8_t hid_dev_addr=255;
uint8_t hid_instance=255;
uint8_t hid_led;

uint8_t keymatrix[16];

#define USB_CHECK_INTERVAL 30 // 31.5us*30=1ms

// QD
uint8_t qd_status=0;
uint8_t qd_filename[16];
lfs_file_t qd_drive;
uint8_t qd_motor=0;
uint8_t qd_data;
uint32_t qd_ptr=0;
uint32_t qd_blocksize=0;
uint32_t qd_sync=0;
uint32_t qd_type=0;
uint32_t qd_stage=0;
uint8_t qd_numblocks=0;
uint32_t qd_count;

const uint8_t qd_header[]={0xa5,0x00,0x00,0x00,0x16};

// Define the flash sizes
// This is setup to read a block of the flash from the end 
#define BLOCK_SIZE_BYTES (FLASH_SECTOR_SIZE)
// for 1M flash pico
//#define HW_FLASH_STORAGE_BASE   (1024*1024 - HW_FLASH_STORAGE_BYTES) 
//#define HW_FLASH_STORAGE_BYTES  (512 * 1024)
// for 2M flash
// #define HW_FLASH_STORAGE_BYTES  (1024 * 1024)
#define HW_FLASH_STORAGE_BYTES  (1536 * 1024)
#define HW_FLASH_STORAGE_BASE   (PICO_FLASH_SIZE_BYTES - HW_FLASH_STORAGE_BYTES) 
// for 16M flash
//#define HW_FLASH_STORAGE_BYTES  (15872 * 1024)
//#define HW_FLASH_STORAGE_BASE   (1024*1024*16 - HW_FLASH_STORAGE_BYTES) 

lfs_t lfs;
lfs_file_t lfs_file;

#define FILE_THREHSOLD 20000000
#define LFS_LS_FILES 9

volatile uint32_t load_enabled=0;
volatile uint32_t save_enabled=0;
//uint32_t file_cycle=0;

unsigned char filename[16];
unsigned char tape_filename[16];

static void draw_framebuffer(uint16_t);
static inline unsigned char tohex(int);
static inline unsigned char fromhex(int);
static inline void video_print(uint8_t *);
uint8_t fdc_find_sector(void);


// Virtual H-Sync for emulation
bool __not_in_flash_func(hsync_handler)(struct repeating_timer *t) {

    if(scanline%262==0) {
        video_vsync=1;
    }

    scanline++;

    video_hsync=1;

    // Tempo 555 32Hz

    if((scanline%246)==0) {
        if(tempo_timer) {
            tempo_timer=0;
        } else {
            tempo_timer=1;
        }
    }

    // cursor 555 1.5Hz

    if((scanline%5249)==0) {
        if(cursor_timer) {
            cursor_timer=0;
        } else {
            cursor_timer=1;
        }

    }

    // run i8253 channel 1/2
    // channel 1 15.75kHz

        if((i8253_access[1]<2)&&(i8253_preload[1]!=0)) {
            if(i8253_counter[1]>2) {
                i8253_counter[1]--;
                    } else {
                i8253_counter[1]=i8253_preload[1];
                if((i8253_access[2]<2)&&(i8253_preload[2]!=0)){
                    if (i8253_counter[2]>1) {
                       i8253_counter[2]--;
                    } else {
                        i8253_counter[2]=i8253_preload[2];

                        // Timer Interrupt
                        // Check mask
                        if(memioport[2]&0x4) {
                            i8253_enable_irq=2;
                        }
                        if((pioa[2]&0x80)&&((pioa[3]&0x20)==0)) { 
                            pioa_enable_irq=1;
                            ioport[0xfe]!=0x20;
                        }
                        
                    }
                }
            }
        }

    return true;

}

// BEEP and PSG emulation
bool __not_in_flash_func(sound_handler)(struct repeating_timer *t) {

    uint16_t timer_diffs;
    uint32_t pon_count;
    uint16_t master_volume;
    uint32_t beep_on,beep_volume;
    uint8_t tone_output[3 * PSG_NUMBERS], noise_output[3 * PSG_NUMBERS];

    pwm_set_chan_level(pwm_slice_num,PWM_CHAN_A,psg_master_volume);

    // BEEP
    // i8253 timer 0

    beep_on=0;

    if(memioport[8]&1) {  // i8253 GATE#0 enable

        timer_diffs= I8253CLOCK / SAMPLING_FREQ;

        if(timer_diffs<i8253_counter[0]){
            i8253_counter[0]-=timer_diffs;

            if(memioport[2]&1) {  // Beep flag

                beep_on=1;

                if((i8253[0]&0x3e)==0x36) {  // Mode 3

                    if(i8253_counter[0]>(i8253_preload[0]/2)) {
                        beep_volume=255;
                    } else {
                        beep_volume=0;
                    }
                } else if((i8253[0]&0x3e)==0x30){ // mode 0
                    beep_volume=0;
                } else if((i8253[0]&0x3e)==0x3a){ // mode 5
                    beep_volume=255;
                }
            } else {
                beep_on=0;
            }

        } else {
            i8253_counter[0]=i8253_counter[0]+(i8253_preload[0]-timer_diffs);

            // interrupt on Mode 0 via Z80PIO

            if((i8253[0]&0x3e)==0x30) { // Mode 0

                if((pioa[2]&0x80)&&((pioa[3]&0x10)==0)) {
                        pioa_enable_irq=1;
                        ioport[0xfe]!=0x10;

                }
            } 

        }
    } else {
        if(memioport[2]&1) {
            beep_on=1; 
            beep_volume=255;
        } else {
            beep_on=0;
            beep_volume=0;
        }
    }

    // PSG

    // Run Noise generator

    for (int i = 0; i < PSG_NUMBERS; i++) {

        psg_noise_counter[i] += SAMPLING_INTERVAL;
        if (psg_noise_counter[i] > psg_noise_interval[i]) {
            psg_noise_seed[i] = (psg_noise_seed[i] >> 1)
                    | (((psg_noise_seed[i] << 14) ^ (psg_noise_seed[i] << 16))
                            & 0x10000);
            psg_noise_output[i] = psg_noise_seed[i] & 1;
            psg_noise_counter[i] -= psg_noise_interval[i];
        }

    }

    // Run Oscillator

    for (int i = 0; i < 3 * PSG_NUMBERS; i++) {
        pon_count = psg_osc_counter[i] += SAMPLING_INTERVAL;
        if (pon_count < (psg_osc_interval[i] / 2)) {
//            tone_output[i] = psg_tone_on[i];
            tone_output[i] = 1;
        } else if (pon_count > psg_osc_interval[i]) {
            psg_osc_counter[i] -= psg_osc_interval[i];
//            tone_output[i] = psg_tone_on[i];
            tone_output[i] = 1;
        } else {
            tone_output[i] = 0;
        }
    }

    // Mixer

    master_volume = 0;
    psg_note_count=0;

    for (int i = 0; i < PSG_NUMBERS; i++) {
        for (int j = 0; j < 3; j++) {
            if(tone_output[j+i*3]) {
                master_volume+=psg_volume[psg_register[j*2+i*8+1]];
//                master_volume+=psg_volume[32];
            } 
        }
        if(psg_noise_output[i]) {
            master_volume+=psg_volume[psg_register[7+i*8]];
        }
    }

    // count enable channels

    for (int i = 0; i < PSG_NUMBERS; i++) {
        for (int j = 0; j < 4; j++) {
            if(psg_register[j*2+i*8+1]!=0xf) {
                    psg_note_count++;
            }            
        }
    }

    if(beep_on) {
        psg_note_count++;
        master_volume+=beep_volume;
    }

//    psg_master_volume = master_volume / (3 * PSG_NUMBERS);
    psg_master_volume = master_volume / psg_note_count;


    return true;
}

// PSG virtual registers
// 0: CH0 Freq
// 1: CH0 Volume
// 6: Noise Freq
// 7: Noise Volume

void psg_write(uint32_t psg_no,uint32_t data) {

    uint32_t channel,freqdiv,freq;

    if(data&0x80) {

        channel=(data&0x60)>>5;
        psg_freq_write[psg_no]=0;

        switch((data&0x70)>>4) {

            // Frequency

            case 0:
            case 2:
            case 4:

                psg_register[psg_no*8+channel*2]=data&0xf;
                psg_freq_write[psg_no]=channel;
                break;

            case 6:  // WIP
                psg_register[psg_no*8+6]=data&0xf;
                switch(data&3){
                    case 0:
                        freqdiv=512;
                        break;
                    case 1:
                        freqdiv=1024;
                        break;
                    case 2:
                        freqdiv=2048;
                        break;
                    case 3:
                        freqdiv=psg_register[psg_no*8+4];
                }


                if(freqdiv==0) {
                    psg_noise_interval[psg_no]=UINT32_MAX;
                    return;
                }

                freq= psg_master_clock / freqdiv;
                freq>>=5;

                if(freq==0) {
                    psg_noise_interval[psg_no]=UINT32_MAX; 
                } else {
                    psg_noise_interval[psg_no]= TIME_UNIT/freq;
                    psg_noise_counter[psg_no]=0;
                }

                break;

            // volume

            case 1:
            case 3:
            case 5:
            case 7:
            
                psg_register[psg_no*8+channel*2+1]=data&0xf;

                break;

        }

    } else {

        uint32_t noise_flag=psg_register[psg_no*8+6]&3;
        
        channel=psg_freq_write[psg_no];
        psg_register[psg_no*8+channel*2]|=(data&0x3f)<<4;

        freqdiv=psg_register[psg_no*8+channel*2];

        if(freqdiv==0) {
            psg_osc_interval[psg_no*3+channel]=UINT32_MAX;
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=UINT32_MAX;
            }
            return;
        }

        freq= psg_master_clock / freqdiv;
        freq>>=5;

        if(freq==0) {
            psg_osc_interval[psg_no*3+channel]=UINT32_MAX; 
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=UINT32_MAX;
            }
        } else {
            psg_osc_interval[psg_no*3+channel]= TIME_UNIT/freq;
            psg_osc_counter[psg_no*3+channel]=0;
            if(noise_flag==3) {
                psg_noise_interval[psg_no]=TIME_UNIT/freq;
                psg_noise_counter[psg_no]=0;
            }

        }

    }    
}

void __not_in_flash_func(uart_handler)(void) {

    uint8_t ch;

    // if((main_cpu.cycles-uart_cycle)>TAPE_THRESHOLD) {
    //     uart_count=0;        
    // }

    // uart_cycle=main_cpu.cycles;

    if(uart_is_readable(uart0)) {
        ch=uart_getc(uart0);
        if(uart_count==0) {
            uart_nibble=fromhex(ch)<<4;
            uart_count++;
        } else {
            ch=fromhex(ch)+uart_nibble;
            uart_count=0;

            if(uart_read_ptr==uart_write_ptr+1) {  // buffer full
                return;
            }
            if((uart_read_ptr==0)&&(uart_write_ptr==31)) {
                return;
            }

            uart_rx[uart_write_ptr]=ch;
            uart_write_ptr++;
            if(uart_write_ptr>31) {
                uart_write_ptr=0;
            }
        }
    }

}

// MZ series tape format
// Leader block
// 0 x 22000
// 1 x 40
// 0 x 40
// 1
//----
// Information block(128B)
// Checksum(2B)
// 1
// 0 x 256
// Informain block(128B)
// Checksum(2B)
// 1
//---
// 0 x 11000
// 1 x 20
// 0 x 20
// 1
// Data(variable)
// Checksum(2B)
// 1
// 0 x 256
// Data(variable)
// Checksum(2B)
// 1

uint8_t tapein() {

    static uint32_t bytecount,bitcount,bitphase;
    static uint32_t tapebit,nextbit;
    static uint8_t tapebyte;
    static uint32_t state,checksum;
    static uint32_t tapeclocks;
    static uint32_t tape_start_ptr;
    static uint32_t blocksize;

    if(load_enabled==0) {
        return 0;
    }

    // check motor on

    if(tape_ready==0) {
        return 0;
    }

    // check initial state

    if((tape_phase==0)&&(tape_count==0)) {
        bytecount=0;
        bitcount=0;
        bitphase=0;
        blocksize=0;
        nextbit=1;
        tape_start_ptr=tape_ptr;
    }

    // need next bit ?

    if(tapebit) {
        if ((cpu_clocks-tapeclocks)>TAPE1*2) {
            nextbit=1;
        }
    } else {
        if ((cpu_clocks-tapeclocks)>TAPE0*2) {
            nextbit=1;
        }
    }

    // get next bit

    if(nextbit) {

        nextbit=0;
        bitphase=0;
        tape_count=1;

        if(tape_phase==0) { // Send 0 x 22000
            bitcount++;
            tapebit=0;
            if(bitcount>22000) {
                tape_phase=1;
                bitcount=0;
            }           
                    
        } 
        if(tape_phase==1) { // Send 1 x 40
            bitcount++;
            tapebit=1;
            if(bitcount>40) {
                tape_phase=2;
                bitcount=0;
            }  

        }
        if(tape_phase==2) { // Send 0 x 40
            bitcount++;
            tapebit=0;
            if(bitcount>40) {
                tape_phase=3;
                bitcount=0;
            }  
        }
        if(tape_phase==3) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=4;
                bitcount=8;
                bytecount=0;
                checksum=0;
            }  
        }
        if(tape_phase==4) { // Send Information Block
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>128) {
                    tape_phase=5;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
                    tape_ptr++;

                    if(bytecount==19) {
                        blocksize=tapebyte;
                    }
                    if(bytecount==20) {
                        blocksize+=tapebyte*256;
                    }


                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==5) { // Send Checksum of  Information Block
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {
                    tape_phase=6;
//                    tape_phase=11;          // SKIP SECOND BLOCK
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==6) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=7;
                bitcount=0;
                bytecount=0;
                checksum=0;
            }  
        }    
        if(tape_phase==7) { // Send 0 x 256
            bitcount++;
            tapebit=0;
            if(bitcount>256) {
                tape_phase=8;
                bitcount=8;
                bytecount=0;
                checksum=0;
                tape_ptr=tape_start_ptr;
                lfs_file_seek(&lfs,&lfs_file,tape_ptr,LFS_SEEK_SET);
            }  
        }
        if(tape_phase==8) { // Send Information Block (2nd)
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>128) {
                    tape_phase=9;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
//                    tapebyte=mztest[tape_ptr];
                    tape_ptr++;

//                    checksum+=tapebyte;
                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==9) { // Send Checksum of  Information Block
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {
                    tape_phase=10;
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }
                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }    
        if(tape_phase==10) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=11;
                bitcount=0;
            }  
        }
        if(tape_phase==11) { // Send 0 x 11000
            bitcount++;
            tapebit=0;
            if(bitcount>11000) {
                tape_phase=12;
                bitcount=0;
            }           
                    
        } 
        if(tape_phase==12) { // Send 1 x 20
            bitcount++;
            tapebit=1;
            if(bitcount>20) {
                tape_phase=13;
                bitcount=0;
            }  

        }
        if(tape_phase==13) { // Send 0 x 20
            bitcount++;
            tapebit=0;
            if(bitcount>20) {
                tape_phase=14;
                bitcount=0;
            }  
        }
        if(tape_phase==14) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=15;
                bitcount=8;
                bytecount=0;
                checksum=0;
                tape_ptr=tape_start_ptr+128;
                lfs_file_seek(&lfs,&lfs_file,tape_ptr,LFS_SEEK_SET);
            }  
        }
        if(tape_phase==15) { // Send Data
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>blocksize) {
                    tape_phase=16;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
//                    tapebyte=mztest[tape_ptr];
                    tape_ptr++;

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==16) { // Send Checksum of DATA
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {  
                    tape_phase=17;
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==17) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=18; // May be finish here
                bitcount=0;
                bytecount=0;
                checksum=0;
            }  
        }    
        if(tape_phase==18) { // Send 0 x 256
            bitcount++;
            tapebit=0;
            if(bitcount>256) {
                tape_phase=19;
                bitcount=8;
                bytecount=0;
                checksum=0;
                tape_ptr=tape_start_ptr+128;
                lfs_file_seek(&lfs,&lfs_file,tape_ptr,LFS_SEEK_SET);
            }  
        }
        if(tape_phase==19) { // Send DATA (2nd)
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>blocksize) {
                    tape_phase=20;
                    bitcount=8;
                    bytecount=0;
                } else {
                
                    lfs_file_read(&lfs,&lfs_file,&tapebyte,1);
//                    tapebyte=mztest[tape_ptr];
                    tape_ptr++;

                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    checksum++;
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }
        if(tape_phase==20) { // Send Checksum of DATA
            bitcount++;
            if(bitcount>8) {
                bytecount++;
                if(bytecount>2) {
                    tape_phase=21;
                    bitcount=0;
                    bytecount=0;
                } else {
                    if(bytecount==1) {
                        tapebyte=(checksum&0xff00)>>8;
                    } else {
                        tapebyte=checksum&0xff;
                    }
                    bitcount=0;
                    tapebit=1;   // start bit

                }
            } else {

                if(tapebyte&(1<<(8-bitcount))) {
                    tapebit=1;
                } else {
                    tapebit=0;
                }
            }
        }    
        if(tape_phase==21) { // Send 1
            bitcount++;
            tapebit=1;
            if(bitcount>1) {
                tape_phase=22;
                bitcount=0;
            }  
        }

                
    }

    // 
    if(bitphase==0) { // send High First
        tapeclocks=cpu_clocks;
        bitphase=1;
        return 1;
    } else if(tapebit) {
        if ((cpu_clocks-tapeclocks)<TAPE1) {
            return 1;
        } else {
            return 0;
        }    
    } else {
        if ((cpu_clocks-tapeclocks)<TAPE0) {
            return 1;
        } else {
            return 0;
        }  
    }
    
    

    
    return 0;

}

void tapeout(uint8_t data) {

    static uint32_t bytecount,bitcount,bitphase;
    static uint32_t tapebit,blocksize;
    static uint32_t tapeclocks;
    static uint8_t tapebyte;

    if(save_enabled==0) {
        return;
    }

    save_enabled=2;

    // check motor on

    if(tape_ready==0) {
        return;
    }

    if(data) {  // bit High
        tapeclocks=cpu_clocks;
        return;
    } else { 

        if((cpu_clocks-tapeclocks)>200) {
            tapebit=1;
        } else {
            tapebit=0;
        }
    }

    switch(tape_phase) {

        case 0:     //  0 x 22000
            if(tapebit) {
                tape_phase=1;
                bitcount=1;
            }
            return;

        case 1:   // 1 x 40 + 0 x 40 + 1
            bitcount++;
            if(bitcount>81) {
                tape_phase=2;
                bitcount=1;
                bytecount=0;
                tape_count=0;
            }
            return;

        case 2:  // Header(1st)
            bitcount++;
            tape_count++;
            if(bitcount>1) {
                tapebyte<<=1;
                if(tapebit) {
                    tapebyte|=1;
                }
            }
            if(bitcount==9) {
                
                lfs_file_write(&lfs,&lfs_file,&tapebyte,1);

                bitcount=0;
                bytecount++;
                if(bytecount==0x13) {
                    blocksize=tapebyte;
                }
                if(bytecount==0x14) {
                    blocksize+=tapebyte*256;
                }

                if(bytecount==128) {
                    tape_phase=3;
                    bitcount=0;
                }
            }
            return;

        case 3: // checksum(18bit) + 1 + 0 x 256 + Header(2nd 2304bit) + checksum(18bit) + 1
            bitcount++;
            if(bitcount>2598) {
                tape_phase=4;
                bitcount=1;
                bytecount=0;
            }
            return;

        case 4:     //  1 x 22000
            if(tapebit) {
                tape_phase=5;
                bitcount=1;
            }
            return;

       case 5: // 1 x 20 + 0 x 20 + 1
            bitcount++;
            if(bitcount>41) {
                tape_phase=6;
                bitcount=1;
                bytecount=0;
            }
            return;

        case 6:  // data(1st)
            bitcount++;
            tape_count++;
            if(bitcount>1) {
                tapebyte<<=1;
                if(tapebit) {
                    tapebyte|=1;
                }
            }
            if(bitcount==9) {

                lfs_file_write(&lfs,&lfs_file,&tapebyte,1);

                bitcount=0;
                bytecount++;

                if(bytecount>=blocksize) {
                    tape_phase=7;
                    bitcount=0;

                    // can be finish here

                    lfs_file_close(&lfs,&lfs_file);
                    save_enabled=0;

                }
            }
            return;

    }

}

void qd_check() {

    uint8_t qdftmp[16];

    lfs_file_seek(&lfs,&qd_drive,0,LFS_SEEK_SET);
    lfs_file_read(&lfs,&qd_drive,qdftmp,16);

    if(memcmp(qdftmp,"-QD format-",11)==0) {
        qd_type=0;
    } else {
        qd_ptr=0;
        qd_numblocks=0;
        while(qd_ptr<lfs_file_size(&lfs,&qd_drive)) {

            lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);
            lfs_file_read(&lfs,&qd_drive,qdftmp,16);
            lfs_file_read(&lfs,&qd_drive,qdftmp,16);
            qd_blocksize=qdftmp[3]*256+qdftmp[2];

            qd_ptr+=128;
            qd_ptr+=qd_blocksize;
            qd_numblocks+=2;
        }

        qd_type=1;
        qd_stage=0;
        qd_count=0;
    }

    lfs_file_seek(&lfs,&qd_drive,0,LFS_SEEK_SET);    

}


uint8_t qd_read() {

    uint8_t qd_data,qd_mark,size_hi,size_lo;
    static uint32_t qd_blockcount;

    if(qd_status==0) {
        return 0xff;
    }

    if(qd_type==0) { // QDF format

    if(qd_motor) {

        // find sync

        if(qd_sync) {

            qd_data=0;
            while(qd_data!=0x16) { // Find Sync
                lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                qd_ptr++;
            }

            while(qd_data==0x16) { // Find Sync
                lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                qd_ptr++;
            }

            qd_sync=0;
            qd_ptr--;

//            lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);  // Not needed ?

            lfs_file_read(&lfs,&qd_drive,&qd_mark,1);

            if(qd_stage==0) {  // first record
                qd_blocksize=5;
                qd_stage=1;
            } else {
                lfs_file_read(&lfs,&qd_drive,&size_lo,1);
                lfs_file_read(&lfs,&qd_drive,&size_hi,1);
                qd_blocksize=size_lo+size_hi*256+7;
            }

            // if((qd_mark==4)||(qd_mark==2)||(qd_mark==0xff)) {
            //     qd_blocksize=5;
            // } else {
            //     lfs_file_read(&lfs,&qd_drive,&size_lo,1);
            //     lfs_file_read(&lfs,&qd_drive,&size_hi,1);
            //     qd_blocksize=size_lo+size_hi*256+7;
            // }

            lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET); 

//            printf("\n\r[QDM:%02x/%05x/%04x]",qd_data,qd_ptr,qd_blocksize);

        }

        if(qd_ptr<=lfs_file_size(&lfs,&qd_drive)) {

            qd_blocksize--;
            if(qd_blocksize==0) {
//                qd_ptr+=8;  // ?
                  qd_ptr+=16;  // ?
                lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);

                qd_sync=1;
                return 0x16;
            }            

            lfs_file_read(&lfs,&qd_drive,&qd_data,1);
            qd_ptr++;
            return qd_data;

        } else {

            qd_motor=0;

            return 0;
        }

    } 
    } else { // Q20/MZT format

        if(qd_motor) {

            switch(qd_stage) {

                case 0:   // Preample
                    qd_data=qd_header[qd_count];

                    qd_count++;

                    if(qd_count==2) {
                        qd_data=qd_numblocks;

                    }
                    if(qd_count>=5) {
                        qd_stage=1;
                        qd_blockcount=0;
                    }

                    return qd_data;

                case 1: // 

                    if(qd_blockcount>=qd_numblocks) {
                        return 0x16;
                    }

                    qd_stage=2;
                    qd_count=0;

                    return 0xa5;

                case 2:  // Header

                    qd_count++;

                    if(qd_count==1) {
                        return 0;
                    }
                    if(qd_count==2) {
                        return 0x40;
                    }
                    if(qd_count==3) {
                        return 0;
                    }

                    if(qd_count==0x16) {
                        return 0;
                    }
                    if(qd_count==0x17) {
                        return 0;
                    }


                    lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                    qd_ptr++;

                    if(qd_count==0x18) {
                        qd_blocksize=qd_data;
                    }
                    if(qd_count==0x19) {                    
                        qd_blocksize+=qd_data*256;
                    }

                    if(qd_count>=67) {
                        qd_stage=3;
                        qd_ptr+=66;
                        lfs_file_seek(&lfs,&qd_drive,qd_ptr,LFS_SEEK_SET);
                    }

                    return qd_data;

                case 3:  // Checksum
                    qd_stage=4;
                    return 0xff;

                case 4:  // Checksum
                    qd_stage=5;
                    return 0xff;

                case 5:  // Sync
                    qd_stage=6;
                    return 0x16;

                case 6:   // Preample
                    qd_stage=7;
                    qd_count=0;
                    return 0xa5;

                case 7:
                    qd_count++;
                    if(qd_count==1) return 5;
                    if(qd_count==2) return qd_blocksize&0xff;
                    if(qd_count==3) {
                        qd_stage=8;
                        qd_count=0;
                        return (qd_blocksize&0xff00)>>8;
                    }

                case 8:  // Data
                    lfs_file_read(&lfs,&qd_drive,&qd_data,1);
                    qd_ptr++;
                    qd_count++;
                    if(qd_count>=qd_blocksize) {
                        qd_stage=9;
                    }

                    return qd_data;

                case 9:  // Checksum
                    qd_stage=10;
                    return 0xff;

                case 10:  // Checksum
                    qd_stage=11;
                    return 0xff;

                case 11:  // Sync
                    qd_stage=1;
                    qd_blockcount+=2;
                    return 0x16;
            }

        }

    }

    return 0;

}


static inline void video_cls() {
    memset(vga_data_array, 0x0, (320*400/2));
}

static inline void video_scroll() {

    memmove(vga_data_array, vga_data_array + VGA_PIXELS_X/2*16, (320*384/2));
    memset(vga_data_array + (320*384/2), 0, VGA_PIXELS_X/2*16);

}

static inline void video_print(uint8_t *string) {

    int len;
    uint8_t fdata;

    len = strlen(string);

    for (int i = 0; i < len; i++) {

        for(int slice=0;slice<16;slice++) {

            uint8_t ch=string[i];


            fdata=mzfont[(mzdisplay[ch]*8+(slice>>1))];

            uint32_t vramindex=cursor_x*4+VGA_PIXELS_X*(cursor_y*16+slice)/2;

            for(int slice_x=0;slice_x<4;slice_x++){

                if(fdata&0x40) {
                    vga_data_array[vramindex+slice_x]=(fbcolor&7)<<4;
                } else {
                    vga_data_array[vramindex+slice_x]=fbcolor&0x70;  
                }

                  if(fdata&0x80) {
                    vga_data_array[vramindex+slice_x]+=fbcolor&7;
                } else {
                    vga_data_array[vramindex+slice_x]+=(fbcolor&0x70)>>4;  
                }              

                fdata<<=2;

            }

        }

        cursor_x++;
        if (cursor_x >= VGA_CHARS_X) {
            cursor_x = 0;
            cursor_y++;
            if (cursor_y >= VGA_CHARS_Y) {
                video_scroll();
                cursor_y = VGA_CHARS_Y - 1;
            }
        }
    }

}

void draw_menu(void) {

    cursor_x=2;
    cursor_y=5;
    fbcolor=7;
      video_print("                                    ");
    for(int i=6;i<19;i++) {
        cursor_x=2;
        cursor_y=i;
        video_print("                                    ");
    }

    cursor_x=2;
    cursor_y=19;
    fbcolor=7;
    video_print("                                    ");

}

int draw_files(int num_selected,int page) {

    lfs_dir_t lfs_dirs;
    struct lfs_info lfs_dir_info;
    uint32_t num_entry=0;
    unsigned char str[16];

    int err= lfs_dir_open(&lfs,&lfs_dirs,"/");

    if(err) return -1;

    for(int i=0;i<LFS_LS_FILES;i++) {
        cursor_x=22;
        cursor_y=i+6;
        fbcolor=7;
        video_print("             ");
    }

    while(1) {

        int res= lfs_dir_read(&lfs,&lfs_dirs,&lfs_dir_info);
        if(res<=0) {
            break;
        }

        cursor_x=28;
        cursor_y=18;
        fbcolor=7;
        sprintf(str,"Page %02d",page+1);

        video_print(str);

        switch(lfs_dir_info.type) {

            case LFS_TYPE_DIR:
                break;
            
            case LFS_TYPE_REG:

                if((num_entry>=LFS_LS_FILES*page)&&(num_entry<LFS_LS_FILES*(page+1))) {

                    cursor_x=22;
                    cursor_y=num_entry%LFS_LS_FILES+6;

                    if(num_entry==num_selected) {
                        fbcolor=0x70;
                        memcpy(filename,lfs_dir_info.name,16);
                    } else {
                        fbcolor=7;
                    }

                    video_print(lfs_dir_info.name);

                }

                num_entry++;

                break;

            default:
                break; 

        }

    }

    lfs_dir_close(&lfs,&lfs_dirs);

    return num_entry;

}

int file_selector(void) {

    uint32_t num_selected=0;
    uint32_t num_files=0;
    uint32_t num_pages=0;

    num_files=draw_files(-1,0);

    if(num_files==0) {
         return -1;
    }

    while(1) {

        while(video_vsync==0) ;
        video_vsync=0;

        draw_files(num_selected,num_selected/LFS_LS_FILES);

        tuh_task();

        if(keypressed==0x52) { // up
            keypressed=0;
            if(num_selected>0) {
                num_selected--;
            }
        }

        if(keypressed==0x51) { // down
            keypressed=0;
            if(num_selected<num_files-1) {
                num_selected++;
            }
        }

        if(keypressed==0x28) { // Ret
            keypressed=0;

            return 0;
        }

        if(keypressed==0x29 ) {  // ESC

            return -1;

        }

    }
}

int enter_filename() {

    unsigned char new_filename[16];
    unsigned char str[32];
    uint8_t keycode;
    uint32_t pos=0;

    memset(new_filename,0,16);

    while(1) {

        sprintf(str,"Filename:%s  ",new_filename);
        cursor_x=3;
        cursor_y=18;
        video_print(str);

        while(video_vsync==0) ;
        video_vsync=0;

        tuh_task();

        if(keypressed!=0) {

            if(keypressed==0x28) { // enter
                keypressed=0;
                if(pos!=0) {
                    memcpy(filename,new_filename,16);
                    return 0;
                } else {
                    return -1;
                }
            }

            if(keypressed==0x29) { // escape
                keypressed=0;
                return -1;
            }

            if(keypressed==0x2a) { // backspace
                keypressed=0;

                cursor_x=3;
                cursor_y=18;
                video_print("Filename:          ");

                new_filename[pos]=0;

                if(pos>0) {
                    pos--;
                }
            }

            if(keypressed<0x4f) {
                keycode=usbhidcode[keypressed*2];
                keypressed=0;

                if(pos<7) {

                    if((keycode>0x20)&&(keycode<0x5f)&&(keycode!=0x2f)) {

                        new_filename[pos]=keycode;
                        pos++;

                    }

                }
            }


        }
    }

}


static void draw_framebuffer(uint16_t addr){

    uint32_t slice_x,slice_y;
    uint16_t offset;
    uint32_t vramindex;
    uint32_t ch,color,bitdata;
    uint32_t bitdataw,bitmaskw;
    uint32_t pcgch;
    uint32_t pcgbw,pcgrw,pcggw,pcgmaskw,pcgw,mixmaskw;
    uint32_t pcgenable;

    union bytemember {
         uint32_t w;
         uint8_t b[4];
    };

    union bytemember bitw,bitc;

 //   printf("[FB:%x]",addr);

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    addr&=0x3ff;

    if(addr>=1000) return;

    slice_x=addr%40;
    slice_y=addr/40;

    ch=vram[addr];
    color=vram[addr+0x800];

    if((ioport[0xf0]&1)&&(vram[addr+0xc00]&8)) {
        pcgenable=1;
    } else {
        pcgenable=0;
    }

//   printf("[%d:%d:%02x:%02x:%04x]",slice_x,slice_y,ch,color,addr);

    for(uint32_t slice_yy=0;slice_yy<8;slice_yy++) {

        if(color&0x80) {
            bitdata=mzfont[ch*8+slice_yy+0x800];
        } else {
            bitdata=mzfont[ch*8+slice_yy];
        }

        bitdataw=bitexpand80[bitdata*2];
        bitmaskw=bitexpand80[bitdata*2+1];

        if(pcgenable) { 

            pcgch=vram[addr+0x400]+((vram[addr+0xc00]&0xc0)<<2);

// printf("[%d:%d:%02x:%02x:%04x:%03x]",slice_x,slice_y,ch,color,addr,pcgch);

            pcgbw=bitexpand80[pcg[pcgch*8+slice_yy]*2];
            pcgmaskw=bitexpand80[pcg[pcgch*8+slice_yy]*2+1];

            pcgrw=bitexpand80[pcg[pcgch*8+slice_yy+0x2000]*2];
            pcgmaskw&=bitexpand80[pcg[pcgch*8+slice_yy+0x2000]*2+1];

            pcggw=bitexpand80[pcg[pcgch*8+slice_yy+0x4000]*2];
            pcgmaskw&=bitexpand80[pcg[pcgch*8+slice_yy+0x4000]*2+1];

            pcgw=pcgbw+pcgrw*2+pcggw*4;

            if(ioport[0xf0]&2) { // PCG>TEXT

                mixmaskw=pcgmaskw*0xf;
                bitdataw*=(color&0x70)>>4;
                bitdataw = (bitdataw & mixmaskw) | pcgw;

                bitmaskw&=pcgmaskw;
                mixmaskw=bitmaskw*0xf;

                bitmaskw*=(color&7);
                bitdataw = (bitdataw & ~mixmaskw) | bitmaskw;

            } else { // TEXT>PCG

                mixmaskw=bitmaskw*0xf;
                bitdataw*=(color&0x70)>>4;
                bitdataw = (pcgw & mixmaskw) | bitdataw;

                bitmaskw&=pcgmaskw;

                mixmaskw=bitmaskw*0xf;

                bitmaskw*=(color&7);
                bitdataw = (bitdataw & ~mixmaskw) | bitmaskw;

            }


        } else {

            bitdataw*=(color&0x70)>>4;
            bitmaskw*=(color&7);

        }

        // color palette

        bitw.w=bitdataw+bitmaskw;

        bitc.b[0]=color_palette[bitw.b[0]];
        bitc.b[1]=color_palette[bitw.b[1]];
        bitc.b[2]=color_palette[bitw.b[2]];
        bitc.b[3]=color_palette[bitw.b[3]];

        vramindex=slice_x+(slice_y*16+slice_yy*2)*40;

//        *(vramptr+vramindex) = bitdataw+bitmaskw;
//        *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw+bitmaskw;

        *(vramptr+vramindex) = bitc.w;
        *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitc.w;

    }

}

#if 0
static void draw_framebuffer_slice(uint16_t addr,uint8_t slice_yy){

    uint32_t slice_x,slice_y;
    uint16_t offset;
    uint32_t vramindex;
    uint32_t ch,color,bitdata;
    uint32_t bitdataw,bitmaskw;
    uint32_t pcgch;
    uint32_t pcgbw,pcgrw,pcggw,pcgmaskw,pcgw,mixmaskw;
    uint32_t pcgenable;

    union bytemember {
         uint32_t w;
         uint8_t b[4];
    };

    union bytemember bitw,bitc;

 //   printf("[FB:%x]",addr);

    uint32_t *vramptr=(uint32_t *)vga_data_array;

    addr&=0x3ff;

    if(addr>=1000) return;

    slice_x=addr%40;
    slice_y=addr/40;

    ch=vram[addr];
    color=vram[addr+0x800];

    if((ioport[0xf0]&1)&&(vram[addr+0xc00]&8)) {
        pcgenable=1;
    } else {
        pcgenable=0;
    }

//   printf("[%d:%d:%02x:%02x:%04x]",slice_x,slice_y,ch,color,addr);

//    for(uint32_t slice_yy=0;slice_yy<8;slice_yy++) {

        if(color&0x80) {
            bitdata=mzfont[ch*8+slice_yy+0x800];
        } else {
            bitdata=mzfont[ch*8+slice_yy];
        }

        bitdataw=bitexpand80[bitdata*2];
        bitmaskw=bitexpand80[bitdata*2+1];

        if(pcgenable) { 

            pcgch=vram[addr+0x400]+((vram[addr+0xc00]&0xc0)<<2);

// printf("[%d:%d:%02x:%02x:%04x:%03x]",slice_x,slice_y,ch,color,addr,pcgch);

            pcgbw=bitexpand80[pcg[pcgch*8+slice_yy]*2];
            pcgmaskw=bitexpand80[pcg[pcgch*8+slice_yy]*2+1];

            pcgrw=bitexpand80[pcg[pcgch*8+slice_yy+0x2000]*2];
            pcgmaskw&=bitexpand80[pcg[pcgch*8+slice_yy+0x2000]*2+1];

            pcggw=bitexpand80[pcg[pcgch*8+slice_yy+0x4000]*2];
            pcgmaskw&=bitexpand80[pcg[pcgch*8+slice_yy+0x4000]*2+1];

            pcgw=pcgbw+pcgrw*2+pcggw*4;

            if(ioport[0xf0]&2) { // PCG>TEXT

                mixmaskw=pcgmaskw*0xf;
                bitdataw*=(color&0x70)>>4;
                bitdataw = (bitdataw & mixmaskw) | pcgw;

                bitmaskw&=pcgmaskw;
                mixmaskw=bitmaskw*0xf;

                bitmaskw*=(color&7);
                bitdataw = (bitdataw & ~mixmaskw) | bitmaskw;

            } else { // TEXT>PCG

                mixmaskw=bitmaskw*0xf;
                bitdataw*=(color&0x70)>>4;
                bitdataw = (pcgw & mixmaskw) | bitdataw;

                bitmaskw&=pcgmaskw;

                mixmaskw=bitmaskw*0xf;

                bitmaskw*=(color&7);
                bitdataw = (bitdataw & ~mixmaskw) | bitmaskw;

            }


        } else {

            bitdataw*=(color&0x70)>>4;
            bitmaskw*=(color&7);

        }

        // color palette

        bitw.w=bitdataw+bitmaskw;

        bitc.b[0]=color_palette[bitw.b[0]];
        bitc.b[1]=color_palette[bitw.b[1]];
        bitc.b[2]=color_palette[bitw.b[2]];
        bitc.b[3]=color_palette[bitw.b[3]];

        vramindex=slice_x+(slice_y*16+slice_yy*2)*40;

//        *(vramptr+vramindex) = bitdataw+bitmaskw;
//        *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitdataw+bitmaskw;

        *(vramptr+vramindex) = bitc.w;
        *(vramptr+vramindex+VGA_PIXELS_X/8) =  bitc.w;

//    }

}
#endif

static inline void redraw(){

    for(int i=0xd000;i<0xd3e8;i++) {
        draw_framebuffer(i);
    }

}

//static inline void redraw_pcg(uint16_t pcgno) {
static inline void redraw_pcg(uint16_t address) {

    uint16_t pcgch,pcgno;
    uint8_t pcgslice;

    pcgno=address/8;
    pcgslice=address%8;

    for(int i=0;i<1000;i++) {

        pcgch=vram[i+0x400]+((vram[i+0xc00]&0xc0)<<2);

        if(pcgch==pcgno) {
//            draw_framebuffer_slice(i+0xd000,pcgslice);
            draw_framebuffer(i+0xd000);
        }

    }

}


 static inline void rebuild_colorpalette(void) {

     for(int i=0;i<8;i++) {
         for(int j=0;j<8;j++) {
             color_palette[i*16+j]=palette[i]*16+palette[j];
         }
     }
 }


//----------------------------------------------------------------------------------------------

void psg_reset(int flag) {

    psg_noise_seed[0] = 12345;
    psg_noise_seed[1] = 12345;


    for (int i = 0; i < 16; i+=2) {
        psg_register[i] = 0;
        psg_register[i+1] = 0xf;
    }

    psg_noise_interval[0] = UINT32_MAX;
    psg_noise_interval[1] = UINT32_MAX;

    for (int i = 0; i < 6; i++) {
        psg_osc_interval[i] = UINT32_MAX;
//        psg_tone_on[i] = 0;
//        psg_noise_on[i] = 0;
    }

}

//----------------------------------------------------------------------------------------------------

static inline unsigned char tohex(int b) {

    if(b==0) {
        return '0';
    } 
    if(b<10) {
        return b+'1'-1;
    }
    if(b<16) {
        return b+'a'-10;
    }

    return -1;

}

static inline unsigned char fromhex(int b) {

    if(b=='0') {
        return 0;
    } 
    if((b>='1')&&(b<='9')) {
        return b-'1'+1;
    }
    if((b>='a')&&(b<='f')) {
        return b-'a'+10;
    }

    return -1;

}

// LittleFS

int pico_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    uint32_t fs_start = XIP_BASE + HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] READ: %p, %d\n", addr, size);
    
    memcpy(buffer, (unsigned char *)addr, size);
    return 0;
}

int pico_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t addr = fs_start + (block * c->block_size) + off;
    
//    printf("[FS] WRITE: %p, %d\n", addr, size);
        
    uint32_t ints = save_and_disable_interrupts();
    multicore_lockout_start_blocking();     // pause another core
    flash_range_program(addr, (const uint8_t *)buffer, size);
    multicore_lockout_end_blocking();
    restore_interrupts(ints);
    
    return 0;
}

int pico_erase(const struct lfs_config *c, lfs_block_t block)
{           
    uint32_t fs_start = HW_FLASH_STORAGE_BASE;
    uint32_t offset = fs_start + (block * c->block_size);
    
//    printf("[FS] ERASE: %p, %d\n", offset, block);
        
    uint32_t ints = save_and_disable_interrupts();   
    multicore_lockout_start_blocking();     // pause another core
    flash_range_erase(offset, c->block_size);  
    multicore_lockout_end_blocking();
    restore_interrupts(ints);

    return 0;
}

int pico_sync(const struct lfs_config *c)
{
    return 0;
}

// configuration of the filesystem is provided by this struct
const struct lfs_config PICO_FLASH_CFG = {
    // block device operations
    .read  = &pico_read,
    .prog  = &pico_prog,
    .erase = &pico_erase,
    .sync  = &pico_sync,

    // block device configuration
    .read_size = FLASH_PAGE_SIZE, // 256
    .prog_size = FLASH_PAGE_SIZE, // 256
    
    .block_size = BLOCK_SIZE_BYTES, // 4096
    .block_count = HW_FLASH_STORAGE_BYTES / BLOCK_SIZE_BYTES, // 352
    .block_cycles = 16, // ?
    
    .cache_size = FLASH_PAGE_SIZE, // 256
    .lookahead_size = FLASH_PAGE_SIZE,   // 256    
};



//  Keyboard

static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

void process_kbd_report(hid_keyboard_report_t const *report) {

    int usbkey;
    uint32_t keycode;
    uint8_t row,col;

    if(menumode==0) { // Emulator mode

        for(int i=0;i<10;i++) {
            keymatrix[i]=0xff;
        }

        if((report->modifier)&0x22) { // Shift
            keymatrix[8]&= ~0x01;
        }

        if((report->modifier)&0x11) { // alt = Graph
            keymatrix[8]&= ~0x40;
        }

        if((report->modifier)&0x44) { // ctrl
            keymatrix[0]&= ~0x40;

        }

        for(int i=0;i<6;i++) {
            keycode=report->keycode[i];
            row=mzusbcode[keycode*2+1];
            if(mzusbcode[keycode*2]) {
                keymatrix[row]&= ~mzusbcode[keycode*2];
            }

            // DEBUG
//             if(report->keycode[i]==0x42) {
//                 memcpy(mainram+0x1200,mztest+0x80,0x6c81);
// //                rombank=1;
//                 cpu.PC=0x7e0f;

//             } 

            // DEBUG
            // if(report->keycode[i]==0x44) {
            //     memcpy(mainram,mztest2+0x1f41+4,0xa29e);
            //     rombank=1;
            //     cpu.PC=0;

            // } 

            // DEBUG
            // if(report->keycode[i]==0x43) {
            //     qd_status=1;
            //     pioa_enable_irq=2;

            // } 


            // Enter Menu
            if(report->keycode[i]==0x45) {
                prev_report=*report;
                menumode=1;
                keypressed=0;
            }  
        }

    prev_report=*report;

} else {  // menu mode

    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
        if ( find_key_in_report(&prev_report, report->keycode[i]) )
        {
            // exist in previous report means the current key is holding
        }else
        {
            keypressed=report->keycode[i];
        }
        }
    } 
    prev_report = *report;
    }

}

static byte mem_read(size_t param, ushort address)
{

    uint8_t b;
    static uint32_t lastclocks;

    if(address<0x1000) {
        if(rombank) {
            return mainram[address];
        } else {
            return mzipl[address];
        }
    } else if(address>=0xd000) {
  
        if(pcgbank) {

            b=ioport[0xe5];

            if(address<0xf000) {

                switch(b) {

                    case 0:
                        return mzfont[address&0xfff];
                    case 1:
                        return pcg[address-0xd000];
                    case 2:
                        return pcg[address-0xd000+0x2000];
                    case 3:
                        return pcg[address-0xd000+0x4000];
                    }

                }

                return 0xff;

            } 

        if(vrambank) {
            return mainram[address];
        }



                if((address>=0xd000)&&(address<0xe000)) {
                    return vram[address&0xfff];
                }

                if((address>=0xe000)&&(address<0xe800)) {

                    uint8_t addr=address-0xe000;

                    if(address<0xe010) {


                    switch (addr) {
                        case 1:
                        return keymatrix[memioport[0]&0xf];

                    case 2:
                        b=0xff;

                        if(video_vsync) {
                            b&=0x7f;
                        }
                        if(cursor_timer) {
                            b&=0xbf;
                        }
                        if(tape_ready==0) {
                            b&=0xe0;
                        }

                        if(tape_ready) {
                            if(tapein()) {
                                b|=0x20;
                            } else {
                                b&=0xdf;
                            }
                        }

                        return b;

                    case 4:
                    case 5:
                    case 6:


                    if(i8253_latch_flag) {

                        if(i8253_access[addr-4]) {
                            i8253_access[addr-4]=0;
                            i8253_latch_flag=0;
                            return (i8253_latch[addr-4]&0xff00)>>8;
                        } else {
                            i8253_access[addr-4]=1;
                            return i8253_latch[addr-4]&0xff;
                        }

                    } else {

                        if(i8253_access[addr-4]) {
                            i8253_access[addr-4]=0;
                            return (i8253_counter[addr-4]&0xff00)>>8;
                        } else {
                            i8253_access[addr-4]=1;
                            return i8253_counter[addr-4]&0xff;
                        }

                    }

                    case 8:

                        b=0xff;

                        if(tempo_timer) {
                            b&=0xfe;
                        }

                        if(video_hsync) {
                            b&=0x7f;
                        }

                        return b;

                    default:
                        return memioport[addr];
                    }
                    } else {
                        return 0xff;
                    }

                }

#ifndef USE_NEWMON
                if(address>=0xe800) {
                    return mzextrom[address-0xe800];
                }
#endif

            return 0xff;

            }
            
  return mainram[address];
}

static void mem_write(size_t param, ushort address, byte data)
{

    uint8_t b;
    uint16_t addr;
    static uint32_t lastclocks;

    if((address<0x1000)&&(rombank)) {
        mainram[address]=data;
        return;
    }

    if(address>=0xd000) {

        if(pcgbank) {
            b=ioport[0xe5]&3;
            if(address<0xf000) {
                addr=(address-0xd000)/8;
                switch(b) {
                    case 1:
                        pcg[address-0xd000]=data;
                        redraw_pcg(address-0xd000);
                        break;

                    case 2:
                        pcg[(address-0xd000)+0x2000]=data;
                        redraw_pcg(address-0xd000);
                        break;

                    case 3:
                        pcg[(address-0xd000)+0x4000]=data;
                        redraw_pcg(address-0xd000);
                        break;

                    default:
                        break;

                }
            }
            return;
        }

        if(vrambank) {
            mainram[address]=data;
            return;
        }

        if((address>=0xd000)&&(address<0xe000)) {
            vram[address&0xfff]=data;
            draw_framebuffer(address);
            return;
        }

        if((address>=0xe000)&&(address<0xe010)) {

            addr=address&0xf;

                switch(addr) {

                    case 2:

                        // Not used ....
                        // Normally Use bit operation

                        if((data&0x8)&&((memioport[2]&0x8)==0)) {
                            if(tape_ready) {
                                tape_ready=0;
                            } else {
                                tape_ready=1;
  //                              tape_phase=0;
  //                              tape_count=0;
                            }
                        }

                        if(data&1) {
                            beep_mute=0;
                        } else {
                            beep_mute=1;
                        }

                        if(data&4) {
                            i8253_enable_irq=1;
                        } else {
                            i8253_enable_irq=0;
                        }

                        break;

                    case 3:

                        if((data&0x80)==0) { // Bit operation

                            b=(data&0x0e)>>1;

                            if(b==1) {  // Tape Out
                                tapeout(data&1);
//                                printf("[%d/%d]\n\r",(data&1),cpu_clocks-lastclocks);
//                                lastclocks=cpu_clocks;
                            }



                            if(data&1) {

                                if(b==3) { // Motor on/off
                                    if((memioport[2]&0x8)==0) {
                                        if(tape_ready) {
//                                                printf("[Motor OFF]\n\r");
                                            tape_ready=0;
                                        } else {
                                            tape_ready=1;
//                                            tape_phase=0;
//                                                printf("[Motor ON]\n\r");
//                                            tape_count=0;
                                        }
                                    }
                                }

                                memioport[2]|= 1<<b;
                            } else {
                                memioport[2]&= ~(1<<b);
                            }

                        }

                        return;

                    case 4:
                    case 5:
                    case 6:

                        if((i8253[addr-4]&0x30)==0x30) {

                            if(i8253_access[addr-4]) {

                                i8253_preload[addr-4]&=0xff;
                                i8253_preload[addr-4]|=data<<8;
                                i8253_access[addr-4]=0;
                                i8253_counter[addr-4]=i8253_preload[addr-4];

                            } else {

                                i8253_preload[addr-4]&=0xff00;
                                i8253_preload[addr-4]|=data;
                                i8253_access[addr-4]=2;
                            }
                        }

                        if((i8253[addr-4]&0x30)==0x20) {
                            i8253_preload[addr-4]&=0xff;
                            i8253_preload[addr-4]|=data<<8;
                            i8253_counter[addr-4]=i8253_preload[addr-4];
                        }

                        if((i8253[addr-4]&0x30)==0x10) {
                            i8253_preload[addr-4]&=0xff00;
                            i8253_preload[addr-4]|=data;
                            i8253_counter[addr-4]=i8253_preload[addr-4];

                        }

//                        i8253_counter[addr-4]=i8253_preload[addr-4];

                        break;

                    case 7:  // i8253 control

                        b=(data&0xc0)>>6;

                        if(b!=3) {
                            if((data&0x30)==0) {
                                i8253_latch[b]=i8253_counter[b];
                                i8253_latch_flag=1;
                            }
                         i8253[b]=data;
                        }

                        break;

                    case 8:

                        if(b&1) { // Beep ON
                            beep_flag=1;
                        } else {  // Beep Off
                            beep_flag=0;
                        }
                        break;

                    default:


                        break;      

                }


            memioport[address-0xe000]=data;

        } 
        
    }

    mainram[address] = data;

}

static byte io_read(size_t param, ushort address)
{
    byte data = ioport[address&0xff];
    uint8_t b;
    uint32_t kanji_addr;

    switch(address&0xff) {

#ifdef USE_KANJI
        case 0xb9:

        if(ioport[0xb8]&0x80) { // KANJI ROM
            b=mzkanji[kanji_ptr++];
        } else {  // JISHO ROM
            kanji_addr=(jisho_ptr++)|(ioport[0xb8]&3)<<16;
            b=mzjisho[kanji_addr];
        }

        if(ioport[0xb8]&0x40) { // Change Endian

            uint8_t rb=0;

            for(int i=0;i<8;i++) {
                if(b&(1<<i)){
                    rb|=(1<<(7-i));
                }
            }   
            b=rb;
        }

        return b;

#endif

        case 0xd8: // FDC is absent.
        case 0xd9:

            return 0xff;

        case 0xea: // RAM FILE

            return ramfile[(ramfile_ptr++)&0xffff];

        case 0xf4: // SIOA DATA

            qd_data=qd_read();

 //           printf("[%02x/%05x/%05x]",qd_data,qd_ptr,qd_blocksize);
 //           printf("[%02x]",qd_data);

            return qd_data;

        case 0xf6: // SIOA

            if(sioa_access) {

                sioa_access=0;

                switch(sioa[0]&7) {

                    case 0:
                        // CTS : Write Protect
                        // DCD : Media present

                    if(qd_status==1) {
                        return 0x2d;
                    } else {
                        return 0x24;
                    }

                    case 1:
                        return 1;
                    case 2:
                        return sioa[2];

                    default:
                        return 0xff;
                }

            } 


            if(qd_status==1) {
                return 0x2d;
            } else {
                return 0x24;
            }

        case 0xf7: // SIOB

            if(siob_access) {

                siob_access=0;

                switch(siob[0]&7) {

                    case 0:

                        // DCD : Home position

                        if(qd_status==1) {
                            return 0xd;
                        } else {
                            return 0x4;
                        }

                    case 1:
                        return 1;
                    case 2:
                        return siob[2];

                    default:
                        return 0xff;
                }

            } 

            return 0xff;






        default:

            break;

    }



  return 0xff;
}

static void io_write(size_t param, ushort address, byte data)
{

  uint8_t addr;
  uint32_t kanji_addr;

  addr=address&0xff;

  switch(addr) {

#ifdef USE_KANJI
    case 0xb9:  // Set Kanji ptr

    kanji_addr=(address&0xff00)+data;

//    printf("[KANJI:%x,%x,%x]",kanji_addr,address,data);

    if(ioport[0xb8]&0x80) { // KANJI ROM
        kanji_ptr=kanji_addr<<5;
    } else {
        jisho_ptr=kanji_addr<<5;
    }
#endif

    // Bank switching

    case 0xe0:
        rombank=1;
        return;
    
    case 0xe1:
        vrambank=1;
        return;

    case 0xe2:
        rombank=0;
        return;

    case 0xe3:
        vrambank=0;
        return;

    case 0xe4:
        rombank=0;
        vrambank=0;
        pcgbank=0;
        return;

    case 0xe5:

        pcgbank=1;
        ioport[addr]=data;        
        return;

    case 0xe6:

        pcgbank=0;
        return;

    case 0xea: // RAM FILE

        ramfile[(ramfile_ptr++)&0xffff]=data;
        return;

    case 0xeb: // RAM FILE Control

        ramfile_ptr=(address&0xff00)+data;
        return;

    case 0xf0: // PCG priotiry

        ioport[addr]=data;
        redraw();
        return;

    case 0xf1:  // Color palette

        palette[(data&0x70)>>4]=data&7;
        rebuild_colorpalette();
        redraw();
        return;
    
    case 0xf2: // PSG1
        psg_write(0,data);
        return;

    case 0xf3: // PSG2
        psg_write(1,data);
        return;


    case 0xe9: // PSG1+2
        psg_write(0,data);
        psg_write(1,data);
        return;

    case 0xf6:

        if(sioa_access==0) {
            sioa[0]=data;
//            if(data&0x7)
             sioa_access=1;
        } else {
            if((sioa[0]&7)==2) {
//                printf("[sioa iv:%d]",data);
            }

            sioa[siob[0]&7]=data;
            sioa_access=0;
        }

        return;

    case 0xf7:

//       printf("[siob:%d,%x]",siob_access,siob[0]);


        if(siob_access==0) {
            siob[0]=data;
//            if(data&0x7)
             siob_access=1;
        } else {

            if((siob[0]&7)==5) {
                if(((siob[5]&0x80)==0)&&(data&0x80)) { // Motor ON
                    qd_motor=1;
                    qd_ptr=0;
                    qd_stage=0;
                    qd_count=0;
                    if(qd_status) {
                        lfs_file_seek(&lfs,&qd_drive,0,LFS_SEEK_SET);
                    }
                    qd_sync=1;
                }
            }

            if((siob[0]&7)==2) {
//                printf("[siob iv:%x]",data);
            }


            siob[siob[0]&7]=data;
            siob_access=0;
        }

        return;

        case 0xfc: // PIOA

            if(pioa_next_mask) {
                pioa[3]=data;
                pioa_next_mask=0;
                return;
            }
            if(pioa_next_iocontrol) {
                pioa_next_iocontrol=0;
                return;
            }

            switch(data&0xf) {

                case 0:
                case 2:
                case 4:
                case 8:
                case 0xa:
                case 0xc:
                case 0xe:

                    pioa[0]=data;
                    return;
                
                case 3: // Intrrupt disable

                    if(data&0x80) {
                        pioa[2]|=0x80;
                    } else {
                        pioa[2]&=0x7f;
                    }

                    return;


                case 7: // Interrupt control

                    pioa[2]=data;
                    if(data&0x10) {
                        pioa_next_mask=1;
                    }
                    return;

                case 0xf: // Mode control

                    if((data&0xc0)==0xc0) { // Mode 3
                        pioa_next_iocontrol=1;
                    }


                default:                

                    return;
            }

        case 0xfd: // PIOA

            if(piob_next_mask) {
                piob[3]=data;
                piob_next_mask=0;
                return;
            }
            if(piob_next_iocontrol) {
                piob_next_iocontrol=0;
                return;
            }


            switch(data&0xf) {

                case 0:
                case 2:
                case 4:
                case 8:
                case 0xa:
                case 0xc:
                case 0xe:

                    piob[0]=data;
                    return;
                
                case 3: // Intrrupt disable

                    if(data&0x80) {
                        piob[2]|=0x80;
                    } else {
                        piob[2]&=0x7f;
                    }

                    return;

                case 7: // Interrupt control

                    piob[2]=data;
                    if(data&0x10) {
                        piob_next_mask=1;
                    }
                    return;

                case 0xf: // Mode control

                    if((data&0xc0)==0xc0) { // Mode 3
                        piob_next_iocontrol=1;
                    }

                default:                

                    return;
            }

    default:

        ioport[addr]=data;
    return;

  }



}


void main_core1(void) {

    uint32_t redraw_start,redraw_length;

    multicore_lockout_victim_init();

    scanline=0;

    // set virtual Hsync timer

    add_repeating_timer_us(63,hsync_handler,NULL  ,&timer);

    while(1) { // Wait framebuffer redraw command


    }
}

int main() {

    uint32_t menuprint=0;
    uint32_t filelist=0;
    uint32_t subcpu_wait;

    static uint32_t hsync_wait,vsync_wait;

    set_sys_clock_khz(DOTCLOCK * CLOCKMUL ,true);

    stdio_init_all();

    uart_init(uart0, 115200);

    initVGA();

    gpio_set_function(12, GPIO_FUNC_UART);
    gpio_set_function(13, GPIO_FUNC_UART);

    // gpio_set_slew_rate(0,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(1,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(2,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(3,GPIO_SLEW_RATE_FAST);
    // gpio_set_slew_rate(4,GPIO_SLEW_RATE_FAST);

    gpio_set_drive_strength(2,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(3,GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_drive_strength(4,GPIO_DRIVE_STRENGTH_2MA);

    // Beep & PSG

    gpio_set_function(6,GPIO_FUNC_PWM);
    gpio_set_function(7,GPIO_FUNC_PWM);
    pwm_slice_num = pwm_gpio_to_slice_num(6);

    pwm_set_wrap(pwm_slice_num, 256);
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_A, 0);
    pwm_set_chan_level(pwm_slice_num, PWM_CHAN_B, 0);
    pwm_set_enabled(pwm_slice_num, true);

    // set PSG timer

    add_repeating_timer_us(1000000/SAMPLING_FREQ,sound_handler,NULL  ,&timer2);

    tuh_init(BOARD_TUH_RHPORT);


    video_cls();

    video_hsync=0;
    video_vsync=0;

    video_mode=0;
    fbcolor=0x7;

// uart handler

    irq_set_exclusive_handler(UART0_IRQ,uart_handler);
    irq_set_enabled(UART0_IRQ,true);
    uart_set_irq_enables(uart0,true,false);

    multicore_launch_core1(main_core1);

    multicore_lockout_victim_init();

    sleep_ms(1);

// mount littlefs
    if(lfs_mount(&lfs,&PICO_FLASH_CFG)!=0) {
       cursor_x=0;
       cursor_y=0;
       fbcolor=7;
       video_print("Initializing LittleFS...");
       // format
       lfs_format(&lfs,&PICO_FLASH_CFG);
       lfs_mount(&lfs,&PICO_FLASH_CFG);
   }

    menumode=1;  // Pause emulator

//  setup emulator 
// QD Init

    qd_motor=0;
    qd_ptr=0;
    siob[5]=0;

//

    for(int i=0;i<8;i++) {
        palette[i]=i;
    }
    rebuild_colorpalette();
 
    psg_reset(0);

    for(int i=0;i<10;i++) {
        keymatrix[i]=0xff;
    }

    i8253_preload[0]=0;
    tape_ready=0;

#ifdef USE_NEWMON

    for(int i=0x800;i<0xbff;i++){
        vram[i]=0x71;
    }
    memioport[2]=8;

#endif


    cpu.memRead = mem_read;
    cpu.memWrite = mem_write;
    cpu.ioRead = io_read;
    cpu.ioWrite = io_write;

    Z80RESET(&cpu);

    uint32_t cpuwait=0;

    // start emulator

    menumode=0;

    while(1) {

        if(menumode==0) { // Emulator mode

        exec_reti=0;

        Z80Execute(&cpu);
        cpu_clocks++;

        // Mode 2 Interrupt (MZ-1500)

        if(exec_reti) { // clear irq on Z80PIO/SIO

            if(pio_irq_processing) {
                pio_irq_processing=0;
                pioa_enable_irq=0;
                piob_enable_irq=0;
            }

        }

        if((pioa_enable_irq)&&(pio_irq_processing==0)) {

            if((cpu.IFF1)&&(cpu.IM==2)) { 
                    Z80INT(&cpu,pioa[0]);
                    pio_irq_processing=1;
                    
                }
    
        }

        // if(qd_read==1) {
        //     qd_read==2;
        //     Z80INT(&cpu,siob[2]&0xfe);
        // }


        // Mode 1 Interrupt (MZ-700)

        if(i8253_enable_irq) {
            Z80INT(&cpu,0x38);
            i8253_enable_irq=0; 
        }

        if(video_hsync==1) {
            hsync_wait++;
            if(hsync_wait>30) {
                video_hsync=0;
                hsync_wait=0;
            }
        }
        
        if((video_vsync)==1) { // Timer
            tuh_task();
            video_vsync=2;
            vsync_scanline=scanline;
        }

        if(video_vsync==2) {
            if(scanline>(vsync_scanline+60)) {
                video_vsync=0;
            }
        }

        } else { // Menu Mode


            unsigned char str[80];

            fbcolor=7;
            
            if(menuprint==0) {

                draw_menu();
                menuprint=1;
                filelist=0;
            }

            cursor_x=3;
            cursor_y=6;
            video_print("MENU");

            uint32_t used_blocks=lfs_fs_size(&lfs);
            sprintf(str,"Free:%d Blocks",(HW_FLASH_STORAGE_BYTES/BLOCK_SIZE_BYTES)-used_blocks);
            cursor_x=3;
            cursor_y=7;
            video_print(str);

            sprintf(str,"TAPE:%x",tape_ptr);
            cursor_x=3;
            cursor_y=8;
            video_print(str);

            cursor_x=3;            
            cursor_y=9;
            if(menuitem==0) { fbcolor=0x70; } else { fbcolor=7; } 
            if(save_enabled==0) {
                video_print("SAVE: empty");
            } else {
                sprintf(str,"SAVE: %8s",tape_filename);
                video_print(str);
            }
            cursor_x=3;
            cursor_y=10;

            if(menuitem==1) { fbcolor=0x70; } else { fbcolor=7; } 
            if(load_enabled==0) {
                video_print("LOAD: empty");
            } else {
                sprintf(str,"LOAD: %8s",tape_filename);
                video_print(str);
            }

            cursor_x=3;
            cursor_y=11;

            if(menuitem==2) { fbcolor=0x70; } else { fbcolor=7; } 
            if(qd_status==0) {
                video_print("QD: empty");
            } else {
                sprintf(str,"QD: %8s",qd_filename);
                video_print(str);
            }

            cursor_x=3;
            cursor_y=13;

            if(menuitem==3) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("DELETE File");

            cursor_x=3;
            cursor_y=16;

            if(menuitem==4) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("Reset");


            cursor_x=3;
            cursor_y=17;

            if(menuitem==5) { fbcolor=0x70; } else { fbcolor=7; } 
            video_print("PowerCycle");

// TEST

            // cursor_x=3;
            //  cursor_y=17;
            //      sprintf(str,"%04x %ld %x %x %x %x",cpu.PC,cpu_clocks,i8253[0],i8253_counter[0],i8253[2],i8253_counter[2]);
            //      video_print(str);

            // cursor_x=3;
            //  cursor_y=17;
            //      sprintf(str,"%04x %ld %d %d",cpu.PC,psg_note_count,psg_master_volume);
            //      video_print(str);

            if(filelist==0) {
                draw_files(-1,0);
                filelist=1;
            }
     
            while(video_vsync==0);

            video_vsync=0;

                tuh_task();

                if(keypressed==0x52) { // Up
                    keypressed=0;
                    if(menuitem>0) menuitem--;
                }

                if(keypressed==0x51) { // Down
                    keypressed=0;
                    if(menuitem<5) menuitem++; 
                }

                if(keypressed==0x28) {  // Enter
                    keypressed=0;

                    if(menuitem==0) {  // SAVE

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=enter_filename();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDWR|LFS_O_CREAT);
                                save_enabled=1;
                                tape_phase=0;
                                tape_ptr=0;
                                tape_count=0;
                            }

                        } else if (save_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            save_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==1) { // LOAD

                        if((load_enabled==0)&&(save_enabled==0)) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(tape_filename,filename,16);
                                lfs_file_open(&lfs,&lfs_file,tape_filename,LFS_O_RDONLY);
                                load_enabled=1;
                                tape_phase=0;
                                tape_ptr=0;
                                tape_count=0;
//                                file_cycle=cpu.PC;
                            }
                        } else if(load_enabled!=0) {
                            lfs_file_close(&lfs,&lfs_file);
                            load_enabled=0;
                        }
                        menuprint=0;
                    }

                    if(menuitem==2) { // QD

                        if(qd_status==0) {

                            uint32_t res=file_selector();

                            if(res==0) {
                                memcpy(qd_filename,filename,16);
                                lfs_file_open(&lfs,&qd_drive,qd_filename,LFS_O_RDONLY);
                                qd_check();
                                qd_status=1;
                            }
                        } else {
                            lfs_file_close(&lfs,&qd_drive);
                            qd_status=0;
                        }
                        menuprint=0;

                    }

                    if(menuitem==3) { // Delete

                        if((load_enabled==0)&&(save_enabled==0)) {
                            uint32_t res=enter_filename();

                            if(res==0) {
                                lfs_remove(&lfs,filename);
                            }
                        }

                        menuprint=0;

                    }

                    if(menuitem==4) { // Reset
                        menumode=0;
                        menuprint=0;
                        redraw();
                    
                        psg_reset(0);

                        Z80RESET(&cpu);

                    }

                    if(menuitem==5) { // PowerCycle
                        menumode=0;
                        menuprint=0;

                        for(int i=0;i<8;i++) {
                             palette[i]=i;
                        }
                        rebuild_colorpalette();

                        memset(mainram,0,0x10000);
                        memset(vram,0,0x1000);
                        memset(ioport,0,0x100);
                        memset(memioport,0,0x100);
                        
                        vrambank=0;
                        rombank=0;

                        redraw();

                        psg_reset(0);

                        Z80RESET(&cpu);

                    }



                }

                if(keypressed==0x45) {
                    keypressed=0;
                    menumode=0;
                    menuprint=0;
                    redraw();
                //  break;     // escape from menu
                }

        }


    }

}
