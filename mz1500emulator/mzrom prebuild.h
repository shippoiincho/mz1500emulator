// Dummy ROM file for Prebuild binary

// ROM Data                 at
// ipl.rom  4KiB    0x10070000
// font.rom 4KiB    0x10074000
// ext.rom  6KiB    0x10078000

#define ROMBASE 0x10070000u

uint8_t *mzipl     =(uint8_t *)(ROMBASE);
uint8_t *mzfont    =(uint8_t *)(ROMBASE+0x4000);
uint8_t *mzextrom  =(uint8_t *)(ROMBASE+0x8000);
