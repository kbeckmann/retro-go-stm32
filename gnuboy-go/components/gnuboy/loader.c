#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#include <assert.h>

#include "defs.h"
#include "regs.h"
#include "mem.h"
#include "hw.h"
#include "lcd.h"
#include "rtc.h"
#include "cpu.h"
#include "sound.h"
#include "main.h"

#ifdef __linux__
#define LINUX_EMU
#endif

#ifndef LINUX_EMU
#include "gw_linker.h"
#endif

#ifdef LINUX_EMU
const uint32_t _GB_ROM_UNPACK_BUFFER_SIZE=512000;
static uint8_t  _GB_ROM_UNPACK_BUFFER[512000];
#endif

#include "lz4_depack.h"
#include  "miniz.h"

#include "rom_manager.h"

#include <odroid_system.h>

static const byte mbc_table[256] =
{
	0, 1, 1, 1, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 3,
	3, 3, 3, 3, 0, 0, 0, 0, 0, 5, 5, 5, MBC_RUMBLE, MBC_RUMBLE, MBC_RUMBLE, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,

	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, MBC_HUC3, MBC_HUC1
};

static const byte rtc_table[256] =
{
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
	1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0
};

static const byte batt_table[256] =
{
	0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0,
	1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0,
	0
};

static const short romsize_table[256] =
{
	2, 4, 8, 16, 32, 64, 128, 256, 512,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 128, 128, 128
	/* 0, 0, 72, 80, 96  -- actual values but bad to use these! */
};

static const byte ramsize_table[] =
{
	1, 1, 1, 4, 16, 8,
};

#define _fread(buffer, size, nmemb)                               \
	do {                                                          \
		memcpy((buffer), (ptr), (size) * (nmemb));                \
		ptr += (size) * (nmemb);                                  \
	} while(0)

// TODO: Jeopardy: What is bounds checking
#define _fwrite(_buffer, _size, _nmemb)                           \
	do {                                                          \
		printf("_fwrite(%p, %d)\n", (ptr), (_size) * (_nmemb)); \
		memcpy((ptr), (_buffer), (_size) * (_nmemb));             \
		ptr += (_size) * (_nmemb);                                \
	} while(0)

#ifdef IS_LITTLE_ENDIAN
#define LIL(x) (x)
#else
#define LIL(x) ((x<<24)|((x&0xff00)<<8)|((x>>8)&0xff00)|(x>>24))
#endif

#define I1(s, p) { 1, s, p }
#define I2(s, p) { 2, s, p }
#define I4(s, p) { 4, s, p }
#define R(r) I1(#r, &R_##r)
#define NOSAVE { -1, "\0\0\0\0", 0 }
#define END { 0, "\0\0\0\0", 0 }

#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))

typedef struct
{
	int len;
	char key[4];
	void *ptr;
} svar_t;

static int ver;
static int sramblock, iramblock, vramblock;
static int hiofs, palofs, oamofs, wavofs;

static svar_t svars[] =
{
	I4("GbSs", &ver),

	I2("PC  ", &PC),
	I2("SP  ", &SP),
	I2("BC  ", &BC),
	I2("DE  ", &DE),
	I2("HL  ", &HL),
	I2("AF  ", &AF),

	I4("IME ", &cpu.ime),
	I4("ima ", &cpu.ima),
	I4("spd ", &cpu.speed),
	I4("halt", &cpu.halt),
	I4("div ", &cpu.div),
	I4("tim ", &cpu.timer),
	I4("lcdc", &lcd.cycles),
	I4("snd ", &snd.cycles),

	I1("ints", &hw.ilines),
	I1("pad ", &hw.pad),
	I4("cgb ", &hw.cgb),
	I4("gba ", &hw.gba),

	I4("mbcm", &mbc.model),
	I4("romb", &mbc.rombank),
	I4("ramb", &mbc.rambank),
	I4("enab", &mbc.enableram),
	I4("batt", &mbc.batt),

	I4("rtcR", &rtc.sel),
	I4("rtcL", &rtc.latch),
	I4("rtcF", &rtc.flags),
	I4("rtcd", &rtc.d),
	I4("rtch", &rtc.h),
	I4("rtcm", &rtc.m),
	I4("rtcs", &rtc.s),
	I4("rtct", &rtc.ticks),
	I1("rtR8", &rtc.regs[0]),
	I1("rtR9", &rtc.regs[1]),
	I1("rtRA", &rtc.regs[2]),
	I1("rtRB", &rtc.regs[3]),
	I1("rtRC", &rtc.regs[4]),

	I4("S1on", &snd.ch[0].on),
	I4("S1p ", &snd.ch[0].pos),
	I4("S1c ", &snd.ch[0].cnt),
	I4("S1ec", &snd.ch[0].encnt),
	I4("S1sc", &snd.ch[0].swcnt),
	I4("S1sf", &snd.ch[0].swfreq),

	I4("S2on", &snd.ch[1].on),
	I4("S2p ", &snd.ch[1].pos),
	I4("S2c ", &snd.ch[1].cnt),
	I4("S2ec", &snd.ch[1].encnt),

	I4("S3on", &snd.ch[2].on),
	I4("S3p ", &snd.ch[2].pos),
	I4("S3c ", &snd.ch[2].cnt),

	I4("S4on", &snd.ch[3].on),
	I4("S4p ", &snd.ch[3].pos),
	I4("S4c ", &snd.ch[3].cnt),
	I4("S4ec", &snd.ch[3].encnt),

	I4("hdma", &hw.hdma),

	I4("sram", &sramblock),
	I4("iram", &iramblock),
	I4("vram", &vramblock),
	I4("hi  ", &hiofs),
	I4("pal ", &palofs),
	I4("oam ", &oamofs),
	I4("wav ", &wavofs),

	END
};


#define BANK_SIZE 0x4000

#ifdef GB_CACHE_ROM

#define BANK_NUM  32
uint8_t banks[BANK_NUM][BANK_SIZE];

#endif

/* Information to support Compressed ROM using SRAM as cache */
/*************************************************************/

enum {
    COMPRESSION_NONE,
    COMPRESSION_LZ4,
    COMPRESSION_DEFLATE,
};
typedef uint8_t compression_t;

static bool rom_bank_cache_enabled;
static compression_t rom_comp_type;

/* SRAM memory :  ROM bank cache */
unsigned char *GB_ROM_SRAM_CACHE;

/*LZ4 compressed ROM */
static const unsigned char *GB_ROM_LZ4;

/* Maximum number of bank stored in cache (structure) */
#define _MAX_GB_ROM_BANK_IN_CACHE 32

/* Maximum number of bank for G/GBC ROMS */
#define _MAX_GB_ROM_BANKS 512

/* Maximum number of bank can be stored in cache using SRAM */
static uint8_t bank_cache_size = 0;

/* Number of banks in ROM */
static short rom_banks_number =0;

/* return cache idx from bank
if _NOT_IN_CACHE, the bank is not in cache
if _NOT_COMPRESSED, the bank is directly available in ROM
*/
#define _NOT_IN_CACHE   0x80
#define _NOT_COMPRESSED 0x40
static uint8_t bank_to_cache_idx[_MAX_GB_ROM_BANKS];

/* offset of compressed bank (up to 512 banks)
format : lz4_compressed_size + lz4_block
If lz4_compressed_size MSB bit : lz4_block is the bank uncompressed
if 0   : no bank */
static uint32_t gb_rom_lz4_bank_offset[_MAX_GB_ROM_BANKS];

/* cache timestamp
if timestamp is 0, the bank is not in cache */
static uint32_t cache_ts[_MAX_GB_ROM_BANK_IN_CACHE];

/* To enable cache trace: D:DIRECT H:HIT F:FREE S:SWAP */
#define _TRACE_GB_CACHE

#ifdef _TRACE_GB_CACHE
	static uint32_t swap_count=0;
#endif

/* Function to load bank dynamically from ROM LZ4 compressed
the bank0 is always uncompressed */
static void
rom_loadbank_cache(short bank)
{
	/* offset in memory cache of requested bank
	OFFSET = bank * BANK_SIZE */
	size_t OFFSET;

	/* uncompress bank to cache_idx */
	uint32_t lz4_compressed_size;

	/* reclaimed bank idx in the cache */
	uint8_t reclaimed_idx=0;

	/* last requested idx in cache */
	static uint8_t active_idx = 0;

	/* reclaimed bank */
	short reclaimed_bank=0;

	#ifdef _TRACE_GB_CACHE
		//printf("L:%03d %03d ",bank, bank_to_cache_idx[bank]);
	#endif

	/* THE BANK IS UNCOMPRESSED AND CAN BE READ DIRECTLY IN ROM */
	if (bank_to_cache_idx[bank] & _NOT_COMPRESSED) {
        switch(rom_comp_type){
            case COMPRESSION_LZ4: {
                OFFSET = gb_rom_lz4_bank_offset[bank] + LZ4_FRAME_SIZE;

                /* set the bank address to the right bank address in cache */
                rom.bank[bank] = (unsigned char *)&GB_ROM_LZ4[OFFSET];
                break;
            }
            case COMPRESSION_DEFLATE: {
                OFFSET = gb_rom_lz4_bank_offset[bank] + 5; //header, LEN, NLEN
                rom.bank[bank] = (unsigned char *)&GB_ROM_LZ4[OFFSET];
                break;
            }
        }

		#ifdef _TRACE_GB_CACHE
			//printf("Direct\n");
		#endif
	/* THE BANK IS NOT IN THE CACHE AND IS COMPRESSED */
	} else if (bank_to_cache_idx[bank] & _NOT_IN_CACHE) {
		/* look for the older bank in cache as a candidate */
		for (int idx = 0; idx < bank_cache_size; idx++)
			if (cache_ts[reclaimed_idx] > cache_ts[idx]) reclaimed_idx = idx;

		/* look for the corresponding allocated bank (skip bank0) */
		for (int bank_idx=1; bank_idx < rom_banks_number; bank_idx++)
			if (bank_to_cache_idx[bank_idx] == reclaimed_idx) reclaimed_bank = bank_idx;

		/* reclaim the removed bank from the cache if necessary */
		if ( (bank_to_cache_idx[reclaimed_bank] <= bank_cache_size)  &  (reclaimed_bank !=0)) {
			bank_to_cache_idx[reclaimed_bank] 	= _NOT_IN_CACHE;
			rom.bank[reclaimed_bank] 			= NULL;

		#ifdef _TRACE_GB_CACHE
			printf("S -bank%03d +bank%03d cch=%02d TS=%d\n",reclaimed_bank,bank, reclaimed_idx, cache_ts[reclaimed_idx]);
			swap_count++;

		} else {
			printf("F +bank%03d cch=%02d TS=%d\n",bank, reclaimed_idx, cache_ts[reclaimed_idx]);
		#endif
		}

		/* allocate the requested bank in cache */
		bank_to_cache_idx[bank] = reclaimed_idx;
        OFFSET = reclaimed_idx * BANK_SIZE;

		wdog_refresh();

        switch(rom_comp_type){
            case COMPRESSION_LZ4: {
                memcpy(&lz4_compressed_size, &GB_ROM_LZ4[gb_rom_lz4_bank_offset[bank]], sizeof lz4_compressed_size);
                wdog_refresh();
                lz4_depack(&GB_ROM_LZ4[gb_rom_lz4_bank_offset[bank]+LZ4_FRAME_SIZE],&GB_ROM_SRAM_CACHE[OFFSET],lz4_compressed_size);
                break;
            }
            case COMPRESSION_DEFLATE: {
                size_t n_decomp_bytes;
                n_decomp_bytes = tinfl_decompress_mem_to_mem(&GB_ROM_SRAM_CACHE[OFFSET], BANK_SIZE, &GB_ROM_LZ4[gb_rom_lz4_bank_offset[bank]], ROM_DATA_LENGTH - gb_rom_lz4_bank_offset[bank], 0);
                assert(n_decomp_bytes != TINFL_DECOMPRESS_MEM_TO_MEM_FAILED);
                break;
            }
        }

		/* set the bank address to the right bank address in cache */
		rom.bank[bank] = (unsigned char *)&GB_ROM_SRAM_CACHE[OFFSET];

		/* refresh timestamp and score*/
		cache_ts[reclaimed_idx] = HAL_GetTick();

		active_idx = reclaimed_idx;

	/* HIT CASE: the bank is already in the cache */
	} else {

		active_idx = bank_to_cache_idx[bank];

		OFFSET = active_idx * BANK_SIZE;

		/* set the bank address to the right bank address in cache */
		rom.bank[bank] = (unsigned char *)&GB_ROM_SRAM_CACHE[OFFSET];

		/* refresh timestamp and score */
		cache_ts[active_idx] = HAL_GetTick();

		#ifdef _TRACE_GB_CACHE
			//printf("H bnk=%02d cch=%02d\n", bank, bank_to_cache_idx[bank]);
		#endif
	}

	#ifdef _TRACE_GB_CACHE
	//just to break using BSOD
	//	if (swap_count > 200) assert(0);
	#endif

}

/* function used to restore the SRAM cache memory
when the cache memory was stolen temporary for another operation (like save_state) */

void gb_loader_restore_cache() {


	/* bank to be refreshed in the cache */
	uint32_t bank=0;

	/* Refresh all cache memory parsing the cache index */
	for (int restored_idx = 0; restored_idx < bank_cache_size; restored_idx++) {

		/* default value if the following loop doesn't find a match */
		bank=0;

		/* look for the corresponding allocated bank */
		for (int bank_nb=1; bank_nb < rom_banks_number; bank_nb++)
			if (bank_to_cache_idx[bank_nb] == restored_idx) bank = bank_nb;

		/* if no allocated bank, the default value 0, it corresponds to bank0, nothing to do in this case */
		if (bank !=0) {
            /* offset in memory cache of requested bank */
            size_t OFFSET = restored_idx * BANK_SIZE;
            wdog_refresh();
            switch(rom_comp_type){
                case COMPRESSION_LZ4:{
                    uint32_t lz4_compressed_size;

                    /* refresh the bank */
                    memcpy(&lz4_compressed_size, &GB_ROM_LZ4[gb_rom_lz4_bank_offset[bank]], sizeof lz4_compressed_size);
                    lz4_depack(&GB_ROM_LZ4[gb_rom_lz4_bank_offset[bank]+LZ4_FRAME_SIZE],&GB_ROM_SRAM_CACHE[OFFSET],lz4_compressed_size);
                }
                break;
                case COMPRESSION_DEFLATE: {
                    size_t n_decomp_bytes;
                    n_decomp_bytes = tinfl_decompress_mem_to_mem(&GB_ROM_SRAM_CACHE[OFFSET], BANK_SIZE, &GB_ROM_LZ4[gb_rom_lz4_bank_offset[bank]], ROM_DATA_LENGTH - gb_rom_lz4_bank_offset[bank], 0);
                    assert(n_decomp_bytes != TINFL_DECOMPRESS_MEM_TO_MEM_FAILED);
                }
                break;
            }

		}
	}
}

// TODO: Revisit this later as memory might run out when loading

int IRAM_ATTR rom_loadbank(short bank)
{
	const size_t OFFSET = bank * BANK_SIZE;

#ifdef GB_CACHE_ROM
	printf("bank_load: loading bank %d.\n", bank);
	rom.bank[bank] = banks[bank % BANK_NUM];
	if (rom.bank[bank] == NULL) {
		for (int i = BANK_NUM-1; i > 0; i--) {
			if (rom.bank[i]) {
				printf("bank_load: reclaiming bank %d.\n", i);
				rom.bank[bank] = rom.bank[i];
				rom.bank[i] = NULL;
				break;
			}
		}
	}


	// TODO: check bounds (famous last words :D)
	memcpy(rom.bank[bank], &ROM_DATA[OFFSET], BANK_SIZE);
#else
	// cached
	if (rom_bank_cache_enabled)
		rom_loadbank_cache(bank);
	// uncached
	else
		rom.bank[bank] = &ROM_DATA[OFFSET];
#endif
	return 0;
}

//uint8_t sram[8192];
uint8_t sram[8192 * 16];


static void gb_rom_compress_load(){
    /* src pointer to the ROM data in the external flash (raw or LZ4) */
    const unsigned char *src = ROM_DATA;
	rom_bank_cache_enabled = false;

    if (memcmp(&src[0], LZ4_MAGIC, LZ4_MAGIC_SIZE) == 0) rom_comp_type = COMPRESSION_LZ4;
    else if(strcmp(ROM_EXT, "zopfli") == 0) rom_comp_type = COMPRESSION_DEFLATE;
    else rom_comp_type = COMPRESSION_NONE;

    if(rom_comp_type == COMPRESSION_NONE) return;

    /* dest pointer to the ROM data in the internal RAM (raw) */
    unsigned char *dest = (unsigned char *)&_GB_ROM_UNPACK_BUFFER;
    uint32_t available_size = (uint32_t)&_GB_ROM_UNPACK_BUFFER_SIZE;

    GB_ROM_LZ4        = (unsigned char *)src;
    GB_ROM_SRAM_CACHE = (unsigned char *)dest;

    printf("Compressed ROM detected #%d\n", ROM_DATA_LENGTH);
    printf("Uncompressing to %p. %ld bytes available.\n", dest, available_size);

    bank_cache_size = available_size / BANK_SIZE;

    /* Under Linux emulation, Force rhe cache size to match the embedded target */
    #ifdef LINUX_EMU
        bank_cache_size=26;
    #endif

    if (bank_cache_size > _MAX_GB_ROM_BANK_IN_CACHE) bank_cache_size = _MAX_GB_ROM_BANK_IN_CACHE-1;

    printf("SRAM cache size : %ld banks\n", bank_cache_size);

    /* parse compressed ROM to determine:
    - number of banks (16KB trunks)
    - banks offset as a compressed chunk
    */

    /* clean up cache information */
    memset(bank_to_cache_idx, _NOT_IN_CACHE, sizeof bank_to_cache_idx);
    memset(cache_ts, 0, sizeof cache_ts);
    memset(gb_rom_lz4_bank_offset, 0, sizeof( gb_rom_lz4_bank_offset));
    //memset(cache_score,SCORE_DOWN,sizeof cache_score);
    
    uint32_t bank_idx = 0;

    switch(rom_comp_type){
        case COMPRESSION_LZ4: {
            /*
            Parse all LZ4 frames and check it as compressed bank
            check header compressed size    	  : bank_compressed_size
            check header uncompressed size  	  : bank_uncompressed_size
            Determine offset in LZ4 for each bank : gb_rom_bank_offset
            Decompress all banks for testing the size coherency with lz4_decompressed_size
            */

            /* LZ4 frame */
            uint32_t header_size = LZ4_MAGIC_SIZE + LZ4_FLG_SIZE + LZ4_BD_SIZE;
            uint32_t lz4_offset = header_size;
            uint32_t lz4_compressed_size = 0;
            uint32_t lz4_uncompressed_size = 0;
            uint32_t lz4_result_size = 0;

            while (lz4_offset < ROM_DATA_LENGTH) {

                memcpy(&lz4_uncompressed_size, &GB_ROM_LZ4[lz4_offset], sizeof lz4_uncompressed_size);
                lz4_offset += LZ4_CONTENT_SIZE + LZ4_HC_SIZE;

                /* store Bank offset Tables */
                gb_rom_lz4_bank_offset[bank_idx] = lz4_offset;

                memcpy(&lz4_compressed_size, &GB_ROM_LZ4[lz4_offset], sizeof lz4_compressed_size);
                lz4_offset += LZ4_FRAME_SIZE;

                /* check that the header information is correct */
                assert( lz4_uncompressed_size == BANK_SIZE );

                if ( (lz4_compressed_size & 0x80000000) != 0) {

                    lz4_compressed_size &= 0x7FFFFFFF;
                    lz4_result_size = lz4_compressed_size;
                    bank_to_cache_idx[bank_idx] = _NOT_COMPRESSED;
                } else {
                    wdog_refresh();
                    // use GB_ROM_SRAM_CACHE to check all compressed bank using LZ4 depack */
                    lz4_result_size = lz4_depack(&GB_ROM_LZ4[lz4_offset],&GB_ROM_SRAM_CACHE[0],lz4_compressed_size);
                }

                /* check that the decompressed bank size is correct */
                assert ( BANK_SIZE == lz4_result_size );

                /* next LZ4 frame */
                bank_idx++;
                lz4_offset += LZ4_ENDMARK_SIZE +  lz4_compressed_size + header_size;
            }
            break;
        }
        case COMPRESSION_DEFLATE: {
            tinfl_decompressor decomp;
            tinfl_status status;
            uint32_t src_offset = 0;

            int flags = 0;
            flags |= TINFL_FLAG_USING_NON_WRAPPING_OUTPUT_BUF;

            for(bank_idx=0; src_offset < ROM_DATA_LENGTH; bank_idx++){
                wdog_refresh();
                uint32_t src_buf_size = ROM_DATA_LENGTH - src_offset; 
                tinfl_init(&decomp);

                uint32_t dst_buf_size = available_size;

                gb_rom_lz4_bank_offset[bank_idx] = src_offset;

                status = tinfl_decompress(&decomp,
                        &GB_ROM_LZ4[src_offset], &src_buf_size,
                        &GB_ROM_SRAM_CACHE[0], &GB_ROM_SRAM_CACHE[0], &dst_buf_size,
                        flags
                        );
                assert(status == TINFL_STATUS_DONE);
                assert(dst_buf_size == BANK_SIZE);

                /* Explicitly check the 3 bit header */
                if((GB_ROM_LZ4[src_offset] & 0x06) == 0){
                    // Not Compressed
                    bank_to_cache_idx[bank_idx] = _NOT_COMPRESSED;
                }

                // src_buf_size now contains how many bytes of the src were transversed
                src_offset += src_buf_size;
            }
            break;
        }
    }
    rom_banks_number       = bank_idx;
    rom_bank_cache_enabled = true;
    printf("LZ4 ROM checked!\n");
}

static int gb_rom_load()
{
	gb_rom_compress_load();

	rom_loadbank(0);

	byte *header = rom.bank[0];

	memcpy(rom.name, header + 0x0134, 16);
	rom.name[16] = 0;

	int tmp = *((int*)(header + 0x0140));
	byte c = tmp >> 24;
	hw.cgb = ((c == 0x80) || (c == 0xc0));
	hw.gba = (hw.cgb && 0);

	tmp = *((int*)(header + 0x0144));
	c = (tmp >> 24) & 0xff;
	mbc.type = mbc_table[c];
	mbc.batt = batt_table[c];
	mbc.rtc = rtc_table[c];

	tmp = *((int*)(header + 0x0148));
	mbc.romsize = romsize_table[(tmp & 0xff)];

	uint8_t ramsize_idx = ((tmp >> 8) & 0xff);
	assert(ramsize_idx <= 5);
	mbc.ramsize = ramsize_table[ramsize_idx];

	rom.length = 16384 * mbc.romsize;

	memcpy(&rom.checksum, header + 0x014E, 2);

	if (!mbc.romsize) emu_die("ROM size == 0");
	if (!mbc.ramsize) emu_die("SRAM size == 0");

	const char* mbcName;
	switch (mbc.type)
	{
		case MBC_NONE:   mbcName = "MBC_NONE"; break;
		case MBC_MBC1:   mbcName = "MBC_MBC1"; break;
		case MBC_MBC2:   mbcName = "MBC_MBC2"; break;
		case MBC_MBC3:   mbcName = "MBC_MBC3"; break;
		case MBC_MBC5:   mbcName = "MBC_MBC5"; break;
		case MBC_RUMBLE: mbcName = "MBC_RUMBLE"; break;
		case MBC_HUC1:   mbcName = "MBC_HUC1"; break;
		case MBC_HUC3:   mbcName = "MBC_HUC3"; break;
		default:         mbcName = "(unknown)"; break;
	}

	printf("loader: rom.name='%s'\n", rom.name);
	printf("loader: mbc.type=%s, mbc.romsize=%d (%dK), mbc.ramsize=%d (%dK)\n",
		mbcName, mbc.romsize, rom.length / 1024, mbc.ramsize, mbc.ramsize * 8);

	// TODO: Revisit this later
	// SRAM
	// ram.sbank = rg_alloc(8192 * mbc.ramsize, MEM_FAST);
	ram.sbank = (unsigned char (*)[8192]) sram;
	ram.sram_dirty = 0;

	assert((8192 * mbc.ramsize) <= sizeof(sram));

	memset(ram.sbank, 0xff, sizeof(sram));
	memset(ram.ibank, 0xff, 4096 * 8);

	mbc.rombank = 1;
	mbc.rambank = 0;

	int preload = mbc.romsize < 64 ? mbc.romsize : 64;

	// RAYMAN stutters too much if we don't fully preload it
	if (strncmp(rom.name, "RAYMAN", 6) == 0)
	{
		printf("loader: Special preloading for Rayman 1/2\n");
		preload = mbc.romsize - 8;
	}

	preload=1;
	printf("loader: Preloading the first %d banks\n", preload);
	for (int i = 1; i < preload; i++)
	{
		rom_loadbank(i);
	}

	// Apply game-specific hacks
	if (strncmp(rom.name, "SIREN GB2 ", 11) == 0 || strncmp(rom.name, "DONKEY KONG", 11) == 0)
	{
		printf("loader: HACK: Window offset hack enabled\n");
		enable_window_offset_hack = 1;
	}

	return 0;
}


int sram_load()
{
	// int ret = -1;
	// FILE *f;

	// if (!mbc.batt || !sramfile || !*sramfile) return -1;

	// odroid_system_spi_lock_acquire(SPI_LOCK_SDCARD);

	// if ((f = fopen(sramfile, "rb")))
	// {
	// 	printf("sram_load: Loading SRAM\n");
	// 	_fread(ram.sbank, 8192, mbc.ramsize, f);
	// 	rtc_load(f);
	// 	fclose(f);
	// 	ret = 0;
	// }

	// odroid_system_spi_lock_release(SPI_LOCK_SDCARD);
	// return ret;
	return 0;
}


int sram_save()
{
	// int ret = -1;
	// FILE *f;

	// if (!mbc.batt || !sramfile || !mbc.ramsize) return -1;

	// odroid_system_spi_lock_acquire(SPI_LOCK_SDCARD);

	// if ((f = fopen(sramfile, "wb")))
	// {
	// 	printf("sram_load: Saving SRAM\n");
	// 	_fwrite(ram.sbank, 8192, mbc.ramsize, f);
	// 	rtc_save(f);
	// 	fclose(f);
	// 	ret = 0;
	// }

	// odroid_system_spi_lock_release(SPI_LOCK_SDCARD);
	// return ret;
	return 0;
}


static uint8_t scratch_buf[4096];

int gb_state_save(uint8_t *flash_ptr, size_t size)
{
	uint8_t *base_ptr = flash_ptr;
	uint8_t *ptr = flash_ptr;
	uint8_t *buf = scratch_buf;

	printf("gb_state_save: base=%p\n", flash_ptr);

	if(size < 24000) {
		return -1;
	}

	int i;

	un32 (*header)[2] = (un32 (*)[2])buf;
	un32 d = 0;
	int irl = hw.cgb ? 8 : 2;
	int vrl = hw.cgb ? 4 : 2;
	int srl = mbc.ramsize << 1;

	ver = 0x106;
	iramblock = 1;
	vramblock = 1+irl;
	sramblock = 1+irl+vrl;
	wavofs = 4096 - 784;
	hiofs = 4096 - 768;
	palofs = 4096 - 512;
	oamofs = 4096 - 256;
	memset(buf, 0, 4096);

	for (i = 0; svars[i].len > 0; i++)
	{
		header[i][0] = *(un32 *)svars[i].key;
		switch (svars[i].len)
		{
		case 1:
			d = *(byte *)svars[i].ptr;
			break;
		case 2:
			d = *(un16 *)svars[i].ptr;
			break;
		case 4:
			d = *(un32 *)svars[i].ptr;
			break;
		}
		header[i][1] = LIL(d);
	}
	header[i][0] = header[i][1] = 0;

	memcpy(buf+hiofs, ram.hi, sizeof ram.hi);
	memcpy(buf+palofs, lcd.pal, sizeof lcd.pal);
	memcpy(buf+oamofs, lcd.oam.mem, sizeof lcd.oam);
	memcpy(buf+wavofs, snd.wave, sizeof snd.wave);

	_fwrite(buf, 4096, 1);
	_fwrite(ram.ibank, 4096, irl);
	_fwrite(lcd.vbank, 4096, vrl);

	byte* tmp = (byte*)ram.sbank;
	for (int j = 0; j < srl; ++j)
	{
		memcpy(buf, (void*)tmp, 4096);
		_fwrite(buf, 4096, 1);
		printf("gb_state_save: wrote sram addr=%p, size=0x%x\n", (void*)tmp, 4096);
		tmp += 4096;
	}

	printf("gb_state_save: done. ptr=%p\n", ptr);

	return ptr - base_ptr;
}


int gb_state_load(const uint8_t *flash_ptr, size_t size)
{
	const uint8_t *ptr = flash_ptr;
	uint8_t *buf = scratch_buf;

	int i, j;

	un32 (*header)[2] = (un32 (*)[2])buf;
	un32 d;
	int irl = hw.cgb ? 8 : 2;
	int vrl = hw.cgb ? 4 : 2;
	int srl = mbc.ramsize << 1;

	ver = hiofs = palofs = oamofs = wavofs = 0;

	_fread(buf, 4096, 1);

	// Sanity-check header
	// Note: This was added because we don't store a header/checksum
	// TODO: Add a magic value and checksum of the state save (it will break old saves though...)
	for (j = 0; j < ARRAY_SIZE(svars) + 1; j++)
	{
		if (header[j][0] == 0) {
			break;
		}
	}

	if (j != ARRAY_SIZE(svars) - 1) {
		printf("gb_state_load: Invalid state save, %d/%d entries\n", j, ARRAY_SIZE(svars));
		return 0;
	}

	for (j = 0; header[j][0]; j++)
	{
		for (i = 0; svars[i].ptr; i++)
		{
			if (header[j][0] != *(un32 *)svars[i].key)
				continue;
			d = LIL(header[j][1]);
			switch (svars[i].len)
			{
			case 1:
				*(byte *)svars[i].ptr = d;
				break;
			case 2:
				*(un16 *)svars[i].ptr = d;
				break;
			case 4:
				*(un32 *)svars[i].ptr = d;
				break;
			}
			break;
		}
	}

	if (hiofs) memcpy(ram.hi, buf+hiofs, sizeof ram.hi);
	if (palofs) memcpy(lcd.pal, buf+palofs, sizeof lcd.pal);
	if (oamofs) memcpy(lcd.oam.mem, buf+oamofs, sizeof lcd.oam);

	if (wavofs) memcpy(snd.wave, buf+wavofs, sizeof snd.wave);
	else memcpy(snd.wave, ram.hi+0x30, 16); /* patch data from older files */

	iramblock = 1;
	vramblock = 1+irl;
	sramblock = 1+irl+vrl;

	_fread(ram.ibank, 4096, irl);
	_fread(lcd.vbank, 4096, vrl);
	_fread(ram.sbank, 4096, srl);

	printf("state_load: read sram addr=%p, size=0x%x\n", (void*)ram.sbank, 4096 * srl);

	pal_dirty();
	sound_dirty();
	mem_updatemap();

	return 0;
}



void loader_unload()
{
	sram_save();
	if (ram.sbank) free(ram.sbank);

	for (int i = 0; i < 512; i++) {
		if (rom.bank[i]) {
			free(rom.bank[i]);
			rom.bank[i] = NULL;
		}
	}

	mbc.type = mbc.romsize = mbc.ramsize = mbc.batt = mbc.rtc = 0;
	// ram.sbank = NULL;
}


void loader_init(char *s)
{
	gb_rom_load();
	// sram_load();
}
