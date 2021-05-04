#ifndef __LOADER_H__
#define __LOADER_H__


typedef struct loader_s
{
	char *rom;
	char *base;
	char *sram;
	char *state;
} loader_t;


extern loader_t loader;

void loader_init(char *s);
void loader_unload();
int rom_loadbank(short);
int rom_load();
int sram_load();
int sram_save();
int gb_state_load(const uint8_t *flash_ptr, size_t size);
int gb_state_save(uint8_t *flash_ptr, size_t size);

/* share CACHE buffer with save state */
extern unsigned char *GB_ROM_SRAM_CACHE;
extern void gb_loader_restore_cache();

#endif
