

#ifndef FLASH_H__
#define FLASH_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nordic_common.h"


static void flash_page_erase(uint32_t * page_address);
static void flash_word_write(uint32_t * address, uint32_t value);
void update_name_flash(const char * newname, uint8_t len);
uint8_t  get_name_flash( char * newname);

#endif