/**
 * @file dump.h
 * @brief Functions to dump the flash to SD card and verify
 */

#ifndef __DUMP_H__
#define __DUMP_H__

#include "../lib/spif/spif.h"

int dump_and_format_flash(SPIF_HandleTypeDef *spif);

#endif // __DUMP_H__
