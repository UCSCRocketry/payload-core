#ifndef __SDHC_H__
#define __SDHC_H__

#include "sdhc_spi.h"
#include "log.h"

#define SDHC_MAX_ACMD41_SEND_ATTEMPTS 5

int sdhc_init(struct sdhc_spi_device *dev);
int sdhc_write_data(struct sdhc_spi_device *dev, struct sdhc_data *data);
int sdhc_read_data(struct sdhc_spi_device *dev, struct sdhc_data *data);

#endif // __SDHC_H__