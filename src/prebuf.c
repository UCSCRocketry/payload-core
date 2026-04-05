/**
 * @file prebuf.c
 * @brief Pre-launch circular buffer
 */

#include "prebuf.h"
#include "log.h"
#include "spif.h"
#include <string.h>

/**
 * @brief Push one sample into the buffer
 *
 * @param s Pointer to the sample to push.
 */
void prebuf_push(struct prebuf *pb, const struct payload_sample *s)
{
    struct payload_sample *prebuf_arr = pb->prebuf;
	uint32_t write_idx = (pb->head + pb->count) % PAYLOAD_PREBUF_DEPTH;
	prebuf_arr[write_idx] = *s;
	if (pb->count < PAYLOAD_PREBUF_DEPTH)
	{
		pb->count++;
	}
	else
	{
		pb->head = (pb->head + 1) % PAYLOAD_PREBUF_DEPTH;
	}
}

/**
 * @brief Write the buffer contents to flash oldest-first.
 *
 * @param spif SPI flash handle.
 * @return Index of the next free flash page after the flush.
 */
uint32_t prebuf_flush(struct prebuf *pb, SPIF_HandleTypeDef *spif)
{
    struct payload_sample *prebuf_arr = pb->prebuf;
	struct payload_page page;
    uint32_t page_idx = 0;

	memset(&page, 0xDE, sizeof(page));
	SPIF_WritePage(spif, page_idx, (uint8_t *) &page, sizeof(page), 0);
	page_idx++;

    // Iterate by four samples
	for (uint32_t i = 0; i < pb->count; i += PAYLOAD_SAMPLES_PER_PAGE)
	{
        // Page should be filled with 1s by default
		memset(&page, 0xFF, sizeof(page));

        // Iterate per sample to transfer into page struct
		for (uint32_t j = 0; (j < PAYLOAD_SAMPLES_PER_PAGE) && ((i + j) < pb->count); j++)
		{
			uint32_t idx = (pb->head + i + j) % PAYLOAD_PREBUF_DEPTH;
			page.samples[j] = prebuf_arr[idx];
		}

        // Perform page write
		if (!SPIF_WritePage(spif, page_idx, (uint8_t *) &page, sizeof(page), 0))
		{
			LOG_ERR("Prebuf flush: write error at page %lu", page_idx);
			break;
		}
        
		page_idx++;
	}

	memset(&page, 0xEF, sizeof(page));
	SPIF_WritePage(spif, page_idx, (uint8_t *) &page, sizeof(page), 0);
	page_idx++;

	LOG_INF("Prebuf flush: flushed %lu samples -> %lu pages", pb->count, page_idx);
	return page_idx;
}
