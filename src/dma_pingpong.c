/**
 * @file dma_pingpong.c
 * @brief Zero-CPU Dual-Buffer DMA Manager Implementation.
 *
 * Implements hardware-aligned double-buffer management matching STM32 DMA2 Stream 0.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "dma_pingpong.h"
#include <string.h>

void dma_pingpong_init(dma_pingpong_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->active_target = DMA_BUFFER_0;
    ctx->ready_buffer = DMA_BUFFER_0;
    ctx->has_new_frame = false;
}

void dma_pingpong_transfer_complete_isr(dma_pingpong_t *ctx)
{
    if (ctx == NULL) {
        return;
    }

    if (ctx->active_target == DMA_BUFFER_0) {
        /* Buffer 0 has finished filling; swap active target to Buffer 1 */
        ctx->ready_buffer = DMA_BUFFER_0;
        ctx->active_target = DMA_BUFFER_1;
    } else {
        /* Buffer 1 has finished filling; swap active target to Buffer 0 */
        ctx->ready_buffer = DMA_BUFFER_1;
        ctx->active_target = DMA_BUFFER_0;
    }

    if (ctx->has_new_frame) {
        /* Previous buffer was not processed before next transfer finished */
        ctx->frames_dropped++;
    }

    ctx->has_new_frame = true;
    ctx->frames_acquired++;
}

bool dma_pingpong_get_ready_frame(dma_pingpong_t *ctx, const uint16_t **out_samples)
{
    if (ctx == NULL || out_samples == NULL || !ctx->has_new_frame) {
        return false;
    }

    if (ctx->ready_buffer == DMA_BUFFER_0) {
        *out_samples = ctx->buffer_0;
    } else {
        *out_samples = ctx->buffer_1;
    }

    return true;
}

void dma_pingpong_release_frame(dma_pingpong_t *ctx)
{
    if (ctx != NULL) {
        ctx->has_new_frame = false;
    }
}
