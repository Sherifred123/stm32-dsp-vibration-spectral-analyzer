/**
 * @file dma_pingpong.h
 * @brief Zero-CPU Dual-Buffer (Ping-Pong) DMA Data Acquisition Manager.
 *
 * Implements hardware-driven double-buffering matching STM32 DMA2 Stream 0
 * circular double-buffer mode (DMA_SxCR_DBM). While the ADC fills one buffer
 * via DMA, the Cortex-M4 core executes 1024-point FFT on the alternate buffer.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef DMA_PINGPONG_H
#define DMA_PINGPONG_H

#include <stdint.h>
#include <stdbool.h>
#include "dsp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DMA_BUFFER_0 = 0,
    DMA_BUFFER_1 = 1
} dma_buffer_id_t;

/**
 * @brief Dual-Buffer DMA Manager Context
 */
typedef struct {
    uint16_t buffer_0[DSP_FFT_SIZE];   /**< Memory Target 0 */
    uint16_t buffer_1[DSP_FFT_SIZE];   /**< Memory Target 1 */
    dma_buffer_id_t active_target;     /**< Buffer currently being written by DMA */
    dma_buffer_id_t ready_buffer;      /**< Buffer currently ready for DSP processing */
    bool            has_new_frame;     /**< True when a complete 1024-sample buffer is ready */
    uint32_t        frames_acquired;   /**< Total frames captured */
    uint32_t        frames_dropped;    /**< Overrun counter if DSP cannot keep up */
} dma_pingpong_t;

/**
 * @brief Initialize double-buffer acquisition engine.
 * @param ctx Pointer to double-buffer context.
 */
void dma_pingpong_init(dma_pingpong_t *ctx);

/**
 * @brief Hardware ISR Callback triggered upon DMA Half-Transfer or Transfer-Complete.
 *
 * Automatically called when DMA finishes filling a 1024-sample frame.
 * Swaps active target in hardware and flags the newly filled buffer for processing.
 *
 * @param ctx Pointer to double-buffer context.
 */
void dma_pingpong_transfer_complete_isr(dma_pingpong_t *ctx);

/**
 * @brief Query if a complete frame of 1024 ADC samples is ready for processing.
 * @param ctx Pointer to double-buffer context.
 * @param out_samples Output pointer set to the address of the filled sample array.
 * @return true if fresh frame is available, false otherwise.
 */
bool dma_pingpong_get_ready_frame(dma_pingpong_t *ctx, const uint16_t **out_samples);

/**
 * @brief Release the processed buffer back to the acquisition pool.
 * @param ctx Pointer to double-buffer context.
 */
void dma_pingpong_release_frame(dma_pingpong_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DMA_PINGPONG_H */
