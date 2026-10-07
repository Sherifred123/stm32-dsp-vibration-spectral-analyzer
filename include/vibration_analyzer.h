/**
 * @file vibration_analyzer.h
 * @brief High-Level Condition Monitoring & Vibration Spectrum Orchestrator.
 *
 * Coordinates ADC ping-pong DMA acquisition, CMSIS-DSP FFT execution,
 * ISO 10816-3 severity evaluation, and real-time telemetry streaming over USART2.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef VIBRATION_ANALYZER_H
#define VIBRATION_ANALYZER_H

#include <stdint.h>
#include <stdbool.h>
#include "dsp_config.h"
#include "dma_pingpong.h"
#include "fft_engine.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    dma_pingpong_t     dma;
    fft_engine_t       fft;
    vibration_report_t latest_report;
    uint32_t           total_processed_frames;
    uint32_t           execution_time_us;     /**< Microseconds to execute 1024-point FFT */
} vibration_analyzer_t;

/**
 * @brief Initialize all vibration analysis subsystems.
 * @param analyzer Pointer to analyzer context.
 */
void vibration_analyzer_init(vibration_analyzer_t *analyzer);

/**
 * @brief Ingest a fresh frame of raw ADC data (for simulation or direct ISR handoff).
 * @param analyzer Pointer to analyzer context.
 * @param raw_adc Array of 1024 ADC samples.
 */
void vibration_analyzer_feed_samples(vibration_analyzer_t *analyzer, const uint16_t *raw_adc);

/**
 * @brief Process ready acquisition frame if available.
 * @param analyzer Pointer to analyzer context.
 * @return true if a fresh FFT spectrum was analyzed, false if waiting for DMA buffer.
 */
bool vibration_analyzer_process(vibration_analyzer_t *analyzer);

/**
 * @brief Print formatted ASCII spectral analysis and harmonic peaks to console.
 * @param analyzer Pointer to analyzer context.
 */
void vibration_analyzer_print_report(const vibration_analyzer_t *analyzer);

#ifdef __cplusplus
}
#endif

#endif /* VIBRATION_ANALYZER_H */

/* Time-Domain Trapezoidal Integration calibration constants for ISO 10816-3 */
