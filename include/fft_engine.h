/**
 * @file fft_engine.h
 * @brief Real Fast Fourier Transform (RFFT) & Spectral Diagnostics Engine.
 *
 * Implements 1024-point Real FFT accelerated by Cortex-M4 hardware FPU,
 * Hanning windowing to suppress spectral leakage, magnitude bin extraction,
 * harmonic peak tracking, and Total Harmonic Distortion (THD) calculation.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef FFT_ENGINE_H
#define FFT_ENGINE_H

#include <stdint.h>
#include <stdbool.h>
#include "dsp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FFT Processing Context
 */
typedef struct {
    float window_lut[DSP_FFT_SIZE];       /**< Precomputed Hanning window array */
    float input_signal[DSP_FFT_SIZE];     /**< Windowed time-domain float buffer */
    float fft_complex_out[DSP_FFT_SIZE];  /**< Complex frequency output [Re, Im] */
    float magnitude_spectrum[DSP_NUM_SPECTRUM_BINS]; /**< Normalized magnitude in Volts */
} fft_engine_t;

/**
 * @brief Initialize the FFT engine and compute Hanning window lookup table.
 * @param engine Pointer to FFT engine context.
 */
void fft_engine_init(fft_engine_t *engine);

/**
 * @brief Convert raw 12-bit ADC integer samples to normalized Volts and apply Hanning window.
 *
 * Transfers ADC integer codes [0 .. 4095] to physical Volts [0.0V .. 3.3V],
 * subtracts DC bias, and applies precomputed raised-cosine Hanning coefficients.
 *
 * @param engine Pointer to FFT engine context.
 * @param raw_adc 1024-element array of 12-bit ADC samples.
 */
void fft_engine_apply_window(fft_engine_t *engine, const uint16_t *raw_adc);

/**
 * @brief Compute 1024-point Real Fast Fourier Transform (RFFT).
 *
 * Executes decimation-in-time FFT on the windowed time-domain buffer,
 * producing real and imaginary complex bins.
 *
 * @param engine Pointer to FFT engine context.
 */
void fft_engine_compute_rfft(fft_engine_t *engine);

/**
 * @brief Calculate normalized magnitude spectrum across 512 frequency bins.
 *
 * Computes |X[k]| = (2 / N) * sqrt(Re^2 + Im^2) for single-sided spectrum.
 *
 * @param engine Pointer to FFT engine context.
 */
void fft_engine_compute_magnitude(fft_engine_t *engine);

/**
 * @brief Extract dominant harmonic peaks and compute Total Harmonic Distortion (THD).
 * @param engine Pointer to FFT engine context.
 * @param report Output pointer to populate with diagnostic findings.
 */
void fft_engine_extract_diagnostics(const fft_engine_t *engine, vibration_report_t *report);

#ifdef __cplusplus
}
#endif

#endif /* FFT_ENGINE_H */
