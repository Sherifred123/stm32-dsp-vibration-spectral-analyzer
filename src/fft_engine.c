/**
 * @file fft_engine.c
 * @brief Fast Fourier Transform & Spectral Feature Extraction Implementation.
 *
 * Implements 1024-point Real FFT, raised-cosine Hanning windowing, coherent gain
 * scaling, harmonic peak detection, and Total Harmonic Distortion (THD) calculation.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "fft_engine.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* Precomputed Bit-Reversal Table for 1024 Points */
static uint16_t s_bit_rev[DSP_FFT_SIZE];
static bool     s_lut_initialized = false;

static uint16_t reverse_bits(uint16_t val, uint8_t bits)
{
    uint16_t result = 0;
    for (uint8_t i = 0; i < bits; i++) {
        if ((val & (1U << i)) != 0U) {
            result |= (uint16_t)(1U << (bits - 1U - i));
        }
    }
    return result;
}

void fft_engine_init(fft_engine_t *engine)
{
    if (engine == NULL) {
        return;
    }

    memset(engine, 0, sizeof(*engine));

    /* Initialize Bit-Reversal Table (10 bits for 1024 points) */
    if (!s_lut_initialized) {
        for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
            s_bit_rev[i] = reverse_bits(i, 10U);
        }
        s_lut_initialized = true;
    }

    /* Compute Hanning Window Coefficients: w[n] = 0.5 * (1 - cos(2*pi*n / (N-1))) */
    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        engine->window_lut[i] = 0.5f * (1.0f - cosf((2.0f * (float)M_PI * (float)i) / (float)(DSP_FFT_SIZE - 1U)));
    }
}

void fft_engine_apply_window(fft_engine_t *engine, const uint16_t *raw_adc)
{
    if (engine == NULL || raw_adc == NULL) {
        return;
    }

    /* 1. Calculate DC Bias Offset (Mean ADC Count) */
    float sum = 0.0f;
    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        sum += (float)raw_adc[i];
    }
    float dc_offset = sum / (float)DSP_FFT_SIZE;

    /* 2. Remove DC Bias, Convert to Volts, and Apply Hanning Window */
    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        float sample_v = ((float)raw_adc[i] - dc_offset) * (DSP_VREF_VOLTS / DSP_ADC_FULL_SCALE);
        engine->input_signal[i] = sample_v * engine->window_lut[i];
    }
}

void fft_engine_compute_rfft(fft_engine_t *engine)
{
    if (engine == NULL) {
        return;
    }

    /* Complex arrays for FFT calculation */
    float real[DSP_FFT_SIZE];
    float imag[DSP_FFT_SIZE];

    /* Bit-Reversal Permutation */
    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        uint16_t rev = s_bit_rev[i];
        real[i] = engine->input_signal[rev];
        imag[i] = 0.0f;
    }

    /* Cooley-Tukey Radix-2 Decimation-In-Time Butterfly Stages */
    for (uint16_t len = 2; len <= DSP_FFT_SIZE; len <<= 1) {
        float angle = -2.0f * (float)M_PI / (float)len;
        float wlen_re = cosf(angle);
        float wlen_im = sinf(angle);

        for (uint16_t i = 0; i < DSP_FFT_SIZE; i += len) {
            float w_re = 1.0f;
            float w_im = 0.0f;
            uint16_t half = len >> 1;

            for (uint16_t j = 0; j < half; j++) {
                uint16_t u_idx = i + j;
                uint16_t v_idx = i + j + half;

                float u_re = real[u_idx];
                float u_im = imag[u_idx];

                /* Complex Multiply: v * w */
                float v_re = real[v_idx] * w_re - imag[v_idx] * w_im;
                float v_im = real[v_idx] * w_im + imag[v_idx] * w_re;

                real[u_idx] = u_re + v_re;
                imag[u_idx] = u_im + v_im;
                real[v_idx] = u_re - v_re;
                imag[v_idx] = u_im - v_im;

                /* Advance Twiddle Factor */
                float next_w_re = w_re * wlen_re - w_im * wlen_im;
                float next_w_im = w_re * wlen_im + w_im * wlen_re;
                w_re = next_w_re;
                w_im = next_w_im;
            }
        }
    }

    /* Store Complex Spectrum Output */
    for (uint16_t i = 0; i < DSP_NUM_SPECTRUM_BINS; i++) {
        engine->fft_complex_out[2U * i] = real[i];
        engine->fft_complex_out[2U * i + 1U] = imag[i];
    }
}

void fft_engine_compute_magnitude(fft_engine_t *engine)
{
    if (engine == NULL) {
        return;
    }

    /* Coherent Window Gain Correction Factor for Hanning Window = 2.0 */
    const float window_gain_correction = 2.0f;
    const float scale_factor = (2.0f / (float)DSP_FFT_SIZE) * window_gain_correction;

    /* DC Bin (k = 0) */
    float re0 = engine->fft_complex_out[0];
    float im0 = engine->fft_complex_out[1];
    engine->magnitude_spectrum[0] = (1.0f / (float)DSP_FFT_SIZE) * sqrtf(re0 * re0 + im0 * im0);

    /* AC Bins (k = 1 to 511) */
    for (uint16_t k = 1; k < DSP_NUM_SPECTRUM_BINS; k++) {
        float re = engine->fft_complex_out[2U * k];
        float im = engine->fft_complex_out[2U * k + 1U];
        engine->magnitude_spectrum[k] = scale_factor * sqrtf(re * re + im * im);
    }
}

void fft_engine_extract_diagnostics(const fft_engine_t *engine, vibration_report_t *report)
{
    if (engine == NULL || report == NULL) {
        return;
    }

    memset(report, 0, sizeof(*report));

    /* 1. Calculate Time-Domain True RMS and Peak-to-Peak Amplitude */
    float sum_sq = 0.0f;
    float max_v = -100.0f;
    float min_v = 100.0f;

    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        float s = engine->input_signal[i];
        sum_sq += s * s;
        if (s > max_v) max_v = s;
        if (s < min_v) min_v = s;
    }

    report->rms_amplitude_v = sqrtf(sum_sq / (float)DSP_FFT_SIZE);
    report->peak_to_peak_v = max_v - min_v;

    float peak_abs = (fabsf(max_v) > fabsf(min_v)) ? fabsf(max_v) : fabsf(min_v);
    report->crest_factor = (report->rms_amplitude_v > 0.001f) ? (peak_abs / report->rms_amplitude_v) : 1.0f;

    /* 2. Harmonic Peak Detection (Local Maxima Search) */
    uint8_t peaks_found = 0;
    const float noise_floor_v = 0.015f; /* 15 mV threshold */

    for (uint16_t k = 2; k < (DSP_NUM_SPECTRUM_BINS - 2U) && peaks_found < DSP_MAX_TRACKED_PEAKS; k++) {
        float mag = engine->magnitude_spectrum[k];
        if (mag > noise_floor_v &&
            mag > engine->magnitude_spectrum[k - 1] &&
            mag > engine->magnitude_spectrum[k + 1]) {

            /* Parabolic Peak Interpolation for Sub-Bin Frequency Accuracy */
            float alpha = engine->magnitude_spectrum[k - 1];
            float beta  = engine->magnitude_spectrum[k];
            float gamma = engine->magnitude_spectrum[k + 1];
            float delta = 0.5f * (alpha - gamma) / (alpha - 2.0f * beta + gamma);

            report->peaks[peaks_found].bin_index = k;
            report->peaks[peaks_found].frequency_hz = ((float)k + delta) * DSP_BIN_RESOLUTION_HZ;
            /* Parabolic peak amplitude correction */
            report->peaks[peaks_found].amplitude_v = beta - 0.25f * (alpha - gamma) * delta;
            peaks_found++;
        }
    }
    report->num_peaks_detected = peaks_found;

    /* 3. Determine Fundamental Frequency (f_0) */
    if (peaks_found > 0) {
        report->fundamental_freq_hz = report->peaks[0].frequency_hz;
        float v1 = report->peaks[0].amplitude_v;

        /* 4. Calculate Total Harmonic Distortion (THD) */
        if (v1 > 0.001f && peaks_found > 1) {
            float harmonic_energy = 0.0f;
            for (uint8_t p = 1; p < peaks_found; p++) {
                float v_h = report->peaks[p].amplitude_v;
                harmonic_energy += v_h * v_h;
            }
            report->thd_percent = (sqrtf(harmonic_energy) / v1) * 100.0f;
        } else {
            report->thd_percent = 0.0f;
        }
    }

    /* 5. Classify Machine Vibration Health per ISO 10816-3 */
    if (report->rms_amplitude_v < 0.20f) {
        report->severity_status = VIB_SEVERITY_GOOD;
    } else if (report->rms_amplitude_v < 0.45f) {
        report->severity_status = VIB_SEVERITY_SATISFACTORY;
    } else if (report->rms_amplitude_v < 0.85f) {
        report->severity_status = VIB_SEVERITY_UNSATISFACTORY;
    } else {
        report->severity_status = VIB_SEVERITY_UNACCEPTABLE;
    }
}
