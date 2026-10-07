/**
 * @file dsp_config.h
 * @brief Configuration Parameters & Mathematical Definitions for Spectral Analysis.
 *
 * Defines sampling rate, FFT window dimensions, frequency bin resolution,
 * and ISO 10816-3 vibration severity thresholds for STM32F446RE (ARM Cortex-M4).
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#ifndef DSP_CONFIG_H
#define DSP_CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Sampling Frequency in Hertz (f_s = 50 kHz) */
#define DSP_SAMPLE_RATE_HZ          50000U

/** @brief Number of ADC Samples per FFT Frame (Power of 2) */
#define DSP_FFT_SIZE                1024U

/** @brief Number of Unique Frequency Bins (N / 2) */
#define DSP_NUM_SPECTRUM_BINS       (DSP_FFT_SIZE / 2U)

/** @brief Frequency Bin Resolution (df = f_s / N = 50000 / 1024 = 48.828125 Hz) */
#define DSP_BIN_RESOLUTION_HZ       48.828125f

/** @brief Nyquist Cutoff Frequency (25 kHz) */
#define DSP_NYQUIST_FREQ_HZ         25000.0f

/** @brief ADC Full-Scale Resolution (12-bit ADC = 4096 counts) */
#define DSP_ADC_FULL_SCALE          4095.0f

/** @brief Analog Reference Voltage (3.3V on NUCLEO-F446RE) */
#define DSP_VREF_VOLTS              3.3f

/** @brief Maximum Tracked Harmonic Peaks */
#define DSP_MAX_TRACKED_PEAKS       5U

/**
 * @brief ISO 10816-3 Machine Vibration Severity Classification (RMS Velocity in mm/s)
 * Class I/II Industrial Machines: Small to Medium Motors (< 300 kW)
 */
typedef enum {
    VIB_SEVERITY_GOOD = 0,           /**< Zone A: Newly commissioned (< 1.4 mm/s RMS) */
    VIB_SEVERITY_SATISFACTORY,      /**< Zone B: Long-term continuous operation (< 2.8 mm/s RMS) */
    VIB_SEVERITY_UNSATISFACTORY,    /**< Zone C: Remedial action required (< 4.5 mm/s RMS) */
    VIB_SEVERITY_UNACCEPTABLE       /**< Zone D: Danger, immediate trip required (>= 4.5 mm/s RMS) */
} vibration_severity_t;

/**
 * @brief Harmonic Peak Information
 */
typedef struct {
    float    frequency_hz;          /**< Exact peak center frequency */
    float    amplitude_v;           /**< Peak magnitude in Volts */
    uint16_t bin_index;             /**< FFT frequency bin index */
} dsp_peak_t;

/**
 * @brief Comprehensive Vibration Diagnostics Report
 */
typedef struct {
    float                rms_amplitude_v;    /**< True RMS signal energy */
    float                peak_to_peak_v;     /**< Peak-to-Peak displacement */
    float                crest_factor;       /**< Peak / RMS ratio (bearing wear indicator) */
    float                thd_percent;        /**< Total Harmonic Distortion (%) */
    float                fundamental_freq_hz;/**< Extracted shaft rotational speed */
    dsp_peak_t           peaks[DSP_MAX_TRACKED_PEAKS]; /**< Top harmonic peaks */
    uint8_t              num_peaks_detected;
    vibration_severity_t severity_status;    /**< ISO 10816 health condition */
} vibration_report_t;

#ifdef __cplusplus
}
#endif

#endif /* DSP_CONFIG_H */
