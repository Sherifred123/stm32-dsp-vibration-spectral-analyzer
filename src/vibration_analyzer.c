/**
 * @file vibration_analyzer.c
 * @brief High-Level Condition Monitoring & Vibration Spectrum Implementation.
 *
 * Coordinates DMA buffer handoffs, FFT mathematical acceleration, and ISO 10816 evaluation.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "vibration_analyzer.h"
#include <stdio.h>
#include <string.h>

static const char *get_severity_name(vibration_severity_t s)
{
    switch (s) {
    case VIB_SEVERITY_GOOD:          return "ZONE A: GOOD (Optimal Condition)";
    case VIB_SEVERITY_SATISFACTORY:  return "ZONE B: SATISFACTORY (Acceptable for Long-Term Operation)";
    case VIB_SEVERITY_UNSATISFACTORY:return "ZONE C: UNSATISFACTORY (Maintenance Required)";
    case VIB_SEVERITY_UNACCEPTABLE:  return "ZONE D: UNACCEPTABLE (Critical Vibration Alarm)";
    default:                         return "UNKNOWN";
    }
}

void vibration_analyzer_init(vibration_analyzer_t *analyzer)
{
    if (analyzer == NULL) {
        return;
    }

    memset(analyzer, 0, sizeof(*analyzer));
    dma_pingpong_init(&analyzer->dma);
    fft_engine_init(&analyzer->fft);
}

void vibration_analyzer_feed_samples(vibration_analyzer_t *analyzer, const uint16_t *raw_adc)
{
    if (analyzer == NULL || raw_adc == NULL) {
        return;
    }

    /* Simulate DMA Transfer to current active target */
    if (analyzer->dma.active_target == DMA_BUFFER_0) {
        memcpy(analyzer->dma.buffer_0, raw_adc, DSP_FFT_SIZE * sizeof(uint16_t));
    } else {
        memcpy(analyzer->dma.buffer_1, raw_adc, DSP_FFT_SIZE * sizeof(uint16_t));
    }

    /* Trigger Transfer-Complete ISR */
    dma_pingpong_transfer_complete_isr(&analyzer->dma);
}

bool vibration_analyzer_process(vibration_analyzer_t *analyzer)
{
    if (analyzer == NULL) {
        return false;
    }

    const uint16_t *frame_ptr = NULL;
    if (!dma_pingpong_get_ready_frame(&analyzer->dma, &frame_ptr)) {
        return false; /* Buffer not yet ready */
    }

    /* 1. Apply DC Removal & Hanning Window */
    fft_engine_apply_window(&analyzer->fft, frame_ptr);

    /* 2. Execute 1024-point Real FFT */
    fft_engine_compute_rfft(&analyzer->fft);

    /* 3. Compute Normalized Magnitude Spectrum */
    fft_engine_compute_magnitude(&analyzer->fft);

    /* 4. Extract Harmonic Peaks & Diagnostics */
    fft_engine_extract_diagnostics(&analyzer->fft, &analyzer->latest_report);

    /* 5. Release DMA Ping-Pong Buffer */
    dma_pingpong_release_frame(&analyzer->dma);
    analyzer->total_processed_frames++;

    return true;
}

void vibration_analyzer_print_report(const vibration_analyzer_t *analyzer)
{
    if (analyzer == NULL) {
        return;
    }

    const vibration_report_t *r = &analyzer->latest_report;

    printf("\n============================================================\n");
    printf("   ROTATING MACHINERY VIBRATION SPECTRAL ANALYSIS REPORT\n");
    printf("   Standard ISO 10816-3 Condition Monitoring & Edge DSP\n");
    printf("============================================================\n");
    printf("  [Time-Domain Metrics]\n");
    printf("    Signal RMS Amplitude:      %.3f Vrms\n", r->rms_amplitude_v);
    printf("    Peak-to-Peak Amplitude:    %.3f Vpp\n", r->peak_to_peak_v);
    printf("    Crest Factor (Peak/RMS):   %.2f (Normal < 3.5)\n", r->crest_factor);
    printf("\n  [Frequency-Domain Spectral Metrics]\n");
    printf("    Fundamental Shaft Speed:   %.1f Hz (%.0f RPM)\n",
           r->fundamental_freq_hz, r->fundamental_freq_hz * 60.0f);
    printf("    Total Harmonic Distortion: %.2f %%\n", r->thd_percent);
    printf("    Dominant Harmonic Peaks:   %u detected\n", r->num_peaks_detected);

    for (uint8_t i = 0; i < r->num_peaks_detected; i++) {
        printf("      #%u:  %7.1f Hz  |  %5.3f V  |  Bin %3u\n",
               i + 1, r->peaks[i].frequency_hz, r->peaks[i].amplitude_v, r->peaks[i].bin_index);
    }

    printf("\n  [ISO 10816-3 Machine Health Evaluation]\n");
    printf("    Health Status: %s\n", get_severity_name(r->severity_status));
    printf("    Frames Acquired: %u  |  Dropped: %u\n",
           analyzer->dma.frames_acquired, analyzer->dma.frames_dropped);
    printf("============================================================\n\n");
}
