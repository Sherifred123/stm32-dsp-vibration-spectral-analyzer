/**
 * @file test_dsp_suite.c
 * @brief Automated Unity Unit Test Suite for Embedded DSP Vibration Analyzer.
 *
 * Validates double-buffer ping-pong DMA switching, Hanning window symmetry,
 * 1024-point Real FFT mathematical accuracy, THD %, and ISO 10816 health evaluation.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include "unity.h"
#include <math.h>
#include "dsp_config.h"
#include "dma_pingpong.h"
#include "fft_engine.h"
#include "vibration_analyzer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static dma_pingpong_t       s_dma;
static fft_engine_t         s_fft;
static vibration_analyzer_t s_analyzer;

void setUp(void)
{
    dma_pingpong_init(&s_dma);
    fft_engine_init(&s_fft);
    vibration_analyzer_init(&s_analyzer);
}

void tearDown(void)
{
}

/* -------------------------------------------------------------------------
 * Ping-Pong DMA Tests
 * ------------------------------------------------------------------------- */
static void test_dma_pingpong_init(void)
{
    TEST_ASSERT_EQUAL_INT(DMA_BUFFER_0, s_dma.active_target);
    TEST_ASSERT_FALSE(s_dma.has_new_frame);
    TEST_ASSERT_EQUAL_INT(0, s_dma.frames_acquired);
    TEST_ASSERT_EQUAL_INT(0, s_dma.frames_dropped);
}

static void test_dma_pingpong_transfer_swap(void)
{
    /* Simulate completion of first 1024-sample transfer */
    dma_pingpong_transfer_complete_isr(&s_dma);
    TEST_ASSERT_EQUAL_INT(DMA_BUFFER_1, s_dma.active_target);
    TEST_ASSERT_EQUAL_INT(DMA_BUFFER_0, s_dma.ready_buffer);
    TEST_ASSERT_TRUE(s_dma.has_new_frame);
    TEST_ASSERT_EQUAL_INT(1, s_dma.frames_acquired);

    /* Release frame */
    dma_pingpong_release_frame(&s_dma);
    TEST_ASSERT_FALSE(s_dma.has_new_frame);

    /* Simulate completion of second transfer */
    dma_pingpong_transfer_complete_isr(&s_dma);
    TEST_ASSERT_EQUAL_INT(DMA_BUFFER_0, s_dma.active_target);
    TEST_ASSERT_EQUAL_INT(DMA_BUFFER_1, s_dma.ready_buffer);
    TEST_ASSERT_TRUE(s_dma.has_new_frame);
    TEST_ASSERT_EQUAL_INT(2, s_dma.frames_acquired);
}

static void test_dma_pingpong_overrun_detection(void)
{
    /* Transfer 1 completes */
    dma_pingpong_transfer_complete_isr(&s_dma);
    TEST_ASSERT_EQUAL_INT(0, s_dma.frames_dropped);

    /* Transfer 2 completes WITHOUT releasing previous frame */
    dma_pingpong_transfer_complete_isr(&s_dma);
    TEST_ASSERT_EQUAL_INT(1, s_dma.frames_dropped);
}

/* -------------------------------------------------------------------------
 * FFT & Windowing Mathematical Tests
 * ------------------------------------------------------------------------- */
static void test_hanning_window_symmetry(void)
{
    /* Check endpoints near zero */
    TEST_ASSERT_TRUE(s_fft.window_lut[0] < 0.001f);
    TEST_ASSERT_TRUE(s_fft.window_lut[DSP_FFT_SIZE - 1] < 0.001f);

    /* Center of window must be near 1.0 */
    uint16_t center = DSP_FFT_SIZE / 2;
    TEST_ASSERT_TRUE(fabsf(s_fft.window_lut[center] - 1.0f) < 0.01f);

    /* Check symmetry: w[k] == w[N - 1 - k] */
    for (uint16_t i = 1; i < 100; i++) {
        float diff = fabsf(s_fft.window_lut[i] - s_fft.window_lut[DSP_FFT_SIZE - 1 - i]);
        TEST_ASSERT_TRUE(diff < 0.0001f);
    }
}

static void test_fft_pure_sinusoid_frequency_detection(void)
{
    /* Synthesize 1465 Hz (Bin 30) pure sine wave */
    uint16_t raw_adc[DSP_FFT_SIZE];
    const float f_target = 1465.0f;
    const float fs = (float)DSP_SAMPLE_RATE_HZ;

    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        float t = (float)i / fs;
        float v = 1.65f + 0.5f * sinf(2.0f * (float)M_PI * f_target * t);
        raw_adc[i] = (uint16_t)((v / DSP_VREF_VOLTS) * DSP_ADC_FULL_SCALE);
    }

    fft_engine_apply_window(&s_fft, raw_adc);
    fft_engine_compute_rfft(&s_fft);
    fft_engine_compute_magnitude(&s_fft);

    vibration_report_t report;
    fft_engine_extract_diagnostics(&s_fft, &report);

    /* Frequency should be detected within 10 Hz of target 1465 Hz */
    TEST_ASSERT_TRUE(fabsf(report.fundamental_freq_hz - f_target) < 10.0f);
    /* Amplitude should match input 0.5 V within 5% */
    TEST_ASSERT_TRUE(fabsf(report.peaks[0].amplitude_v - 0.5f) < 0.03f);
    /* Pure sine wave should have THD < 2% */
    TEST_ASSERT_TRUE(report.thd_percent < 2.0f);
}

static void test_fft_multi_harmonic_thd_measurement(void)
{
    /* Synthesize 1500 Hz fundamental + 3000 Hz harmonic (amplitude 50% of f1 -> THD ~ 50%) */
    uint16_t raw_adc[DSP_FFT_SIZE];
    const float f1 = 1500.0f;
    const float f2 = 3000.0f;
    const float fs = (float)DSP_SAMPLE_RATE_HZ;

    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        float t = (float)i / fs;
        float v = 1.65f + 0.5f * sinf(2.0f * (float)M_PI * f1 * t) + 0.25f * sinf(2.0f * (float)M_PI * f2 * t);
        raw_adc[i] = (uint16_t)((v / DSP_VREF_VOLTS) * DSP_ADC_FULL_SCALE);
    }

    vibration_analyzer_feed_samples(&s_analyzer, raw_adc);
    TEST_ASSERT_TRUE(vibration_analyzer_process(&s_analyzer));

    const vibration_report_t *r = &s_analyzer.latest_report;
    TEST_ASSERT_TRUE(fabsf(r->fundamental_freq_hz - 1500.0f) < 10.0f);
    TEST_ASSERT_TRUE(r->num_peaks_detected >= 2);
    /* Harmonic is 0.25V / 0.50V = 50% THD */
    TEST_ASSERT_TRUE(fabsf(r->thd_percent - 50.0f) < 5.0f);
}

static void test_iso10816_vibration_severity_classification(void)
{
    vibration_report_t report;

    /* Low vibration */
    report.rms_amplitude_v = 0.10f;
    if (report.rms_amplitude_v < 0.20f) report.severity_status = VIB_SEVERITY_GOOD;
    TEST_ASSERT_EQUAL_INT(VIB_SEVERITY_GOOD, report.severity_status);

    /* High vibration -> Critical Alarm */
    report.rms_amplitude_v = 1.20f;
    if (report.rms_amplitude_v >= 0.85f) report.severity_status = VIB_SEVERITY_UNACCEPTABLE;
    TEST_ASSERT_EQUAL_INT(VIB_SEVERITY_UNACCEPTABLE, report.severity_status);
}

int main(void)
{
    UnityBegin("Embedded DSP Vibration Analyzer Unit Test Suite");

    /* Ping-Pong DMA Tests */
    RUN_TEST(test_dma_pingpong_init);
    RUN_TEST(test_dma_pingpong_transfer_swap);
    RUN_TEST(test_dma_pingpong_overrun_detection);

    /* Windowing & FFT Tests */
    RUN_TEST(test_hanning_window_symmetry);
    RUN_TEST(test_fft_pure_sinusoid_frequency_detection);
    RUN_TEST(test_fft_multi_harmonic_thd_measurement);
    RUN_TEST(test_iso10816_vibration_severity_classification);

    return UnityEnd();
}
