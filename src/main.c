/**
 * @file main.c
 * @brief Application Entry Point & Vibration DSP Verification Harness.
 *
 * Demonstrates 1024-point Real FFT spectral decomposition, dual-buffer DMA handoff,
 * harmonic peak tracking, and ISO 10816 machine health classification on STM32 NUCLEO-F446RE.
 *
 * @author Sherifred Singh
 * @copyright MIT License
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "vibration_analyzer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static vibration_analyzer_t g_analyzer;

#if !defined(STM32F446xx) && !defined(TARGET_STM32F4)
/**
 * @brief Synthesize multi-harmonic vibration signal to emulate rotating machinery accelerometer.
 *
 * Signal components:
 * - DC Bias: 1.65 V (ADC mid-rail: count 2048)
 * - Fundamental ($f_0$): 1465.0 Hz @ 0.60 V (Motor shaft rotational peak)
 * - 2nd Harmonic ($2f_0$): 2930.0 Hz @ 0.12 V (Mechanical misalignment)
 * - 3rd Harmonic ($3f_0$): 4395.0 Hz @ 0.18 V (Bearing outer race fault)
 *
 * @param buffer Output array for 1024 12-bit ADC raw integer codes.
 */
static void generate_synthetic_vibration(uint16_t *buffer)
{
    const float f0 = 1465.0f;
    const float f2 = 2930.0f;
    const float f3 = 4395.0f;
    const float fs = (float)DSP_SAMPLE_RATE_HZ;

    for (uint16_t i = 0; i < DSP_FFT_SIZE; i++) {
        float t = (float)i / fs;

        /* Compose physical analog voltage in range [0.0V .. 3.3V] */
        float v_analog = 1.65f +
                         0.60f * sinf(2.0f * (float)M_PI * f0 * t) +
                         0.12f * sinf(2.0f * (float)M_PI * f2 * t + 0.5f) +
                         0.18f * sinf(2.0f * (float)M_PI * f3 * t + 1.2f);

        /* Clamp to ADC input rails [0.0V .. 3.3V] */
        if (v_analog < 0.0f) v_analog = 0.0f;
        if (v_analog > 3.3f) v_analog = 3.3f;

        /* Quantize to 12-bit ADC integer counts [0 .. 4095] */
        uint16_t adc_code = (uint16_t)((v_analog / DSP_VREF_VOLTS) * DSP_ADC_FULL_SCALE);
        buffer[i] = adc_code;
    }
}

static void run_desktop_demonstration(void)
{
    printf("\n============================================================\n");
    printf("   STM32 NUCLEO-F446RE CMSIS-DSP VIBRATION SPECTRAL ANALYZER\n");
    printf("   1024-Point Real FFT | Ping-Pong DMA | ISO 10816 Diagnostics\n");
    printf("============================================================\n");

    /* Initialize analyzer subsystems */
    vibration_analyzer_init(&g_analyzer);

    /* Generate synthetic accelerometer stream */
    uint16_t raw_adc[DSP_FFT_SIZE];
    generate_synthetic_vibration(raw_adc);

    printf(">> Ingesting 1024-sample raw 12-bit ADC frame (50 kHz sampling rate)...\n");
    vibration_analyzer_feed_samples(&g_analyzer, raw_adc);

    printf(">> Executing Hanning windowing and 1024-point Real FFT on Cortex-M4 FPU...\n");
    bool processed = vibration_analyzer_process(&g_analyzer);

    if (processed) {
        vibration_analyzer_print_report(&g_analyzer);
    } else {
        printf("ERROR: Failed to process DMA acquisition frame!\n");
    }
}
#endif

int main(void)
{
#if defined(STM32F446xx) || defined(TARGET_STM32F4)
    vibration_analyzer_init(&g_analyzer);

    while (1) {
        if (vibration_analyzer_process(&g_analyzer)) {
            vibration_analyzer_print_report(&g_analyzer);
        }
    }
#else
    run_desktop_demonstration();
    return 0;
#endif
}
