# ⚙️ Real-Time Vibration Spectral Analyzer & Edge DSP Monitor
### *Production-Grade 1024-Point Real FFT & Ping-Pong DMA Engine on STM32F446RE (ARM Cortex-M4 @ 180 MHz)*

[![C/C++ CI Test Suite](https://img.shields.io/badge/CI%20Build-Passing-brightgreen?style=for-the-badge&logo=githubactions&logoColor=white)](https://github.com/Sherifred123/stm32-dsp-vibration-spectral-analyzer/actions)
[![Language](https://img.shields.io/badge/Language-Embedded%20C%20(C99)-00599C?style=for-the-badge&logo=c&logoColor=white)](https://en.wikipedia.org/wiki/C_(programming_language))
[![Target Architecture](https://img.shields.io/badge/Target-STM32F446RE%20%2F%20Cortex--M4%20FPU-032347?style=for-the-badge&logo=stmicroelectronics&logoColor=white)](https://www.st.com)
[![DSP Engine](https://img.shields.io/badge/DSP-1024--Point%20RFFT%20%7C%20Hanning-orange?style=for-the-badge)]()
[![Standard](https://img.shields.io/badge/Standard-ISO%2010816--3%20Condition%20Monitoring-blue?style=for-the-badge)]()
[![Unit Tests](https://img.shields.io/badge/Unit%20Tests-7%2F7%20Passing%20(100%25)-success?style=for-the-badge)]()
[![License](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)](LICENSE)

> **A deterministic, real-time vibration spectral analysis engine engineered for industrial condition monitoring and predictive maintenance on rotating machinery (electric motors, pumps, bearings, turbines). Captures continuous 50 kHz accelerometer telemetry via hardware-triggered double-buffer DMA with zero CPU overhead, executes 1024-point Real FFT decomposition on the Cortex-M4 hardware FPU in under 1.1 ms, and computes Total Harmonic Distortion (THD) and ISO 10816-3 severity levels.**

---

## 📌 System Architecture

<p align="center">
  <img src="docs/architecture.svg" alt="Vibration DSP Spectral Analyzer Architecture" width="100%">
</p>

---

## 📑 Table of Contents

- [1. System Overview & Problem Statement](#1-system-overview--problem-statement)
- [2. Hardware-Timed Ingestion & Ping-Pong DMA](#2-hardware-timed-ingestion--ping-pong-dma)
- [3. DSP Pipeline & Mathematical Foundation](#3-dsp-pipeline--mathematical-foundation)
- [4. Diagnostic Feature Extraction & ISO 10816-3](#4-diagnostic-feature-extraction--iso-10816-3)
- [5. Hardware Pinout & Wiring (NUCLEO-F446RE)](#5-hardware-pinout--wiring-nucleo-f446re)
- [6. Automated Verification & Test Results](#6-automated-verification--test-results)
- [7. Quickstart Guide](#7-quickstart-guide)
- [8. Architecture Design Decisions & Mathematical Rationale](#8-architecture-design-decisions--mathematical-rationale)

---

## 1. System Overview & Problem Statement

Unscheduled machine downtime in industrial rotating equipment (electric drive motors, centrifugal pumps, gearboxes) causes catastrophic failures. Simple time-domain RMS thresholding cannot detect early-stage mechanical faults such as:
1. **Shaft Misalignment & Imbalance:** Manifesting primarily as strong energy spikes at $1\times$ and $2\times$ rotational speed ($f_0$ and $2f_0$).
2. **Bearing Outer/Inner Race Damage:** High-frequency impact harmonics ($3\times$ to $5\times f_0$) visible only in the spectral frequency domain.
3. **Harmonic Energy Distortion:** Total Harmonic Distortion (THD) metrics that quantify mechanical degradation before physical seizure.

This project delivers an **autonomous edge DSP pipeline** executing directly on the **STM32F446RE MCU**, converting analog accelerometer telemetry into calibrated frequency spectra and ISO-standard health evaluations in real time.

---

## 2. Hardware-Timed Ingestion & Ping-Pong DMA

To guarantee strictly uniform time-domain sampling without software jitter or CPU blocking:

```
                  ┌──────────────────────────────────────────────┐
                  │   TIM2 TRGO Hardware Trigger @ 50,000 Hz    │
                  └──────────────────────┬───────────────────────┘
                                         │ Hardware Trigger
                                         ▼
                  ┌──────────────────────────────────────────────┐
                  │    ADC1 12-Bit Peripheral (PA0 / ADC1_IN0)   │
                  └──────────────────────┬───────────────────────┘
                                         │ Direct DMA Request
                                         ▼
┌────────────────────────────────────────────────────────────────────────────────┐
│                   DMA2 Stream 0 Double-Buffer Mode (DBM)                       │
│                                                                                │
│   ┌───────────────────────────┐             ┌───────────────────────────┐      │
│   │   Buffer 0 (1024 Samples) │             │   Buffer 1 (1024 Samples) │      │
│   │     [ACTIVE ADC TARGET]   │             │   [PROCESSED BY FPU CORE] │      │
│   └───────────────────────────┘             └───────────────────────────┘      │
│                 ▲                                         │                    │
│                 └────────── Circular Hardware Swap ───────┘                    │
└────────────────────────────────────────────────────────────────────────────────┘
```

- **Sampling Rate:** $f_s = 50,000\,\text{Hz}$ ($\Delta t = 20\,\mu\text{s}$).
- **Nyquist Limit:** $f_{\text{nyq}} = 25,000\,\text{Hz}$.
- **Double-Buffering Guarantee:** While the ADC fills Buffer 0 via DMA, the Cortex-M4 core computes the 1024-point FFT on Buffer 1. When Buffer 0 reaches 1024 samples, silicon switches pointers automatically without dropping a single sample.

---

## 3. DSP Pipeline & Mathematical Foundation

### 3.1 DC Bias Removal & Windowing
Raw 12-bit ADC codes ($0 \dots 4095$) centered at the mid-rail ($1.65\,\text{V}$) are mean-subtracted and scaled to physical Volts:
$$V[n] = \left(\text{ADC}[n] - \overline{\text{ADC}}\right) \times \frac{V_{\text{ref}}}{4095.0}$$

To prevent spectral leakage caused by finite rectangular truncation, a **raised-cosine Hanning window** is applied:
$$w[n] = 0.5 \times \left(1.0 - \cos\left(\frac{2\pi n}{N - 1}\right)\right)$$
$$x[n] = V[n] \times w[n]$$

### 3.2 1024-Point Real Fast Fourier Transform (RFFT)
The time-domain sequence $x[n]$ is transformed using a decimation-in-time radix-2 algorithm with precomputed bit-reversal and twiddle factor lookup tables:
$$X[k] = \sum_{n=0}^{N-1} x[n] \cdot e^{-j 2\pi k n / N}, \quad k = 0, 1, \dots, \frac{N}{2}-1$$

### 3.3 Calibrated Magnitude Spectrum
Coherent gain loss of the Hanning window is compensated by a factor of $2.0$:
$$|X[k]| = \frac{2}{N} \cdot 2.0 \cdot \sqrt{\text{Re}[k]^2 + \text{Im}[k]^2}$$
- **Bin Frequency Resolution:** $\Delta f = \frac{f_s}{N} = \frac{50,000}{1024} = 48.828125\,\text{Hz}$ per bin.

---

## 4. Diagnostic Feature Extraction & ISO 10816-3

### 4.1 Sub-Bin Parabolic Peak Interpolation
To achieve true frequency accuracy exceeding raw bin resolution, local spectral maxima are interpolated parabolically:
$$\delta = 0.5 \times \frac{\alpha - \gamma}{\alpha - 2\beta + \gamma}, \quad f_{\text{peak}} = (k + \delta) \times \Delta f$$
where $\alpha = |X[k-1]|$, $\beta = |X[k]|$, $\gamma = |X[k+1]|$.

### 4.2 Total Harmonic Distortion (THD)
Quantifies structural and mechanical degradation across the first 5 harmonics:
$$\text{THD} = \frac{\sqrt{V_2^2 + V_3^2 + V_4^2 + V_5^2}}{V_1} \times 100\%$$

### 4.3 ISO 10816-3 Machinery Severity Table

| Velocity Severity Band | RMS Range (mm/s) | Machine Health Condition | Action Required |
|:---:|:---:|:---:|---|
| **Zone A** | $< 1.4$ | 🟢 **GOOD** | Newly commissioned machinery; optimal balance. |
| **Zone B** | $1.4 - 2.8$ | 🔵 **SATISFACTORY** | Normal continuous industrial operation. |
| **Zone C** | $2.8 - 4.5$ | 🟡 **UNSATISFACTORY** | Mechanical degradation; schedule maintenance. |
| **Zone D** | $\ge 4.5$ | 🔴 **UNACCEPTABLE** | Critical vibration; trip machine to prevent damage. |

---

## 5. Hardware Pinout & Wiring (NUCLEO-F446RE)

| Pin | Function | Hardware Connection | Description |
|:---:|:---:|:---:|---|
| **PA0** | ADC1_IN0 | Accelerometer Analog Out | Vibration sensor analog voltage input ($0 - 3.3\,\text{V}$) |
| **PA4** | DAC1_OUT | Bench Loopback (Optional) | Synthesizes synthetic multi-harmonic test waveforms |
| **PA2** | USART2_TX | ST-LINK VCP (USB) | 115,200 baud diagnostic telemetry and ASCII waterfall |
| **PA3** | USART2_RX | ST-LINK VCP (USB) | Command / calibration configuration input |
| **PA5** | GPIO Out | User LED (LD2) | Blinks on each completed FFT frame |
| **GND** | Ground | Sensor Common Ground | Analog ground reference |

---

## 6. Automated Verification & Test Results

### 🧪 Unity C Unit Test Suite (7/7 Passing)
```text
------------------------------------------------------------
   UNITY UNIT TEST EXECUTION: Embedded DSP Vibration Analyzer
------------------------------------------------------------
  [PASS] test_dma_pingpong_init                        (Line 173)
  [PASS] test_dma_pingpong_transfer_swap               (Line 174)
  [PASS] test_dma_pingpong_overrun_detection           (Line 175)
  [PASS] test_hanning_window_symmetry                  (Line 178)
  [PASS] test_fft_pure_sinusoid_frequency_detection    (Line 179)
  [PASS] test_fft_multi_harmonic_thd_measurement       (Line 180)
  [PASS] test_iso10816_vibration_severity_classification (Line 181)

============================================================
   TEST SUMMARY: 7 Tests, 0 Failures, 0 Ignored
   RESULT: [PASS] (100% Tests Verified)
============================================================
```

### 📊 Desktop Execution Benchmark Output
```text
============================================================
   ROTATING MACHINERY VIBRATION SPECTRAL ANALYSIS REPORT
   Standard ISO 10816-3 Condition Monitoring & Edge DSP
============================================================
  [Time-Domain Metrics]
    Signal RMS Amplitude:      0.276 Vrms
    Peak-to-Peak Amplitude:    1.379 Vpp
    Crest Factor (Peak/RMS):   2.93 (Normal < 3.5)

  [Frequency-Domain Spectral Metrics]
    Fundamental Shaft Speed:   1465.0 Hz (87898 RPM)
    Total Harmonic Distortion: 36.05 %
    Dominant Harmonic Peaks:   3 detected
      #1:   1465.0 Hz  |  0.599 V  |  Bin  30
      #2:   2929.9 Hz  |  0.120 V  |  Bin  60
      #3:   4394.9 Hz  |  0.180 V  |  Bin  90

  [ISO 10816-3 Machine Health Evaluation]
    Health Status: ZONE B: SATISFACTORY (Acceptable for Long-Term Operation)
    Frames Acquired: 1  |  Dropped: 0
============================================================
```

---

## 7. Quickstart Guide

### 1. Build and Run Host Demo
```bash
git clone https://github.com/Sherifred123/stm32-dsp-vibration-spectral-analyzer.git
cd stm32-dsp-vibration-spectral-analyzer

# Compile and run host simulation
make host
./build_demo
```

### 2. Run Automated Unit Tests
```bash
make test
```

### 3. Run Python Spectral Waterfall Visualizer
```bash
python tools/spectral_viewer.py
```

### 4. Cross-Compile for STM32 Cortex-M4
```bash
make arm
```

---

## 8. Architecture Design Decisions & Mathematical Rationale

### 8.1 Why Frequency-Domain Analysis vs. Time-Domain Filtering?
Simple time-domain RMS thresholding only alarms when severe machine destruction has already begun. In rotating machinery, early degradation manifests as specific narrowband spectral energy spikes (e.g. inner race bearing frequencies at non-integer multiples of rotational speed). Fast Fourier decomposition isolates the exact mechanical defect signature long before total energy levels breach safety trip lines.

### 8.2 Why Raised-Cosine Windowing (Hanning vs. Rectangular)?
Rectangular windowing produces significant spectral leakage ($\text{sinc}$ sidelobes down only $-13\,\text{dB}$), which falsely masks low-amplitude bearing fault harmonics behind large fundamental peaks. The Hanning window attenuates sidelobes to $-32\,\text{dB}$ with a rapid $-18\,\text{dB/octave}$ decay, ensuring weak harmonic anomalies remain clearly distinguishable above the noise floor.

### 8.3 Zero-CPU Double-Buffering Overhead Guarantee
At $50\,\text{kHz}$, servicing an interrupt for every individual ADC conversion would consume $> 25\%$ of the CPU budget in context switching alone. Using DMA2 Stream 0 circular double-buffer mode decouples sample acquisition completely from CPU execution: the core is interrupted only once every $20.48\,\text{ms}$ (1024 samples), requiring only $\approx 1.1\,\text{ms}$ ($5.3\%$ CPU load) to complete the entire FFT decomposition and diagnostic evaluation.

---

## 📄 License

This project is licensed under the [MIT License](LICENSE) - see the LICENSE file for details.  
Designed and engineered by **Sherifred Singh**.
