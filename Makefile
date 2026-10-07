# =========================================================================
# Makefile for STM32 DSP Vibration Spectral Analyzer
# Target MCU: STM32F446RET6 (ARM Cortex-M4 @ 180 MHz with FPU)
# Host Target: Linux / macOS / Windows MinGW GCC (Desktop Simulation)
# =========================================================================

CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -Iinclude
LDFLAGS ?= -lm
TEST_CFLAGS ?= $(CFLAGS) -Itests/unity

# ARM Toolchain for Target Hardware Compilation
ARM_CC ?= arm-none-eabi-gcc
ARM_CFLAGS ?= -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
              -DSTM32F446xx -DTARGET_STM32F4 -Iinclude -O2 -Wall -Wextra

SRC = src/dma_pingpong.c src/fft_engine.c src/vibration_analyzer.c
MAIN_SRC = $(SRC) src/main.c
TEST_SRC = $(SRC) tests/unity/unity.c tests/test_dsp_suite.c

BIN_DEMO = build_demo
BIN_TEST = run_tests

.PHONY: all host test arm clean

all: host test

host:
	@echo "==> Building Host Vibration DSP Demonstration..."
	$(CC) $(CFLAGS) $(MAIN_SRC) $(LDFLAGS) -o $(BIN_DEMO)
	@echo "==> Build successful: $(BIN_DEMO)"

test:
	@echo "==> Building Unity Unit Test Suite..."
	$(CC) $(TEST_CFLAGS) $(TEST_SRC) $(LDFLAGS) -o $(BIN_TEST)
	@echo "==> Executing Unit Tests..."
	./$(BIN_TEST)

arm:
	@echo "==> Cross-compiling for STM32 Cortex-M4 (arm-none-eabi-gcc)..."
	$(ARM_CC) $(ARM_CFLAGS) -c $(MAIN_SRC)
	@echo "==> Cortex-M4 DSP objects compiled successfully."

clean:
	@echo "==> Cleaning build artifacts..."
	rm -f $(BIN_DEMO) $(BIN_TEST) $(BIN_DEMO).exe $(BIN_TEST).exe *.o
