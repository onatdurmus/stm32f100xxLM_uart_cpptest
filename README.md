# STM32F100RB UART Driver – C++

## Project Overview

This project is a C++ implementation of a low-level UART driver for the STM32F100RB microcontroller.

It is derived from my previous project, in which I developed peripheral drivers in C and tested them on STM32 boards. You can find that repository here:

[STM32F100 Driver Tests](https://github.com/onatdurmus/stm32f100xxLM_driver_tests)

You can test this project by connecting an STM32VLDISCOVERY board to an STM32F407VG board.

The code for the STM32F4 board is available in the previously mentioned repository under `stm32cube_workspace01/UART_02_InterruptTest`.

## Hardware and Development Environment

- **Microcontroller:** STM32F100RB
- **Development Board:** STM32VLDISCOVERY
- **IDE:** Keil µVision
- **Programming Language:** C++

## Build Instructions

1. Clone this repository.
2. Open the `.uvprojx` project file using Keil µVision.
3. Build the project.

### Common Build Errors

**No UTAG found**

Go to Options → Debugger and select ST-Link from the dropdown menu.

**Cortex-M0 not found**

Go to Options → Debugger → Settings. Remove the existing Flash Download configuration and add the appropriate Flash Programming Algorithm again.

### Notes

The legacy Keil software pack was intentionally selected because newer software packs cause compatibility issues with the STM32F100RB device.