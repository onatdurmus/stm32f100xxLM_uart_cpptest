#ifndef STM32F100XXLM_RCC_HPP
#define STM32F100XXLM_RCC_HPP

#include "stm32f100xxLM.hpp"

namespace stm32f100::rcc {

std::uint32_t getPclk1();  // APB1 clock in Hz (USART2/3)
std::uint32_t getPclk2();  // APB2 clock in Hz (USART1, GPIO)

void enableGpioClock(std::uintptr_t gpioBase);
void enableUsartClock(std::uintptr_t usartBase);

}  // namespace stm32f100::rcc

#endif  // STM32F100XXLM_RCC_HPP
