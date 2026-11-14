#include "stm32f100xxLM_rcc.hpp"

namespace stm32f100::rcc {

namespace {

constexpr std::uint32_t HSI_HZ = 8'000'000U;  // Internal RC oscillator
constexpr std::uint32_t HSE_HZ = 8'000'000U;  // On-board crystal

// RCC_CFGR bit positions
constexpr unsigned CFGR_SWS    = 2;   // [3:2]  system clock switch status
constexpr unsigned CFGR_HPRE   = 4;   // [7:4]  AHB prescaler
constexpr unsigned CFGR_PPRE1  = 8;   // [10:8] APB1 prescaler
constexpr unsigned CFGR_PPRE2  = 11;  // [13:11] APB2 prescaler
constexpr unsigned CFGR_PLLSRC = 16;
constexpr unsigned CFGR_PLLMUL = 18;  // [21:18]

// RCC_APBxENR bit positions
constexpr unsigned APB2ENR_IOPAEN   = 2;   // GPIOA..GPIOE are bits 2..6
constexpr unsigned APB2ENR_USART1EN = 14;
constexpr unsigned APB1ENR_USART2EN = 17;
constexpr unsigned APB1ENR_USART3EN = 18;

std::uint32_t sysclk()
{
    const std::uint32_t cfgr = RCC().CFGR;

    switch ((cfgr >> CFGR_SWS) & 0x3U)
    {
    case 0:  // HSI
        return HSI_HZ;
    case 1:  // HSE
        return HSE_HZ;
    case 2:  // PLL
    {
        // 0000: x2, 0001: x3 ... 1110 and 1111: x16
        std::uint32_t multiplier = ((cfgr >> CFGR_PLLMUL) & 0xFU) + 2U;
        if (multiplier > 16U)
        {
            multiplier = 16U;
        }

        const std::uint32_t pllInput = (cfgr & bit(CFGR_PLLSRC))
                                           ? HSE_HZ / ((RCC().CFGR2 & 0xFU) + 1U)  // HSE / PREDIV1
                                           : HSI_HZ / 2U;                          // HSI / 2
        return pllInput * multiplier;
    }
    default:  // Reserved value
        return 0U;
    }
}

// HPRE: 0xxx: /1, 1000: /2, 1001: /4, 1010: /8, 1011: /16, 1100: /64 ... 1111: /512 (/32 does not exist)
std::uint32_t ahbDivider(std::uint32_t cfgr)
{
    const std::uint32_t hpre = (cfgr >> CFGR_HPRE) & 0xFU;
    if (hpre < 8U)
    {
        return 1U;
    }
    const std::uint32_t shift = (hpre - 7U) + ((hpre >= 12U) ? 1U : 0U);
    return 1U << shift;
}

// PPREx: 0xx: /1, 100: /2, 101: /4, 110: /8, 111: /16
std::uint32_t apbDivider(std::uint32_t ppre)
{
    return (ppre < 4U) ? 1U : (1U << (ppre - 3U));
}

}  // namespace

std::uint32_t getPclk1()
{
    const std::uint32_t cfgr = RCC().CFGR;
    return sysclk() / (ahbDivider(cfgr) * apbDivider((cfgr >> CFGR_PPRE1) & 0x7U));
}

std::uint32_t getPclk2()
{
    const std::uint32_t cfgr = RCC().CFGR;
    return sysclk() / (ahbDivider(cfgr) * apbDivider((cfgr >> CFGR_PPRE2) & 0x7U));
}

void enableGpioClock(std::uintptr_t gpioBase)
{
    // GPIO ports are placed 0x400 apart, and their enable bits are consecutive.
    const std::uintptr_t index = (gpioBase - base::GPIOA) / 0x400U;
    if (index <= 4U)
    {
        RCC().APB2ENR |= bit(APB2ENR_IOPAEN + static_cast<unsigned>(index));
    }
}

void enableUsartClock(std::uintptr_t usartBase)
{
    switch (usartBase)
    {
    case base::USART1: RCC().APB2ENR |= bit(APB2ENR_USART1EN); break;
    case base::USART2: RCC().APB1ENR |= bit(APB1ENR_USART2EN); break;
    case base::USART3: RCC().APB1ENR |= bit(APB1ENR_USART3EN); break;
    default: break;
    }
}

}  // namespace stm32f100::rcc
