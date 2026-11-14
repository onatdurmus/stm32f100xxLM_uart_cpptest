#ifndef STM32F100XXLM_HPP
#define STM32F100XXLM_HPP

// Device header for the STM32F100xx low/medium density MCU (C++ version).
// Contains only what the UART test needs: GPIO, USART, RCC registers and NVIC.

#include <cstddef>
#include <cstdint>

namespace stm32f100 {

/* ---------------- Peripheral base addresses ---------------- */
namespace base {
constexpr std::uintptr_t GPIOA  = 0x40010800U;
constexpr std::uintptr_t GPIOB  = 0x40010C00U;
constexpr std::uintptr_t GPIOC  = 0x40011000U;
constexpr std::uintptr_t GPIOD  = 0x40011400U;
constexpr std::uintptr_t GPIOE  = 0x40011800U;

constexpr std::uintptr_t USART1 = 0x40013800U;  // APB2
constexpr std::uintptr_t USART2 = 0x40004400U;  // APB1
constexpr std::uintptr_t USART3 = 0x40004800U;  // APB1

constexpr std::uintptr_t RCC    = 0x40021000U;

constexpr std::uintptr_t NVIC_ISER = 0xE000E100U;  // Interrupt Set-Enable registers
constexpr std::uintptr_t NVIC_ICER = 0xE000E180U;  // Interrupt Clear-Enable registers
}  // namespace base

/* ---------------- Register layouts ---------------- */
struct GpioRegs
{
    volatile std::uint32_t CRL;
    volatile std::uint32_t CRH;
    volatile std::uint32_t IDR;
    volatile std::uint32_t ODR;
    volatile std::uint32_t BSRR;
    volatile std::uint32_t BRR;
    volatile std::uint32_t LCKR;
};

struct UsartRegs
{
    volatile std::uint32_t SR;
    volatile std::uint32_t DR;
    volatile std::uint32_t BRR;
    volatile std::uint32_t CR1;
    volatile std::uint32_t CR2;
    volatile std::uint32_t CR3;
    volatile std::uint32_t GTPR;
};

struct RccRegs
{
    volatile std::uint32_t CR;
    volatile std::uint32_t CFGR;
    volatile std::uint32_t CIR;
    volatile std::uint32_t APB2RSTR;
    volatile std::uint32_t APB1RSTR;
    volatile std::uint32_t AHBENR;
    volatile std::uint32_t APB2ENR;
    volatile std::uint32_t APB1ENR;
    volatile std::uint32_t BDCR;
    volatile std::uint32_t CSR;
    volatile std::uint32_t Reserved;
    volatile std::uint32_t CFGR2;
};

// Compile-time checks that the structs match the reference manual offsets.
static_assert(offsetof(GpioRegs, LCKR) == 0x18, "GPIO register layout mismatch");
static_assert(offsetof(UsartRegs, GTPR) == 0x18, "USART register layout mismatch");
static_assert(offsetof(RccRegs, CFGR2) == 0x2C, "RCC register layout mismatch");

/* ---------------- Helpers ---------------- */
template <typename Regs>
inline Regs& regs(std::uintptr_t address)
{
    return *reinterpret_cast<Regs*>(address);
}

inline RccRegs& RCC() { return regs<RccRegs>(base::RCC); }

constexpr std::uint32_t bit(unsigned position) { return 1U << position; }

/* ---------------- NVIC ---------------- */
enum class Irq : std::uint8_t
{
    Usart1 = 37,
    Usart2 = 38,
    Usart3 = 39,
};

namespace nvic {
// ISER/ICER are write-1-to-act registers, writing 0 has no effect, so no read-modify-write is needed.
inline void enable(Irq irq)
{
    const auto n = static_cast<std::uint8_t>(irq);
    reinterpret_cast<volatile std::uint32_t*>(base::NVIC_ISER)[n / 32U] = bit(n % 32U);
}

inline void disable(Irq irq)
{
    const auto n = static_cast<std::uint8_t>(irq);
    reinterpret_cast<volatile std::uint32_t*>(base::NVIC_ICER)[n / 32U] = bit(n % 32U);
}

// RAII guard: masks one IRQ for the lifetime of the object.
// Used to protect read-modify-write of registers that the ISR also modifies.
class IrqGuard
{
public:
    explicit IrqGuard(Irq irq) : irq_(irq) { disable(irq_); }
    ~IrqGuard() { enable(irq_); }
    IrqGuard(const IrqGuard&) = delete;
    IrqGuard& operator=(const IrqGuard&) = delete;

private:
    Irq irq_;
};
}  // namespace nvic

}  // namespace stm32f100

#endif  // STM32F100XXLM_HPP
