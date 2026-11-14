#ifndef STM32F100XXLM_GPIO_HPP
#define STM32F100XXLM_GPIO_HPP

#include "stm32f100xxLM.hpp"

namespace stm32f100 {

// One GPIO pin. The constructor is constexpr, so global pin objects are
// constant-initialized and do not depend on C++ static constructors at startup.
class GpioPin
{
public:
    enum class Mode : std::uint8_t
    {
        InputAnalog,
        InputFloating,
        InputPullUp,
        InputPullDown,
        OutputPushPull,
        OutputOpenDrain,
        AltFnPushPull,
        AltFnOpenDrain,
    };

    // MODE[1:0] bits. Input pins always use 00.
    enum class Speed : std::uint8_t
    {
        Input    = 0x0,
        Max10MHz = 0x1,
        Max2MHz  = 0x2,
        Max50MHz = 0x3,
    };

    constexpr GpioPin(std::uintptr_t portBase, std::uint8_t pinNumber)
        : port_(portBase), pin_(pinNumber)
    {
    }

    void init(Mode mode, Speed speed = Speed::Input) const;
    bool read() const;

private:
    GpioRegs& regs() const { return stm32f100::regs<GpioRegs>(port_); }

    std::uintptr_t port_;
    std::uint8_t pin_;
};

}  // namespace stm32f100

#endif  // STM32F100XXLM_GPIO_HPP
