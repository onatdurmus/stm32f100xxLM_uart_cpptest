#include "stm32f100xxLM_gpio.hpp"
#include "stm32f100xxLM_rcc.hpp"

namespace stm32f100 {

namespace {

// CNF[1:0] bits for each mode (see RM0041, GPIO configuration tables).
constexpr std::uint32_t cnfBits(GpioPin::Mode mode)
{
    switch (mode)
    {
    case GpioPin::Mode::InputAnalog:     return 0x0U;
    case GpioPin::Mode::InputFloating:   return 0x1U;
    case GpioPin::Mode::InputPullUp:     return 0x2U;
    case GpioPin::Mode::InputPullDown:   return 0x2U;  // Same CNF as pull-up, ODR selects the direction.
    case GpioPin::Mode::OutputPushPull:  return 0x0U;
    case GpioPin::Mode::OutputOpenDrain: return 0x1U;
    case GpioPin::Mode::AltFnPushPull:   return 0x2U;
    case GpioPin::Mode::AltFnOpenDrain:  return 0x3U;
    }
    return 0x1U;  // Reset state: floating input
}

constexpr bool isInput(GpioPin::Mode mode)
{
    return mode <= GpioPin::Mode::InputPullDown;
}

}  // namespace

void GpioPin::init(Mode mode, Speed speed) const
{
    rcc::enableGpioClock(port_);

    if (!isInput(mode) && speed == Speed::Input)
    {
        speed = Speed::Max2MHz;  // An output cannot have MODE = 00, choose a safe default.
    }

    const std::uint32_t modeBits = isInput(mode) ? 0U : static_cast<std::uint32_t>(speed);
    const std::uint32_t field    = (cnfBits(mode) << 2) | modeBits;

    // Pins 0..7 are configured in CRL, pins 8..15 in CRH. Each pin has a 4-bit field.
    volatile std::uint32_t& cr = (pin_ < 8U) ? regs().CRL : regs().CRH;
    const unsigned shift = (pin_ % 8U) * 4U;
    cr = (cr & ~(0xFU << shift)) | (field << shift);

    // Pull direction is selected through ODR. BSRR/BRR change one pin atomically.
    if (mode == Mode::InputPullUp)
    {
        regs().BSRR = bit(pin_);
    }
    else if (mode == Mode::InputPullDown)
    {
        regs().BRR = bit(pin_);
    }
}

bool GpioPin::read() const
{
    return (regs().IDR & bit(pin_)) != 0U;
}

}  // namespace stm32f100
