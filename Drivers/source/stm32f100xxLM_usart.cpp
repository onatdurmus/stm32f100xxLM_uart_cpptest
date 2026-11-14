#include "stm32f100xxLM_usart.hpp"
#include "stm32f100xxLM_rcc.hpp"

#include <cstring>

namespace stm32f100 {

namespace {

// USART_SR bits
constexpr unsigned SR_RXNE = 5;
constexpr unsigned SR_TC   = 6;
constexpr unsigned SR_TXE  = 7;

// USART_CR1 bits
constexpr unsigned CR1_RE     = 2;
constexpr unsigned CR1_TE     = 3;
constexpr unsigned CR1_RXNEIE = 5;
constexpr unsigned CR1_TCIE   = 6;
constexpr unsigned CR1_TXEIE  = 7;
constexpr unsigned CR1_PS     = 9;
constexpr unsigned CR1_PCE    = 10;
constexpr unsigned CR1_M      = 12;
constexpr unsigned CR1_UE     = 13;
constexpr unsigned CR1_OVER8  = 15;

// USART_CR2 bits
constexpr unsigned CR2_STOP = 12;  // [13:12]

// USART_CR3 bits
constexpr unsigned CR3_RTSE = 8;
constexpr unsigned CR3_CTSE = 9;

// USART_BRR fields
constexpr unsigned BRR_MANTISSA = 4;

}  // namespace

void Usart::init(const Config& config, UsartListener* listener)
{
    config_   = config;
    listener_ = listener;
    txState_  = State::Ready;
    rxState_  = State::Ready;

    rcc::enableUsartClock(base_);

    UsartRegs& r = regs();

    // CR1: enable, word length, parity, direction. Interrupt enables start cleared.
    std::uint32_t cr1 = bit(CR1_UE);
    if (config.wordLength == WordLength::Bits9)
    {
        cr1 |= bit(CR1_M);
    }
    if (config.parity != Parity::None)
    {
        cr1 |= bit(CR1_PCE);
        if (config.parity == Parity::Odd)
        {
            cr1 |= bit(CR1_PS);
        }
    }
    if (config.direction != Direction::RxOnly)
    {
        cr1 |= bit(CR1_TE);
    }
    if (config.direction != Direction::TxOnly)
    {
        cr1 |= bit(CR1_RE);
    }
    r.CR1 = cr1;

    // CR2: stop bits. The enum values match the register encoding.
    r.CR2 = (r.CR2 & ~(0x3U << CR2_STOP)) |
            (static_cast<std::uint32_t>(config.stopBits) << CR2_STOP);

    // CR3: hardware flow control
    std::uint32_t cr3 = r.CR3 & ~(bit(CR3_RTSE) | bit(CR3_CTSE));
    if (config.flowControl == FlowControl::Cts || config.flowControl == FlowControl::CtsRts)
    {
        cr3 |= bit(CR3_CTSE);
    }
    if (config.flowControl == FlowControl::Rts || config.flowControl == FlowControl::CtsRts)
    {
        cr3 |= bit(CR3_RTSE);
    }
    r.CR3 = cr3;

    setBaudRate(config.baudRate);

    nvic::enable(irq_);
}

void Usart::setBaudRate(std::uint32_t baudRate)
{
    const std::uint32_t pclk = (base_ == base::USART1) ? rcc::getPclk2() : rcc::getPclk1();
    UsartRegs& r = regs();

    // USARTDIV = PCLK / (8 * (2 - OVER8) * baud)
    // With oversampling by 16, BRR (mantissa << 4 | fraction) equals PCLK / baud directly.
    // Rounding to the nearest integer also carries a rounded-up fraction into the mantissa.
    if (r.CR1 & bit(CR1_OVER8))
    {
        const std::uint32_t div8     = (2U * pclk + baudRate / 2U) / baudRate;  // USARTDIV * 8
        const std::uint32_t mantissa = div8 >> 3;
        const std::uint32_t fraction = div8 & 0x7U;  // DIV_Fraction[3] must stay 0
        r.BRR = (mantissa << BRR_MANTISSA) | fraction;
    }
    else
    {
        r.BRR = (pclk + baudRate / 2U) / baudRate;  // USARTDIV * 16
    }
}

bool Usart::transmitIT(const std::uint8_t* data, std::uint32_t length)
{
    if (txState_ == State::Busy || data == nullptr || length == 0U)
    {
        return false;
    }

    txBuffer_ = data;
    txLen_    = length;
    txState_  = State::Busy;

    // The ISR also writes CR1, so the read-modify-write is done with this IRQ masked.
    nvic::IrqGuard guard(irq_);
    regs().CR1 |= bit(CR1_TXEIE);
    return true;
}

bool Usart::receiveIT(std::uint8_t* buffer, std::uint32_t length)
{
    if (rxState_ == State::Busy || buffer == nullptr || length == 0U)
    {
        return false;
    }

    rxBuffer_ = buffer;
    rxLen_    = length;
    rxState_  = State::Busy;

    nvic::IrqGuard guard(irq_);
    regs().CR1 |= bit(CR1_RXNEIE);
    return true;
}

void Usart::irqHandler()
{
    UsartRegs& r = regs();
    const std::uint32_t sr  = r.SR;  // Reading SR is the first step of the TC/RXNE clear sequences.
    const std::uint32_t cr1 = r.CR1;

    // TC: the last frame has left the shift register. TCIE is only enabled after the last TXE.
    if ((sr & bit(SR_TC)) && (cr1 & bit(CR1_TCIE)))
    {
        r.CR1 &= ~bit(CR1_TCIE);
        txState_ = State::Ready;
        notify(UsartEvent::TxComplete);
    }

    // TXE: data register is empty, write the next frame.
    if ((sr & bit(SR_TXE)) && (cr1 & bit(CR1_TXEIE)) && txState_ == State::Busy)
    {
        if (nineDataBits())
        {
            // 9 data bits take two bytes in the buffer. memcpy avoids unaligned/aliasing problems.
            std::uint16_t frame;
            std::memcpy(&frame, txBuffer_, sizeof(frame));
            r.DR = frame & 0x01FFU;
            txBuffer_ += 2;
        }
        else
        {
            r.DR = *txBuffer_++;  // With parity enabled, hardware replaces the MSB with the parity bit.
        }

        if (--txLen_ == 0U)
        {
            // Stop TXE interrupts and wait for TC before reporting completion.
            r.CR1 = (r.CR1 & ~bit(CR1_TXEIE)) | bit(CR1_TCIE);
        }
    }

    // RXNE: a frame was received.
    if ((sr & bit(SR_RXNE)) && (cr1 & bit(CR1_RXNEIE)) && rxState_ == State::Busy)
    {
        const std::uint32_t data = r.DR;  // Reading DR clears RXNE.

        if (nineDataBits())
        {
            const std::uint16_t frame = static_cast<std::uint16_t>(data & 0x01FFU);
            std::memcpy(rxBuffer_, &frame, sizeof(frame));
            rxBuffer_ += 2;
        }
        else
        {
            // 8-bit word with parity has 7 data bits; otherwise 8 (or 9 with parity) -> 8 data bits.
            const bool sevenDataBits =
                config_.wordLength == WordLength::Bits8 && config_.parity != Parity::None;
            *rxBuffer_++ = static_cast<std::uint8_t>(data & (sevenDataBits ? 0x7FU : 0xFFU));
        }

        if (--rxLen_ == 0U)
        {
            r.CR1 &= ~bit(CR1_RXNEIE);
            rxState_ = State::Ready;
            notify(UsartEvent::RxComplete);
        }
    }
}

}  // namespace stm32f100
