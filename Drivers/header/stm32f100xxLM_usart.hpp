#ifndef STM32F100XXLM_USART_HPP
#define STM32F100XXLM_USART_HPP

#include "stm32f100xxLM.hpp"

namespace stm32f100 {

class Usart;

enum class UsartEvent : std::uint8_t
{
    TxComplete,
    RxComplete,
};

// The application implements this interface to get notified from the ISR.
// Replaces the weak USART_ApplicationCallback() function of the C driver.
class UsartListener
{
public:
    virtual void onUsartEvent(Usart& usart, UsartEvent event) = 0;

protected:
    ~UsartListener() = default;  // Not deleted through a base pointer.
};

class Usart
{
public:
    enum class WordLength : std::uint8_t { Bits8, Bits9 };
    enum class StopBits : std::uint8_t { One = 0, Half = 1, Two = 2, OneAndHalf = 3 };  // Matches CR2.STOP encoding
    enum class Parity : std::uint8_t { None, Even, Odd };
    enum class Direction : std::uint8_t { TxOnly, RxOnly, TxRx };
    enum class FlowControl : std::uint8_t { None, Cts, Rts, CtsRts };

    struct Config
    {
        std::uint32_t baudRate  = 115200U;
        WordLength wordLength   = WordLength::Bits8;
        StopBits stopBits       = StopBits::One;
        Parity parity           = Parity::None;
        Direction direction     = Direction::TxRx;
        FlowControl flowControl = FlowControl::None;
    };

    constexpr Usart(std::uintptr_t usartBase, Irq irq) : base_(usartBase), irq_(irq) {}

    // A driver object owns a hardware peripheral, so it must not be copied.
    Usart(const Usart&) = delete;
    Usart& operator=(const Usart&) = delete;

    void init(const Config& config, UsartListener* listener = nullptr);

    // Interrupt based transfer. Returns false if the channel is busy or the arguments are invalid.
    bool transmitIT(const std::uint8_t* data, std::uint32_t length);
    bool receiveIT(std::uint8_t* buffer, std::uint32_t length);

    // Sends a string literal without its '\0'. The length is known at compile time.
    template <std::size_t N>
    bool transmitIT(const char (&text)[N])
    {
        return transmitIT(reinterpret_cast<const std::uint8_t*>(text), static_cast<std::uint32_t>(N - 1U));
    }

    // Must be called from the USARTx_IRQHandler.
    void irqHandler();

    std::uint32_t rxRemaining() const { return rxLen_; }
    std::uint32_t txRemaining() const { return txLen_; }
    bool txBusy() const { return txState_ == State::Busy; }
    bool rxBusy() const { return rxState_ == State::Busy; }

private:
    enum class State : std::uint8_t { Ready, Busy };

    UsartRegs& regs() const { return stm32f100::regs<UsartRegs>(base_); }
    void setBaudRate(std::uint32_t baudRate);
    bool nineDataBits() const
    {
        return config_.wordLength == WordLength::Bits9 && config_.parity == Parity::None;
    }
    void notify(UsartEvent event)
    {
        if (listener_ != nullptr)
        {
            listener_->onUsartEvent(*this, event);
        }
    }

    std::uintptr_t base_;
    Irq irq_;
    Config config_{};
    UsartListener* listener_ = nullptr;

    const std::uint8_t* txBuffer_ = nullptr;
    std::uint8_t* rxBuffer_       = nullptr;

    // Shared between main and the ISR, so they are volatile.
    volatile std::uint32_t txLen_ = 0U;
    volatile std::uint32_t rxLen_ = 0U;
    volatile State txState_       = State::Ready;
    volatile State rxState_       = State::Ready;
};

}  // namespace stm32f100

#endif  // STM32F100XXLM_USART_HPP
