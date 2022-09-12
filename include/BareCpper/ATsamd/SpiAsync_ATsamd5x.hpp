#ifndef SPI_ASYNC_ATSAMD5X_HPP
#define SPI_ASYNC_ATSAMD5X_HPP

#if !__SAMD51__
#error "SAMD library error, please check and update accordingly."
#endif

#include <cstdint>
#include <array>      //< std::array
#include <functional> //< std::function
#include "../Gpio.hpp"
#include "../Spi.hpp"

#define CONF_SPIRXEN 0x1
#define CONF_SPICHSIZE 0x0
#define CONF_SPIDUMMYBYTE 0x1ff //< TBC!!
#define CONF_SPIIBON 0x0        ///< Immediate Buffer Overflow Notification
#define CONF_SPIRUNSTDBY 0x0
/// @todo Slave SPI only: #define CONF_SPIAMODE_EN 0 //< AddressMode
/// @todo Slave SPI only: #define CONF_SPIAMODE 0
/// @todo Slave SPI only: #define CONF_SPIADDR 0
/// @todo Slave SPI only: #define CONF_SPIADDRMASK 0
#define CONF_SPISSDE 0    ///< Slave Select Low Detect Enable
#define CONF_SPIMSSEN 0x0 ///< Master Slave Select Enable
#define CONF_SPIPLOADEN 0 ///< Data Preload Enable

#ifndef SERCOM_SPI_CTRLA_MODE_SPI_MASTER
#define SERCOM_SPI_CTRLA_MODE_SPI_MASTER SERCOM_SPI_CTRLA_MODE(0x03) //< @todo Is the CTRLA SPI MASTER  defined somewhere?
#endif
namespace BareCpper
{
  extern std::function<void()> sercomTxCallbacks[SERCOM_INST_NUM];
  extern std::function<void()> sercomRxCallbacks[SERCOM_INST_NUM];
  namespace SAMD51
  {
    template <
        typename MosiPinT
      , typename MisoPinT
      , typename SckPinT
      , typename CsPinT>
    struct SpiPins
    {
      static constexpr MosiPinT mosi = {};
      static constexpr MisoPinT miso = {};
      static constexpr SckPinT sck = {};
      static constexpr CsPinT cs = {};
    };

    template <uint8_t GclkId_CORE_Src, size_t GclkId_CORE_Freq, uint8_t GclkId_SLOW_Src, bool Use32BitExtension = false>
    struct SpiPlatformConfig
    {
      static constexpr uint8_t gclkId_CORE_Src = GclkId_CORE_Src; // CONF_GCLK_SERCOM2_CORE_SRC;
      static constexpr size_t gclkId_CORE_Freq = GclkId_CORE_Freq;
      static constexpr uint8_t gclkId_SLOW_Src = GclkId_SLOW_Src; // CONF_GCLK_SERCOM2_SLOW_SRC;
      static constexpr bool use32BitExtension = Use32BitExtension;
    };

    template <typename DataType>
    struct SpiMessage
    {
      using dataType_t = DataType;

      const dataType_t* txBuffer;
      dataType_t* rxBuffer;
      size_t bufferLength;
      std::function<void()> txCallback;
      std::function<void()> rxCallback;
    };

    template <typename Pins_t, typename PlatformConfig_t>
    class AsyncSpi
    {
    public:
      using platformConfig_t = PlatformConfig_t;

      bool initialise(const Pins_t &pins = {}, const platformConfig_t &platformConfig = {})
      {
        constexpr std::optional<uint8_t> sercomIndex = ATsamd5x::sercomForPins(
            {id(pins.mosi), id(pins.miso), id(pins.sck)}, {
              [](const uint8_t padIndex)
              { return ((padIndex == 0) || (padIndex == 3)); } // MOSI must always be PAD== 0 or 3
              ,
              nullptr //< MISO can be on any PAD
              ,
              [](const uint8_t padIndex)
              { return padIndex == 1; } //< SCK must always be PAD == 1
            });
        static_assert((bool)sercomIndex, "Pin combination {Mosi, Miso, Sck} must map to a valid SERCOM peripheral");
        sercomIndex_ = *sercomIndex; //< needed for calling correct callback function

        platformConfig_ = platformConfig;
        pins_ = pins;
        return initialiseClock(*sercomIndex, platformConfig)
              && initialiseGpio()
              && initialiseDevice(*sercomIndex, pins)
              && initialiseAsync(*sercomIndex)
            ;
      }

      template <typename Config_t>
      bool configure(const Config_t &config = {})
      {
        using BaudBit_t = decltype(hw_->BAUD.reg);
        const BaudBit_t baudBit = static_cast<BaudBit_t>((static_cast<float>(platformConfig_.gclkId_CORE_Freq) / static_cast<float>(2 * config.baudRate)) - 1);

        // SERCOM_CRITICAL_SECTION_ENTER();
        constexpr uint32_t ctrlAMask = SERCOM_SPI_CTRLA_DORD | SERCOM_SPI_CTRLA_CPOL | SERCOM_SPI_CTRLA_CPHA;
        const uint32_t ctrlA = (config.bitOrder == SpiBitOrder::LSBFirst ? SERCOM_SPI_CTRLA_DORD : 0) ///@todo Support selecting slave modes?
                               | (spiClockPolarity(config.mode) == SpiClockPolarity::Cpol1 ? SERCOM_SPI_CTRLA_CPOL : 0) | (spiClockPhase(config.mode) == SpiClockPhase::Cpha1 ? SERCOM_SPI_CTRLA_CPHA : 0);

        hw_->CTRLA.reg = (hw_->CTRLA.reg & ~ctrlAMask) | ctrlA;
        hw_->BAUD.bit.BAUD = baudBit;
        hw_->CTRLC.bit.DATA32B = platformConfig_.use32BitExtension;
        // SERCOM_CRITICAL_SECTION_LEAVE();

        return true;
      }

      bool destroy()
      {
        hw_->CTRLA.bit.ENABLE = false;
        hw_->SYNCBUSY.bit.SWRST = true;
        while (hw_->SYNCBUSY.bit.SWRST | hw_->SYNCBUSY.bit.ENABLE)
          ;
        return true;
      }

      bool enable()
      {
        return enableSync();
      }

      bool disable()
      {
        return disableSync();
      }

      using DataType = std::conditional_t<PlatformConfig_t::use32BitExtension, uint32_t, uint8_t>;
      /**
       * @brief Start async transfer.
       *
       * @param message Buffers and callbacks.
       * @return int32_t -1 if there is transfer in progress,
       * 1 if successfully started new transfer
       */
      int32_t asyncTransfer(const SpiMessage<DataType> &message)
      {
        static size_t iTxBuffer = 0;
        static size_t iRxBuffer = 0;

        if (transferInProgress_)
          return -1;

        iTxBuffer = 0;
        iRxBuffer = 0;

        if (message.txBuffer != nullptr)
        {
          BareCpper::sercomTxCallbacks[sercomIndex_] = [this, message]()
          {
            if (hw_->INTFLAG.reg & SERCOM_SPI_INTFLAG_TXC) ///@todo: not needed, Handler is dedicated to TXC IRQ
            {
              // if all bytes are transferred, disable TXC IRQ
              // check if RXC IRQ is disabled and stop SPI clock if that is the case
              ///@todo race condition on INTENSET_RXC
              if (iTxBuffer == message.bufferLength)
              {
                hw_->INTENCLR.reg |= SERCOM_SPI_INTENCLR_TXC;
                hw_->INTFLAG.reg |= SERCOM_SPI_INTFLAG_TXC;  //< clear TXC flag
                if (!(hw_->INTENSET.reg & SERCOM_SPI_INTENSET_RXC))
                {
                  BareCpper::gpioOutHigh(pins_.cs);
                  disableSync();
                  transferInProgress_ = false;
                }
              }
              else
              {
                // transfer new data, also clears TXC irq flag
                hw_->DATA.reg = message.txBuffer[iTxBuffer++];
              }
              // call user callback
              if(message.txCallback) message.txCallback();
            }
          };
          // enable TXC IRQ
          hw_->INTENSET.reg |= SERCOM_SPI_INTENSET_TXC;
        }

        if (message.rxBuffer != nullptr)
        {
          BareCpper::sercomRxCallbacks[sercomIndex_] = [this, message]()
          {
            if (hw_->INTFLAG.reg & SERCOM_SPI_INTFLAG_RXC)  ///@todo: not needed, Handler is dedicated to RXC IRQ
            {
              if (iRxBuffer < message.bufferLength)
              {
                // read received data, also clears RXC irq flag
                message.rxBuffer[iRxBuffer++] = hw_->DATA.reg;
              }
              // if all bytes are received, disable RXC IRQ
              // check if TXC IRQ is disabled and stop SPI clock if that is the case
              ///@todo race condition on INTENSET_TXC
              else
              {
                hw_->INTENCLR.reg |= SERCOM_SPI_INTENCLR_RXC;
                if (!(hw_->INTENSET.reg & SERCOM_SPI_INTENSET_TXC))
                {
                  BareCpper::gpioOutHigh(pins_.cs);
                  disableSync();
                  transferInProgress_ = false;
                }
              }
              // call user callback
              if(message.rxCallback) message.rxCallback();
            }
          };
          // enable RXC IRQ
          hw_->INTENSET.reg |= SERCOM_SPI_INTENSET_RXC;
        }
        // enable SPI clock
        enableSync();
        // pull CS low
        BareCpper::gpioOutLow(pins_.cs);
        transferInProgress_ = true;
        // send first data
        if(message.txBuffer != nullptr)
        {
          // wait for DRE to become set and send data
          while(!hw_->INTFLAG.bit.DRE);
          hw_->DATA.reg = message.txBuffer[iTxBuffer++];
        }
        return 1;
      }

      int32_t syncTransfer(SpiMessage<DataType> &message)
      {
        if (transferInProgress_)
          return -1;

        volatile size_t iTxBuffer = 0, iRxBuffer = 0;
        message.txCallback = [&iTxBuffer]() { ++iTxBuffer; };
        message.rxCallback = [&iRxBuffer]() { ++iRxBuffer; };

        if(asyncTransfer(message) < 0) return -1;

        if(message.txBuffer == nullptr)
        {
          while((iRxBuffer < message.bufferLength));
        }
        else if(message.rxBuffer == nullptr)
        {
          while((iTxBuffer < message.bufferLength));
        }
        else
        {
          while((iTxBuffer < message.bufferLength) || (iRxBuffer < message.bufferLength));  //< wait for transfer to finish          
        }

        return 1;
      }

    protected:
      bool enableSync()
      {
        // SERCOM_CRITICAL_SECTION_ENTER();
        hw_->CTRLA.bit.ENABLE = true;
        while (hw_->SYNCBUSY.bit.SWRST | hw_->SYNCBUSY.bit.ENABLE)
          ;
        // SERCOM_CRITICAL_SECTION_LEAVE();
        return true;
      }

      bool enableAsync()
      {
        // SERCOM_CRITICAL_SECTION_ENTER();
        hw_->CTRLA.bit.ENABLE = true;
        // SERCOM_CRITICAL_SECTION_LEAVE();
        return true;
      }

      bool disableSync()
      {
        // SERCOM_CRITICAL_SECTION_ENTER();
        hw_->CTRLA.bit.ENABLE = false;
        while (hw_->SYNCBUSY.bit.SWRST | hw_->SYNCBUSY.bit.ENABLE)
          ;
        // SERCOM_CRITICAL_SECTION_LEAVE();
        return true;
      }

      bool disableAsync()
      {
        hw_->CTRLA.bit.ENABLE = false;

        return true;
      }

      bool initialiseClock(const uint8_t sercomIndex, const platformConfig_t &platformConfig)
      {
        /// @TODO Power-Saving: TBC if Slow-Clock even necessary

        const ATsamd5x::SercomClocks params = ATsamd5x::sercomClocks(sercomIndex);

        GCLK->PCHCTRL[params.gclkId_CORE].bit.CHEN = 0; // Disable timer
        GCLK->PCHCTRL[params.gclkId_SLOW].bit.CHEN = 0; // Disable timer

        while (GCLK->PCHCTRL[params.gclkId_SLOW].bit.CHEN || GCLK->PCHCTRL[params.gclkId_CORE].bit.CHEN); // Wait for disable

        // GCLK_CRITICAL_SECTION_ENTER();
        GCLK->PCHCTRL[params.gclkId_CORE].reg = platformConfig.gclkId_CORE_Src | GCLK_PCHCTRL_CHEN;
        GCLK->PCHCTRL[params.gclkId_SLOW].reg = platformConfig.gclkId_SLOW_Src | GCLK_PCHCTRL_CHEN;
        // GCLK_CRITICAL_SECTION_LEAVE();

        // MCLK_CRITICAL_SECTION_ENTER();
        ATsamd5x::sercomApbEnable(sercomIndex);
        // MCLK_CRITICAL_SECTION_LEAVE();

        while (!GCLK->PCHCTRL[params.gclkId_CORE].bit.CHEN && !GCLK->PCHCTRL[params.gclkId_SLOW].bit.CHEN); // Wait for clock enable

        return true;
      }

      bool initialiseDevice(const uint8_t sercomIndex, const Pins_t &pins)
      {
        ::Sercom *sercom = ATsamd5x::sercom(sercomIndex);
        if (!sercom)
          return false;
        hw_ = &sercom->SPI;

        if (!hw_->SYNCBUSY.bit.SWRST)
        {
          if (hw_->CTRLA.bit.ENABLE) // Disable
          {
            hw_->CTRLA.bit.ENABLE = false;
            while (hw_->SYNCBUSY.bit.ENABLE);
          }

          hw_->CTRLA.reg = SERCOM_SPI_CTRLA_SWRST | SERCOM_SPI_CTRLA_MODE_SPI_MASTER; // Master reset
        }

        while (hw_->SYNCBUSY.bit.SWRST); // Wait for reset

        const uint8_t mosiPad = *ATsamd5x::sercomPinPad(sercomIndex, pins.mosi); //<@note We know the result shall be valid derefernece as this is checked via sercomForPins()
        const uint8_t misoPad = *ATsamd5x::sercomPinPad(sercomIndex, pins.miso);
        const uint8_t sckPad = *ATsamd5x::sercomPinPad(sercomIndex, pins.sck);

        const bool mosiValid = ((mosiPad == 0) || (mosiPad == 3)); // MOSI must always be PAD== 0 or 3
        const bool misoValid = ((misoPad >= 0) || (misoPad <= 3)); // MISO can be on any PAD
        const bool sckValid = (sckPad == 1);                       // SCK must always be PAD==1

        if (!(mosiValid && misoValid && sckValid))
        {
          return false;
        }

        const uint8_t DIPO = misoPad;
        const uint8_t DOPO = (mosiPad == 0) ? 0 : 2;

        // SERCOM_CRITICAL_SECTION_ENTER();
        const uint32_t ctrlA = SERCOM_SPI_CTRLA_MODE_SPI_MASTER
                              /// @todo Slave SPI only:  | (CONF_SPIAMODE_EN ? SERCOM_SPI_CTRLA_FORM(2) : SERCOM_SPI_CTRLA_FORM(0))
                              | SERCOM_SPI_CTRLA_DOPO(DOPO)
                              | SERCOM_SPI_CTRLA_DIPO(DIPO) 
                              | (CONF_SPIIBON ? SERCOM_SPI_CTRLA_IBON : 0) 
                              | (CONF_SPIRUNSTDBY ? SERCOM_SPI_CTRLA_RUNSTDBY : 0);
        const uint32_t ctrlB = ((CONF_SPIRXEN ? SERCOM_SPI_CTRLB_RXEN : 0) 
                              | (CONF_SPIMSSEN ? SERCOM_SPI_CTRLB_MSSEN : 0)
                              | (CONF_SPISSDE ? SERCOM_SPI_CTRLB_SSDE : 0)
                              | (CONF_SPIPLOADEN ? SERCOM_SPI_CTRLB_PLOADEN : 0)
                              /// @todo Slave SPI only: | SERCOM_SPI_CTRLB_AMODE(CONF_SPIAMODE)
                              | SERCOM_SPI_CTRLB_CHSIZE(CONF_SPICHSIZE));

        hw_->CTRLA.reg = ctrlA;
        hw_->CTRLB.reg = ctrlB;

        /// @todo Slave SPI only:  |addr = (SERCOM_SPI_ADDR_ADDR(CONF_SPIADDR) | SERCOM_SPI_ADDR_ADDRMASK(CONF_SPIADDRMASK));
        hw_->DBGCTRL.bit.DBGSTOP = true;
        // SERCOM_CRITICAL_SECTION_LEAVE();

        return true;
      }

      bool initialiseAsync(const uint8_t sercomIndex)
      {
        // enable RXC and TXC irqs in NVIC
        // TXC is at bit pos 1 in INTFLAG, so enable SERCOMn_1 IRQ
        const IRQn_Type txcIrqn = static_cast<IRQn_Type>(SERCOM0_1_IRQn + sercomIndex * 4);
        // RXC is at bit pos 2 in INTFLAG, so enable SERCOMn_2 IRQ
        const IRQn_Type rxcIrqn = static_cast<IRQn_Type>(SERCOM0_2_IRQn + sercomIndex * 4);
        NVIC_ClearPendingIRQ(txcIrqn);
        NVIC_SetPriority(txcIrqn, 1);
        NVIC_EnableIRQ(txcIrqn);
        NVIC_ClearPendingIRQ(rxcIrqn);
        NVIC_SetPriority(rxcIrqn, 1);
        NVIC_EnableIRQ(rxcIrqn);

        return true;
      }

      bool initialiseGpio()
      {
        gpioDirectionIn(pins_.miso);// Set pin direction to input
        gpioPullDisable(pins_.miso);

        gpioOutLow(pins_.sck); ///< Initial level
        gpioDirectionOut(pins_.sck);// Set pin direction to output

        gpioOutLow(pins_.mosi);
        gpioDirectionOut(pins_.mosi); // Set pin direction to output

        gpioDirectionOut(pins_.cs);
        gpioOutHigh(pins_.cs);

        gpioFunction(pins_.miso, ATsamd5x::sercomPinPeripheral(sercomIndex_, pins_.miso));
        gpioFunction(pins_.mosi, ATsamd5x::sercomPinPeripheral(sercomIndex_, pins_.mosi));
        gpioFunction(pins_.sck, ATsamd5x::sercomPinPeripheral(sercomIndex_, pins_.sck));

        return true;
      }

    private:
      SercomSpi *hw_ = nullptr;
      platformConfig_t platformConfig_;
      Pins_t pins_;
      uint8_t sercomIndex_;
      bool transferInProgress_ = false;
    };
  }
} // END: BareCpper

#endif // SPI_ASYNC_ATSAMD5X_HPP