#if ARDUINO
#include "BareCpper/include/BareCpper/ATsamd/SpiAsync_ATsamd5x.hpp"
#include "BareCpper/include/BareCpper/ATsamd/TwiAsync_ATsamd5x.hpp"
#else
#include "BareCpper/ATsamd/SpiAsync_ATsamd5x.hpp"
#include "BareCpper/ATsamd/TwiAsync_ATsamd5x.hpp"
#endif

extern "C"
{
  void SERCOM0_1_Handler()
  {
    BareCpper::sercomTxCallbacks[0]();
  }

  void SERCOM0_2_Handler()
  {
    BareCpper::sercomRxCallbacks[0]();
  }

  void SERCOM1_1_Handler()
  {
    BareCpper::sercomTxCallbacks[1]();
  }

  void SERCOM1_2_Handler()
  {
    BareCpper::sercomRxCallbacks[1]();
  }

  void SERCOM2_0_Handler(void)
  {
    BareCpper::TwiAsync::SERCOM2_Handler();
  }

  void SERCOM2_1_Handler()
  {
    BareCpper::sercomTxCallbacks[2]();
    BareCpper::TwiAsync::SERCOM2_Handler();
  }

  void SERCOM2_2_Handler()
  {
    BareCpper::sercomRxCallbacks[2]();
    BareCpper::TwiAsync::SERCOM2_Handler();
  }

  void SERCOM2_OTHER_Handler()
  {
    BareCpper::TwiAsync::SERCOM2_Handler();
  }

  void SERCOM3_1_Handler()
  {
    BareCpper::sercomTxCallbacks[3]();
  }

  void SERCOM3_2_Handler()
  {
    BareCpper::sercomRxCallbacks[3]();
  }

  void SERCOM4_1_Handler()
  {
    BareCpper::sercomTxCallbacks[4]();
  }

  void SERCOM4_2_Handler()
  {
    BareCpper::sercomRxCallbacks[4]();
  }

  void SERCOM5_1_Handler()
  {
    BareCpper::sercomTxCallbacks[5]();
  }

  void SERCOM5_2_Handler()
  {
    BareCpper::sercomRxCallbacks[5]();
  }
}