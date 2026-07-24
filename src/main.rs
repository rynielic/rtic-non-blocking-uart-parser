#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f3xx_hal::pac, dispatchers = [SPI1])]
mod app {
      use stm32f3xx_hal::prelude::*;

      #[shared]
      struct Shared {}

      #[local]
      struct Local {}

      #[init]
      fn init (_: init::Context) -> (Shared, Local) {
            // setup

            (Shared {}, Local {})
      }
      
      #[idle]
      fn idle (_: idle::Context) -> ! {
            loop {
                  // pwm works as a background task
            }
      }

      #[task(binds = DMA1_CH5, priority = 2)]
      fn uart_rx (_: uart_rx::Context) {
            // fills the uart buffer, if \n byte incomes,
            // uart_paser.spawn()
      }

      #[task(binds = EXTI0, priority = 3)]
      fn stop_button (_: stop_button::Context) {
            // stops pwm until Resume command incomes
      }

      #[task(priority = 1)]
      async fn uart_parser(_: uart_parser::Context) {
            // parsing uart buffer
      }
}