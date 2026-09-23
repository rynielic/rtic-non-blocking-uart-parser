#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f3xx_hal::pac, dispatchers = [SPI1])]
mod app {
      use cortex_m::singleton;
      use defmt::println;
      use stm32f3xx_hal::{
            dma::{Channel, Direction::FromPeripheral, Increment::{Disable, Enable}, Target}, gpio::Edge, pac::{DMA1, USART1}, prelude::*, serial::{Event, Serial},
      };
      use embedded_hal::*;

      type Uart1TxPin = stm32f3xx_hal::gpio::Pin<stm32f3xx_hal::gpio::Gpioa, stm32f3xx_hal::gpio::U<9>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>;
      type Uart1RxPin = stm32f3xx_hal::gpio::Pin<stm32f3xx_hal::gpio::Gpioa, stm32f3xx_hal::gpio::U<10>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>;
      type StopButtonPin = stm32f3xx_hal::gpio::Pin<stm32f3xx_hal::gpio::Gpioa, stm32f3xx_hal::gpio::U<0>, stm32f3xx_hal::gpio::Input>;

      #[shared]
      struct Shared {}

      #[local]
      struct Local {
            stop_button: StopButtonPin,
            uart1: Serial<USART1, (Uart1TxPin, Uart1RxPin)>,
            uart1_rx_channel: stm32f3xx_hal::dma::dma1::C5,
            rx_buffer_addr: u32,
            last_cndtr: u16,
            rx_buffer_length: u16,
      }

      #[init]
      fn init (cx: init::Context) -> (Shared, Local) {
            // Setup the device and core peripherals
            let dp = cx.device;
            let cp = cx.core;

            // Clock
            let mut rcc = dp.RCC.constrain();
            let mut flash = dp.FLASH.constrain();
            let clock = rcc.cfgr
                  .use_hse(8.MHz())
                  .sysclk(72.MHz())
                  .pclk1(36.MHz())
                  .pclk2(72.MHz())
                  .freeze(&mut flash.acr);

            // GPIO
            let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
            let mut stop_button = gpioa
                  .pa0
                  .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
            let mut uart1_tx = gpioa
                  .pa9
                  .into_af_push_pull::<7>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
            let mut uart1_rx = gpioa
                  .pa10
                  .into_af_push_pull::<7>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

            // UART1
            let mut uart1 = Serial::new(
                  dp.USART1,
                  (uart1_tx, uart1_rx),
                  115_200.Bd(),
                  clock,
                  &mut rcc.apb2,
            );
            let usart1_rdr: u32 = unsafe {&(*USART1::ptr()).rdr as *const _ as u32 };
            uart1.enable_dma();
            
            // EXTI0
            let mut syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);
            let mut exti = dp.EXTI;
            syscfg.select_exti_interrupt_source(&stop_button);
            stop_button.trigger_on_edge(&mut exti, Edge::Rising);
            stop_button.enable_interrupt(&mut exti);
            uart1.enable_interrupt(Event::Idle);

            // DMA 1 channel 5
            let dma1 = dp.DMA1.split(&mut rcc.ahb);
            let mut uart1_rx_channel= dma1.ch5;
            
            // DMA buffer
            let rx_buffer: &'static mut [u8; 64] = singleton!(: [u8; 64] = [0;64]).unwrap();
            let rx_buffer_length = rx_buffer.len() as u16;
            let rx_buffer_addr = rx_buffer.as_mut_ptr() as u32;
            let last_cndtr: u16;

            // DMA configuration
            unsafe {
                  uart1_rx_channel.set_peripheral_address(usart1_rdr, Disable);
                  uart1_rx_channel.set_memory_address(rx_buffer_addr, Enable);
                  uart1_rx_channel.set_transfer_length(rx_buffer.len() as u16);
                  uart1_rx_channel.set_word_size::<u8>();
                  uart1_rx_channel.set_direction(FromPeripheral);

                  // Circ bit enabled
                  (*stm32f3xx_hal::pac::DMA1::ptr()).ch5.cr.modify(|_, w| w.circ().set_bit());

                  last_cndtr = (*stm32f3xx_hal::pac::DMA1::ptr()).ch5.ndtr.read().ndt().bits();

                  // Prevents from compiler reordering  
                  core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::Release);

                  uart1_rx_channel.enable();
            }


            (Shared {}, Local {stop_button, uart1, uart1_rx_channel, rx_buffer_addr, last_cndtr, rx_buffer_length})
      }
      
      #[idle(local = [uart1])]
      fn idle (cx: idle::Context) -> ! {
            let uart1 = cx.local.uart1;

            let idle = unsafe { (*stm32f3xx_hal::pac::USART1::ptr()).isr.read().idle().bit_is_set() };
            println!("idle interrupt is {}", idle);

            println!("before write");
            for &b in b"HI from idle\n" {
                  nb::block!(uart1.write(b)).unwrap();
            }
            println!("after write");

            loop {
                  
            }
      }

      #[task(binds = USART1_EXTI25, local = [last_cndtr, rx_buffer_length, rx_buffer_addr], priority = 2)]
      fn uart_rx (cx: uart_rx::Context) {

            // Clear USART1_IDLE interrupt 
            unsafe { (*stm32f3xx_hal::pac::USART1::ptr()).icr.write(|w| w.idlecf().set_bit()) };
            println!("uart_rx task started");

            // Declare the locals
            let last_cndtr = cx.local.last_cndtr;
            let rx_buffer_length = *cx.local.rx_buffer_length;
            let rx_buffer_addr = *cx.local.rx_buffer_addr;

            // Unsafe to access the DMA1_CNDRT register 
            let current_cndtr = unsafe { (*stm32f3xx_hal::pac::DMA1::ptr()).ch5.ndtr.read().ndt().bits() };

            let delta = (*last_cndtr + rx_buffer_length - current_cndtr) % rx_buffer_length;
            let mut cmd_buf = [0u8; 16];

            for i in 0..delta {
                  // Calculate the physical index in the buffer
                  let index = (rx_buffer_length - *last_cndtr + i) % rx_buffer_length;
                  
                  // Perform a volatile read so the compiler doesn't cache stale memory
                  let byte = unsafe { 
                        let rx_buffer_ptr = (rx_buffer_addr as *const u8).add(index as usize);
                        core::ptr::read_volatile(rx_buffer_ptr)
                  };

                  cmd_buf[i as usize] = byte;
            }
            uart_parser::spawn(cmd_buf, delta).unwrap();
      }

      #[task(binds = EXTI0, priority = 3)]
      fn stop_button (_: stop_button::Context) {
            // stops pwm until Resume command incomes
      }

      #[task(priority = 1)]
      async fn uart_parser(_: uart_parser::Context, cmd_buf: [u8; 16], length: u16) {
            println!("got: {:?}", cmd_buf);
            let mut count: usize = 0;

            // parsing example
            for i in b"HI from idle\n" {
                  if *i == cmd_buf[count] {
                        count += 1;
                        if count as u16 == length {
                              println!("success");
                        }
                  }
                  else {
                        println!("0");
                        break;
                  }
                  
            }
      }
}