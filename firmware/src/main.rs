#![no_main]
#![no_std]

mod command_parser;
mod pwm_control;

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f3xx_hal::pac, dispatchers = [SPI1])]
mod app {
      use crate::pwm_control::{set_duty, timer_state, apply_pwm_duty};
      use crate::command_parser::{parse_command, ParsedCommand, CommandError};

      use core::{fmt::Write, sync::atomic::Ordering::Release};
      use defmt::println;
      use heapless::String;
      use cortex_m::{asm::wfi, singleton};
      use stm32f3xx_hal::{
            dma::{
                  Channel, 
                  Direction::FromPeripheral, 
                  Increment::{Disable, Enable}, 
                  Target
            }, 
            gpio::Edge, 
            pac::{CRC, DMA1, EXTI, RCC, TIM3, USART1}, prelude::*, serial::{Event, Serial},
      };

      type Uart1TxPin = stm32f3xx_hal::gpio::Pin<stm32f3xx_hal::gpio::Gpioc, stm32f3xx_hal::gpio::U<4>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>;
      type Uart1RxPin = stm32f3xx_hal::gpio::Pin<stm32f3xx_hal::gpio::Gpioc, stm32f3xx_hal::gpio::U<5>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>;
      
      #[shared]
      struct Shared {
            uart1: Serial<USART1, (Uart1TxPin, Uart1RxPin)>,
            pwm_duty: u8,
            pwm_change_flag: bool,
            tim3_ch1_enabled: bool,
      }

      #[local]
      struct Local {
            rx_buffer_addr: u32,
            rx_buffer_length: u16,
            last_cndtr: u16,
            cmd_index: usize,
            cmd: [u8; 32],
      }

      #[init]
      fn init (cx: init::Context) -> (Shared, Local) {
            // Setup the device peripherals
            let dp = cx.device;

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
            let mut gpioc = dp.GPIOC.split(&mut rcc.ahb);

            // PC6
            let _ = gpioc
                  .pc6
                  .into_af_push_pull::<2>(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
            let mut stop_button = gpioa
                  .pa0
                  .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
            let uart1_tx = gpioc
                  .pc4
                  .into_af_push_pull::<7>(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
            let uart1_rx = gpioc
                  .pc5
                  .into_af_push_pull::<7>(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);

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
            uart1.enable_interrupt(Event::Idle);
            
            // EXTI0
            let mut syscfg = dp.SYSCFG.constrain(&mut rcc.apb2);
            let mut exti = dp.EXTI;
            syscfg.select_exti_interrupt_source(&stop_button);
            stop_button.trigger_on_edge(&mut exti, Edge::Rising);
            stop_button.enable_interrupt(&mut exti);

            // DMA 1 channel 5
            let dma1 = dp.DMA1.split(&mut rcc.ahb);
            let mut uart1_rx_channel= dma1.ch5;
            
            // DMA buffer 
            let rx_buffer: &'static mut [u8; 64] = singleton!(: [u8; 64] = [0;64])
                  .expect("rx_buffer singleton double-init - should be unreachables");
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

                  // Circ bit enable
                  (*DMA1::ptr()).ch5.cr.modify(|_, w| w.circ().set_bit());
                  // Read the DMA1_CNDTR register
                  last_cndtr = (*DMA1::ptr()).ch5.ndtr.read().ndt().bits();
                  // Prevent compiler reordering  
                  core::sync::atomic::compiler_fence(Release);

                  uart1_rx_channel.enable();
            }

            // TIM3 channel 1
            unsafe {
                  // TIM3 clock enable
                  (*RCC::ptr()).apb1enr.modify(|_, w| w.tim3en().set_bit());
                  // Force the enable to settle before touching CRC's registers
                  let _ = (*RCC::ptr()).apb1enr.read().tim3en().bit();
                  // Set const values
                  (*TIM3::ptr()).psc.write(|w| w.psc().bits(1439)); // set prescaler to 1440
                  (*TIM3::ptr()).arr.write(|w| w.arr().bits(99)); // set resolution to 100 steps
                  // Enable compare preload mode
                  (*TIM3::ptr()).ccer.modify(|_, w| w.cc1e().set_bit());
                  (*TIM3::ptr()).ccmr1_output().modify(|_, w| {
                        w.oc1m().pwm_mode1();
                        w.oc1pe().enabled()
                  });
                  // Set duty cycle
                  (*TIM3::ptr()).ccr1().write(|w| w.ccr().bits(0));
                  // Configure and enable
                  (*TIM3::ptr()).cr1.write(|w| { 
                        w.arpe().enabled();
                        w.cms().edge_aligned();
                        w.dir().up();
                        w.cen().enabled()
                  });
            }
            
            // CRC configuration
            unsafe {
                  // CRC clock enable
                  (*RCC::ptr()).ahbenr.modify(|_, w| w.crcen().set_bit());
                  // Force the enable to settle before touching CRC's registers
                  let _ = (*stm32f3xx_hal::pac::RCC::ptr()).ahbenr.read(); 
                  // Set the inital CRC's value and polynomial
                  (*CRC::ptr()).init.write(|w| w.init().bits(0xFFFF));
                  (*CRC::ptr()).pol.write(|w| w.pol().bits(0x1021));
                  // CCITT-FALSE
                  (*CRC::ptr()).cr.modify(|_, w| {
                        w.polysize().polysize16();
                        w.rev_in().normal();
                        w.rev_out().normal()
                  });
            }

            (
                  Shared {uart1, pwm_duty: 0, pwm_change_flag: false, tim3_ch1_enabled: true}, 
                  Local {rx_buffer_addr, rx_buffer_length, last_cndtr, cmd_index: 0, cmd: [0u8; 32]}
            )
      }
      
      #[idle(shared = [pwm_duty, pwm_change_flag])]
      fn idle (mut cx: idle::Context) -> ! {
            loop {
                  // Check if the pwm_duty value was changed since the last read 
                  let changed = cx.shared.pwm_change_flag.lock(|flag| {
                        let was_set = *flag;
                        *flag = false;
                        was_set
                  });
                  if changed {
                        cx.shared.pwm_duty.lock(|pwm_duty| {
                              println!("applying duty: {}", *pwm_duty);
                              set_duty(*pwm_duty);
                        })
                  }
                  // Sleep mode
                  wfi();
            }
      }

      #[task(binds = USART1_EXTI25, priority = 2, shared = [uart1], local = [last_cndtr, rx_buffer_length, rx_buffer_addr, cmd_index, cmd])]
      fn uart_rx (mut cx: uart_rx::Context) {

            // Clear USART1_IDLE interrupt 
            unsafe { (*USART1::ptr()).icr.write(|w| w.idlecf().set_bit()) };

            // Declare the locals
            let last_cndtr = cx.local.last_cndtr;
            let rx_buffer_length = *cx.local.rx_buffer_length;
            let rx_buffer_addr = *cx.local.rx_buffer_addr;
            let cmd_index = cx.local.cmd_index;
            let cmd = cx.local.cmd;

            // Unsafe to access the DMA1_CNDRT register 
            let current_cndtr = unsafe { (*DMA1::ptr()).ch5.ndtr.read().ndt().bits() };
            // Compute how many bytes income
            let delta = (*last_cndtr + rx_buffer_length - current_cndtr) % rx_buffer_length;
            for i in 0..delta {
                  // Calculate the physical index in the buffer
                  let rx_index = (rx_buffer_length - *last_cndtr + i) % rx_buffer_length;
                  
                  // Perform a volatile read so the compiler doesn't cache stale memory
                  let byte = unsafe { 
                        let rx_buffer_ptr = (rx_buffer_addr as *const u8).add(rx_index as usize);
                        core::ptr::read_volatile(rx_buffer_ptr)
                  };
                  
                  // Wtire byte to the extra buffer index
                  cmd[*cmd_index] = byte;
                  *cmd_index += 1;

                  // Prevent overflow 
                  if *cmd_index == 32 {
                        *cmd_index = 0;
                        *cmd = [0u8; 32];
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str(CommandError::BufferOverflow.message()); });
                  }

                  // Start parsing and clear extra buffer by sending \n
                  else if byte == 13 {
                        // Spawn error handler
                        if let Err(_) = uart_parser::spawn(*cmd, *cmd_index) {
                              cx.shared.uart1.lock(|uart1| {
                                    let _ = uart1.write_str(CommandError::CommandBusy.message());
                              });
                        }
                        *cmd_index = 0;
                        *cmd = [0u8; 32];
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_char('\n'); });
                  }

            }
            *last_cndtr = current_cndtr;
      }

      #[task(binds = EXTI0, priority = 3, shared = [tim3_ch1_enabled])]
      fn stop_button (mut cx: stop_button::Context) {
            // stop PWM
            (cx.shared.tim3_ch1_enabled).lock(|enabled| {
                  timer_state(false);
                  *enabled = false;
            });

            // unsafe to clear EXTI0 interrupt
            unsafe { (*EXTI::ptr()).pr1.write(|w| w.pr0().set_bit()); }
      }

      #[task(priority = 1, shared = [uart1, pwm_duty, pwm_change_flag, tim3_ch1_enabled])]
      async fn uart_parser(mut cx: uart_parser::Context, cmd_buf: [u8; 32], length: usize) {
            let cmd_body = &cmd_buf[..length];
            match parse_command(cmd_body) {
                  ParsedCommand::SetPwm(duty) => {
                        if cx.shared.tim3_ch1_enabled.lock(|enabled| *enabled) {
                              apply_pwm_duty(
                                    cx.shared.pwm_duty, 
                                    cx.shared.pwm_change_flag, 
                                    duty,
                              );  
                              cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str("PWM set\r\n"); });
                        }
                        else {
                              cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str(CommandError::PwmStopped.message()); });
                        }
                  }
                  ParsedCommand::Stop => {
                        unsafe { (*EXTI::ptr()).swier1.write(|w| w.swier0().set_bit()); }
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str("PWM stopped\r\n"); });
                  }
                  ParsedCommand::Resume => {
                        (cx.shared.tim3_ch1_enabled).lock(|enabled| {
                              timer_state(true);
                              *enabled = true;
                        });
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str("PWM resumed\r\n"); });
                  }
                  ParsedCommand::DropArg => {
                        apply_pwm_duty(
                              cx.shared.pwm_duty, 
                              cx.shared.pwm_change_flag, 
                              0,
                        );
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str("Argument was dropped to 0\r\n"); });
                  }
                  ParsedCommand::Status => {
                        let duty = cx.shared.pwm_duty.lock(|duty| *duty);
                        let state = cx.shared.tim3_ch1_enabled.lock(|state| *state);

                        // Store status metadata
                        let mut status_handler: String<96> = String::new();
                        if let Err(_) = write!(
                              status_handler, 
                              "# - - - - - - -\r\n| PWM works on: {}%\r\n| PWM state is: {}\r\n", 
                              duty, 
                              if state { "RUNNING" } else { "STOPPED" } 
                        ) {
                              cx.shared.uart1.lock(|uart1| {
                                    let _ = uart1.write_str("Error: status_handler buffer is too small to show this message!\r\n");
                              });
                              return;
                        }
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str(&status_handler); });
                  }
                  ParsedCommand::Error(error) => {
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str(error.message()); });
                  }
            }
      }
}