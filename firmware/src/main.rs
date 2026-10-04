#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device = stm32f3xx_hal::pac, dispatchers = [SPI1])]
mod app {
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
            }, gpio::Edge, pac::USART1, prelude::*, pwm::tim1, serial::{Event, Serial},
      };

      type Uart1TxPin = stm32f3xx_hal::gpio::Pin<stm32f3xx_hal::gpio::Gpioc, stm32f3xx_hal::gpio::U<4>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>;
      type Uart1RxPin = stm32f3xx_hal::gpio::Pin<stm32f3xx_hal::gpio::Gpioc, stm32f3xx_hal::gpio::U<5>, stm32f3xx_hal::gpio::Alternate<stm32f3xx_hal::gpio::PushPull, 7>>;
      type Tim3Channel1 = stm32f3xx_hal::pwm::PwmChannel<stm32f3xx_hal::pwm::Tim1Ch1, stm32f3xx_hal::pwm::WithPins>;

      enum CommandError {
            EmptyArgument,
            InvalidArgument,
            ArgumentTooLong,
            UnknownCommand,
            PwmStopped,
            BufferOverflow,
            CommandBusy,
            CrcMismatch,
            CrcNotFound,
      }
      impl CommandError {
            fn message (&self) -> &'static str {
                  match self {
                        CommandError::EmptyArgument => "Error: Argument can not be empty!\r\n",
                        CommandError::InvalidArgument => "Error: Wrong argument!\r\n",
                        CommandError::ArgumentTooLong => "Error: Argument is too big!\r\n",
                        CommandError::UnknownCommand => "Error: Unknown command!\r\n",
                        CommandError::PwmStopped => "Error: Unable to set value - PWM is stopped. To continue enter RESUME\r\n",
                        CommandError::BufferOverflow => "Error: Command too long, buffer cleared!\r\n",
                        CommandError::CommandBusy => "Error: Busy, command dropped\r\n",
                        CommandError::CrcMismatch => "Error: CRC mismatch!\r\n",
                        CommandError::CrcNotFound => "Error: Corrupted data!\r\n",
                  }
            }
      }

      enum ParsedCommand {
            SetPwm(u8),
            Stop,
            Resume,
            DropArg,
            Status,
            Error(CommandError),
      }

      fn parse_command (cmd_body: &[u8]) -> ParsedCommand {
            let cmd = match crc_match(cmd_body) {
                  Ok(cmd) => cmd,
                  Err(err) => {
                        return ParsedCommand::Error(err);
                  }
            };

            if let Some(rest) = cmd.strip_prefix(b"SET_PWM(") {
                  return match rest.iter().position(|br| *br == b')') {
                        Some(finish) if finish == rest.len() - 1 => parse_pwm_argument(&rest[..finish]),
                        _ => ParsedCommand::Error(CommandError::UnknownCommand)
                  };
            }
            match cmd {
                  b"STOP" => ParsedCommand::Stop,
                  b"RESUME" => ParsedCommand::Resume,
                  b"DROP_ARG" => ParsedCommand::DropArg,
                  b"STATUS" => ParsedCommand::Status,
                  _ => ParsedCommand::Error(CommandError::UnknownCommand),        
            }
      }

      fn parse_pwm_argument(argument: &[u8]) -> ParsedCommand {
            match argument {
                  [] => ParsedCommand::Error(CommandError::EmptyArgument),
                  [a] if a.is_ascii_digit() => ParsedCommand::SetPwm(a - b'0'),
                  [a, b] if a.is_ascii_digit() && b.is_ascii_digit() => ParsedCommand::SetPwm((a - b'0')*10 + b - b'0'),
                  [b'1', b'0', b'0'] => ParsedCommand::SetPwm(100),
                  [_] | [_,_] | [_,_,_] => ParsedCommand::Error(CommandError::InvalidArgument), 
                  _ => ParsedCommand::Error(CommandError::ArgumentTooLong), 
            }
      }

      fn apply_pwm_duty (
            mut pwm_duty: impl rtic::Mutex<T = u8>,
            mut pwm_change_flag: impl rtic::Mutex<T = bool>,
            value: u8,
      ) {
            pwm_duty.lock(|duty| *duty = value);   
            pwm_change_flag.lock(|flag| *flag = true);
      }
      
      fn crc_match (cmd_body: &[u8]) -> Result<&[u8], CommandError> {
            // search for '*'
            let slice_index = match cmd_body.iter().rposition(|ch| *ch == b'*') {
                  Some(ind) => ind,
                  _ => return Err(CommandError::CrcNotFound),
            };

            // Contains the command itself
            let cmd_slice= &cmd_body[..slice_index];
            // Contains raw bytes of the checksum
            let crc_slice = &cmd_body[slice_index + 1..cmd_body.len() - 1];
            // Formated checsum
            let expected_checksum = core::str::from_utf8(crc_slice)
                  .ok()
                  .and_then(|s| u16::from_str_radix(s, 16).ok());

            unsafe {
                  // Reset CRC
                  (*stm32f3xx_hal::pac::CRC::ptr()).cr.modify(|_, w| w.reset().set_bit());
                  // Iterate over each byte from the command
                  for &byte in cmd_slice {
                        // Write the byte into CRC data register
                        (*stm32f3xx_hal::pac::CRC::ptr()).dr8().write(|w| w.dr8().bits(byte));
                  }
                  // Read the checksum from CRC data register
                  let calculated_checksum = ((*stm32f3xx_hal::pac::CRC::ptr()).dr().read().bits() & 0xFFFF) as u16;

                  if expected_checksum != Some(calculated_checksum) {
                        return Err(CommandError::CrcMismatch)
                  }
                  Ok(cmd_slice)
            }
            
      }

      #[shared]
      struct Shared {
            uart1: Serial<USART1, (Uart1TxPin, Uart1RxPin)>,
            pwm_duty: u8,
            pwm_change_flag: bool,
            tim1_ch1: Tim3Channel1,
            tim1_ch1_enabled: bool,
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
            let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

            // Pins
            let red_led = gpioe
                  .pe9
                  .into_af_push_pull::<2>(&mut gpioe.moder, &mut gpioe.otyper, &mut gpioe.afrh);
            let mut stop_button = gpioa
                  .pa0
                  .into_pull_down_input(&mut gpioa.moder, &mut gpioa.pupdr);
            let uart1_tx = gpioc
                  .pc4
                  .into_af_push_pull::<7>(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
            let uart1_rx = gpioc
                  .pc5
                  .into_af_push_pull::<7>(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);

            // TIM1 channel 1
            let (tim1_ch1_no_pins, ..) = tim1(dp.TIM1, 100, 500.Hz(), &clock);
            let mut tim1_ch1 = tim1_ch1_no_pins.output_to_pe9(red_led);
            tim1_ch1.enable();

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
                  (*stm32f3xx_hal::pac::DMA1::ptr()).ch5.cr.modify(|_, w| w.circ().set_bit());
                  // Read the DMA1_CNDTR register
                  last_cndtr = (*stm32f3xx_hal::pac::DMA1::ptr()).ch5.ndtr.read().ndt().bits();
                  // Prevent compiler reordering  
                  core::sync::atomic::compiler_fence(Release);

                  uart1_rx_channel.enable();
            }
            
            // CRC configuration
            unsafe {
                  // CRC clock enable
                  (*stm32f3xx_hal::pac::RCC::ptr()).ahbenr.modify(|_, w| w.crcen().set_bit());
                  // Rorce the enable to settle before touching CRC's registers
                  let _ = (*stm32f3xx_hal::pac::RCC::ptr()).ahbenr.read(); 

                  // Configure CRC's registers
                  (*stm32f3xx_hal::pac::CRC::ptr()).cr.modify(|_, w| w.polysize().polysize16());
                  (*stm32f3xx_hal::pac::CRC::ptr()).init.write(|w| w.init().bits(0xFFFF));
                  (*stm32f3xx_hal::pac::CRC::ptr()).pol.write(|w| w.pol().bits(0x1021));
            }

            (
                  Shared {uart1, pwm_duty: 0, pwm_change_flag: false, tim1_ch1, tim1_ch1_enabled: true}, 
                  Local {rx_buffer_addr, rx_buffer_length, last_cndtr, cmd_index: 0, cmd: [0u8; 32]}
            )
      }
      
      #[idle(shared = [pwm_duty, pwm_change_flag, tim1_ch1])]
      fn idle (mut cx: idle::Context) -> ! {
            loop {
                  // Check if the pwm_duty value was changed since the last read 
                  let changed = cx.shared.pwm_change_flag.lock(|flag| {
                        let was_set = *flag;
                        *flag = false;
                        was_set
                  });
                  if changed {
                        cx.shared.tim1_ch1.lock(|tim1_ch1| {
                              cx.shared.pwm_duty.lock(|pwm_duty| {
                                    tim1_ch1.set_duty(*pwm_duty as u16);
                              })
                        });
                  }
                  // Sleep mode
                  wfi();
            }
      }

      #[task(binds = USART1_EXTI25, priority = 2, shared = [uart1], local = [last_cndtr, rx_buffer_length, rx_buffer_addr, cmd_index, cmd])]
      fn uart_rx (mut cx: uart_rx::Context) {

            // Clear USART1_IDLE interrupt 
            unsafe { (*stm32f3xx_hal::pac::USART1::ptr()).icr.write(|w| w.idlecf().set_bit()) };

            // Declare the locals
            let last_cndtr = cx.local.last_cndtr;
            let rx_buffer_length = *cx.local.rx_buffer_length;
            let rx_buffer_addr = *cx.local.rx_buffer_addr;
            let cmd_index = cx.local.cmd_index;
            let cmd = cx.local.cmd;

            // Unsafe to access the DMA1_CNDRT register 
            let current_cndtr = unsafe { (*stm32f3xx_hal::pac::DMA1::ptr()).ch5.ndtr.read().ndt().bits() };
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

      #[task(binds = EXTI0, priority = 3, shared = [tim1_ch1, tim1_ch1_enabled])]
      fn stop_button (cx: stop_button::Context) {
            // stop PWM
            (cx.shared.tim1_ch1, cx.shared.tim1_ch1_enabled).lock(|tim1_ch1, enabled| {
                  tim1_ch1.disable();
                  *enabled = false;
            });

            // unsafe to clear EXTI0 interrupt
            unsafe { (*stm32f3xx_hal::pac::EXTI::ptr()).pr1.write(|w| w.pr0().set_bit()); }
      }

      #[task(priority = 1, shared = [uart1, pwm_duty, pwm_change_flag, tim1_ch1, tim1_ch1_enabled])]
      async fn uart_parser(mut cx: uart_parser::Context, cmd_buf: [u8; 32], length: usize) {
            let cmd_body = &cmd_buf[..length];
            match parse_command(cmd_body) {
                  ParsedCommand::SetPwm(duty) => {
                        if cx.shared.tim1_ch1_enabled.lock(|enabled| *enabled) {
                              apply_pwm_duty(
                                    cx.shared.pwm_duty, 
                                    cx.shared.pwm_change_flag, 
                                    duty,
                              );  
                              cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str("PWM setted\r\n"); });
                        }
                        else {
                              cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str(CommandError::PwmStopped.message()); });
                        }
                  }
                  ParsedCommand::Stop => {
                        unsafe { (*stm32f3xx_hal::pac::EXTI::ptr()).swier1.write(|w| w.swier0().set_bit()); }
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str("PWM stopped\r\n"); });
                  }
                  ParsedCommand::Resume => {
                        (cx.shared.tim1_ch1, cx.shared.tim1_ch1_enabled).lock(|tim1_ch1, enabled| {
                              tim1_ch1.enable();
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
                        let state = cx.shared.tim1_ch1_enabled.lock(|state| *state);

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
                                    return;
                              });
                        }
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str(&status_handler); });
                  }
                  ParsedCommand::Error(error) => {
                        cx.shared.uart1.lock(|uart1| { let _ = uart1.write_str(error.message()); });
                  }
            }
      }
}