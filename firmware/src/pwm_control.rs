use stm32f3xx_hal::pac::TIM3;

pub fn apply_pwm_duty (
            mut pwm_duty: impl rtic::Mutex<T = u8>,
            mut pwm_change_flag: impl rtic::Mutex<T = bool>,
            value: u8,
      ) {
            pwm_duty.lock(|duty| *duty = value);   
            pwm_change_flag.lock(|flag| *flag = true);
      }

pub fn timer_state(state: bool) {
            unsafe {
                  if state {
                        // Restore PWM Mode 1
                        (*TIM3::ptr()).ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1());  
                        // Restart the counter
                        (*TIM3::ptr()).cr1.modify(|_, w| w.cen().enabled());
                        return;
                  }
                  // Stop the counter (saves power)
                  (*TIM3::ptr()).cr1.modify(|_, w| w.cen().disabled());
                  // Actively force the output pin LOW (0V)
                  (*TIM3::ptr()).ccmr1_output().modify(|_, w| w.oc1m().force_inactive());
                  // Reset counter to 0 so next start begins at the phase start
                  (*TIM3::ptr()).cnt.write(|w| w.bits(0));
            }
      }

pub fn set_duty(duty: u8) {
            unsafe {
                  if duty == 0 {
                        // Change duty
                        (*TIM3::ptr()).ccr1().write(|w| w.ccr().bits(duty as u16));
                        // Stop the counter (saves power)
                        timer_state(false);
                        return;
                  }
                  // Change duty
                  (*TIM3::ptr()).ccr1().write(|w| w.ccr().bits(duty as u16));
                  // Force immediate transfer from preload to active shadow register
                  (*TIM3::ptr()).egr.write(|w| w.ug().set_bit());

                  timer_state(true);
            }
      }