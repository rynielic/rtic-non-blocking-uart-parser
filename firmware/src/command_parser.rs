#[cfg(not(test))]
use stm32f3xx_hal::pac::CRC;

#[cfg(test)]
extern crate std;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandError {
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
    pub fn message (&self) -> &'static str {
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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParsedCommand {
    SetPwm(u8),
    Stop,
    Resume,
    DropArg,
    Status,
    Error(CommandError),
}

pub fn parse_command (cmd_body: &[u8]) -> ParsedCommand {
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

#[allow(dead_code)]
pub fn calculate_crc_sw(data: &[u8]) -> u16 {
    let polynomial = 0x1021;
    let mut crc = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ polynomial;
            }
            else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(not(test))]
fn calculate_crc(cmd_slice: &[u8]) -> u16 {
    unsafe {
        // Reset CRC
        (*CRC::ptr()).cr.modify(|_, w| w.reset().set_bit());
        // Iterate over each byte from the command
        for &byte in cmd_slice {
            // Write the byte into CRC data register
          (*CRC::ptr()).dr8().write(|w| w.dr8().bits(byte));
        }
        // Read the checksum from CRC data register
        ((*CRC::ptr()).dr().read().bits() & 0xFFFF) as u16
    }
}
#[cfg(test)]
fn calculate_crc(cmd_slice: &[u8]) -> u16 {
    calculate_crc_sw(cmd_slice)
}

fn crc_match (cmd_body: &[u8]) -> Result<&[u8], CommandError> {
    // search for '*'
    let slice_index = match cmd_body.iter().rposition(|ch| *ch == b'*') {
        Some(ind) => ind,
        _ => return Err(CommandError::CrcNotFound),
    };

    if slice_index == 0 {
        return Err(CommandError::CrcNotFound);
    }
    // Contains the command itself
    let cmd_slice= &cmd_body[..slice_index];
    // Contains raw bytes of the checksum
    let crc_slice = &cmd_body[slice_index + 1..cmd_body.len() - 1];

    // Formated checsum
    let expected_checksum = core::str::from_utf8(crc_slice)
        .ok()
        .and_then(|s| u16::from_str_radix(s, 16).ok());

    let calulated_checksum = calculate_crc(cmd_slice);
    if expected_checksum != Some(calulated_checksum) {
        return Err(CommandError::CrcMismatch)
    }
    Ok(cmd_slice)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calculate_crc_known_vector() {
        assert_eq!(calculate_crc(b"123456789"), 0x29B1)
    }

    #[test]
    fn valid_commands() {
        let crc = calculate_crc_sw(b"STOP");
        let s = std::format!("STOP*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Stop);

        let crc = calculate_crc_sw(b"RESUME");
        let s = std::format!("RESUME*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Resume);

        let crc = calculate_crc_sw(b"DROP_ARG");
        let s = std::format!("DROP_ARG*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::DropArg);

        let crc = calculate_crc_sw(b"STATUS");
        let s = std::format!("STATUS*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Status);
    }

    #[test]
    fn valid_pwm_argument() {
        let crc = calculate_crc_sw(b"SET_PWM(0)");
        let s = std::format!("SET_PWM(0)*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::SetPwm(0));

        let crc = calculate_crc_sw(b"SET_PWM(50)");
        let s = std::format!("SET_PWM(50)*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::SetPwm(50));

        let crc = calculate_crc_sw(b"SET_PWM(100)");
        let s = std::format!("SET_PWM(100)*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::SetPwm(100));
    }

    #[test]
    fn test_invalid_pwm_arguments() {
        // Empty argument: SET_PWM()*...
        let crc = calculate_crc_sw(b"SET_PWM()");
        let s = std::format!("SET_PWM()*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Error(CommandError::EmptyArgument));

        // Value > 100, e.g. 101
        let crc = calculate_crc_sw(b"SET_PWM(101)");
        let s = std::format!("SET_PWM(101)*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Error(CommandError::InvalidArgument));

        // Non-digits, e.g. abc
        let crc = calculate_crc_sw(b"SET_PWM(abc)");
        let s = std::format!("SET_PWM(abc)*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Error(CommandError::InvalidArgument));

        // Too long (> 3 digits), e.g. 1000
        let crc = calculate_crc_sw(b"SET_PWM(1000)");
        let s = std::format!("SET_PWM(1000)*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Error(CommandError::ArgumentTooLong));
    }

    #[test]
    fn test_crc_errors() {
        // Missing '*' slicer
        assert_eq!(parse_command(b"STOPAF2E\r"), ParsedCommand::Error(CommandError::CrcNotFound));

        // Wrong CRC value
        assert_eq!(parse_command(b"STOP*0000\r"), ParsedCommand::Error(CommandError::CrcMismatch));

        // Corrupted hex string
        assert_eq!(parse_command(b"STOP*ZZZZ\r"), ParsedCommand::Error(CommandError::CrcMismatch));

        // Malformed inputs without crash
        assert_eq!(parse_command(b"*\r"), ParsedCommand::Error(CommandError::CrcNotFound));
        assert_eq!(parse_command(b"*"), ParsedCommand::Error(CommandError::CrcNotFound));
        assert_eq!(parse_command(b""), ParsedCommand::Error(CommandError::CrcNotFound));
    }

    #[test]
    fn test_unknown_command() {
        let crc = calculate_crc_sw(b"UNKNOWN");
        let s = std::format!("UNKNOWN*{:04X}\r", crc);
        assert_eq!(parse_command(s.as_bytes()), ParsedCommand::Error(CommandError::UnknownCommand));
    }
}