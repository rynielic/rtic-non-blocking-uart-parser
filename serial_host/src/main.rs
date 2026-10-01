use std::thread;
use std::time::Duration;
use std::io::{self, Write, Read};

const BAUD_RATE: u32 = 115_200;

pub fn crc16 (cmd: &[u8]) -> u16 {
    let polynomial = 0x1021;
    let mut crc = 0xFFFF;
    for &byte in cmd {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ polynomial;
            }
            else {
                crc <<= 1;
            }
        }
    }
    crc
}

fn main() {
    let port = &serialport::available_ports()
        .unwrap_or_else(|e| {
            eprintln!("Failed to find port: {}", e);
            std::process::exit(1);
        })[0].port_name;
    
    let mut serial = serialport::new(port, BAUD_RATE)
        .timeout(Duration::from_millis(10))
        .open()
        .unwrap_or_else(|e| {
            eprintln!("Failed to open port {}: {}", port, e);
            std::process::exit(1);
        });
    
    let mut rx_port = serial.try_clone().expect("Failed to clone the port");

    thread::spawn(move || {
        let mut rx_line: [u8; 32] = [0; 32];

        loop {
            match rx_port.read(rx_line.as_mut_slice()) {
                Ok(t) => {
                    io::stdout().write_all(&rx_line[..t]).unwrap();
                    io::stdout().flush().unwrap();
                }
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
                Err(e) => {
                    eprint!("Failed to read: {:?}", e);
                    break;
                }
            }
        }
    });

    for line in io::stdin().lines() {
        let line = (line.unwrap()).to_ascii_uppercase();

        let checksum = crc16(line.as_bytes());
        let formated_line = format!("{}*{:04X}\r", line, checksum);
        
        match serial.write_all(formated_line.as_bytes()) {
            Ok(_) => {
                if let Err(e) = serial.flush() {
                    eprintln!("Failed to flush: {}", e)
                }
            }
            Err(e) => eprintln!("Failed to write: {}", e)
        }        
    }

}