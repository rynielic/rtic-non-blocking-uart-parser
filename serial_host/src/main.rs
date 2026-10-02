use std::thread;
use std::time::Duration;
use std::io::{self, Write, Read};

use serialport::SerialPortInfo;

const BAUD_RATE: u32 = 115_200;

fn crc16 (cmd: &[u8]) -> u16 {
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

fn pick_port() -> SerialPortInfo {
    loop {
        match serialport::available_ports() {
            Ok(ports) => {
                for (count, port) in ports.iter().enumerate() {
                    println!("[{}]: {}", count, port.port_name);
                }
                let mut port_index_buffer = String::new(); 

                loop {
                    port_index_buffer.clear();
                    println!("Choose your port: ");
                    io::stdin().read_line(&mut port_index_buffer).unwrap();

                    match port_index_buffer.trim().parse::<usize>() {
                        Ok(index) if index < ports.len() => return ports[index].clone(),
                        Ok(_) => {
                            println!("Error: index out of range!");
                        }
                        Err(_) => {
                            eprintln!("Error: please enter a number!");
                        }
                    };

                    
                }
            }
            Err(e) => {
                eprintln!("Failed to find ports: {}", e );
                println!("Press enter to reload");
                let _ = io::stdin().read_line(&mut String::new());
            }
        }
    }


}

fn main() {
    loop {
        let port = pick_port().port_name;
        println!("Port {} selected", port);
        let mut serial = match serialport::new(&port, BAUD_RATE)
            .timeout(Duration::from_millis(10))
            .open() {
                Ok(serial) => serial,
                Err(e) => {
                    eprintln!("Failed to open port {}: {}", port, e);
                    continue;
                }
            };

        let mut rx_port = match serial.try_clone() {
            Ok(rx_port) => rx_port,
            Err(e) => {
                eprintln!("Failed to clone port {}: {}", port, e);
                continue;
            }
        };

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
}