use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
use std::sync::Arc;
use std::thread;
use std::time::Duration;
use std::io::{self, Write, Read};
use serialport::SerialPortInfo;

const BAUD_RATE: u32 = 115_200;
static RELOAD_ERROR_FLAG: AtomicBool = AtomicBool::new(false);

// Checksum computation
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

// Select available port interactively
fn pick_port() -> SerialPortInfo {
    loop {
        match serialport::available_ports() {
            Ok(ports) if ports.is_empty() => {
                eprintln!("No ports found. Press enter to retry.");
                let _ = io::stdin().read_line(&mut String::new());
            }
            Ok(ports) => {
                println!("\nAvailable ports:");

                // Iterate over available ports and print them
                for (count, port) in ports.iter().enumerate() {
                    println!("[{}]: {}", count, port.port_name);
                }

                if RELOAD_ERROR_FLAG.load(Relaxed) {
                    println!("\x1b[31mA problem occured due to error above\x1b[0m")
                }
                // Buffer for a port selection
                let mut port_input = String::new(); 
                loop {
                    port_input.clear();

                    // Select a port from keybord input
                    print!("Choose your port: ");
                    io::stdout().flush().unwrap();
                    io::stdin().read_line(&mut port_input).unwrap();



                    // Parse keyboard input
                    match port_input.trim().parse::<usize>() {
                        // Return the selected port
                        Ok(index) if index < ports.len() => {
                            RELOAD_ERROR_FLAG.store(false, Relaxed);
                            return ports[index].clone();
                        }
                        // Print and retry
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
                eprintln!("Error: Failed to find ports: {}", e );
                println!("Press Enter to retry");
                let _ = io::stdin().read_line(&mut String::new());
            }
        }
    }
}

fn main() {
    println!("\r- - - - - - - - - - - - -\r\nHelp: press Ctrl + C to quit or Ctrl + D to flush input buffer\r\n- - - - - - - - - - - - -\r\n");
    loop {
        // Declare the found port
        let port = pick_port().port_name;
        println!("Port {} selected", port);

        // Open serial interface 
        let mut serial = match serialport::new(&port, BAUD_RATE)
            .timeout(Duration::from_millis(10))
            .open() 
        {
            Ok(serial) => serial,
            Err(e) => {
                eprintln!("Error: Failed to open port {}: {}", port, e);
                RELOAD_ERROR_FLAG.store(true, Relaxed);
                continue;
            }
        };
        
        // Clone serial for the READ thread
        let mut rx_port = match serial.try_clone() {
            Ok(rx_port) => rx_port,
            Err(e) => {
                eprintln!("Error: Failed to clone port {}: {}", port, e);
                RELOAD_ERROR_FLAG.store(true, Relaxed);
                continue;
            }
        };

        // Cancellation flag for reader with shared access (one flag two owners)
        let stop_reader = Arc::new(AtomicBool::new(false));
        let stop_reader_clone = Arc::clone(&stop_reader);

        // READ thread
        let reader_handle = thread::spawn(move || {
            let mut rx_line: [u8; 32] = [0; 32];
            // Check the falg on every iteration:
            while !stop_reader_clone.load(Relaxed) {
                match rx_port.read(rx_line.as_mut_slice()) {
                    Ok(t) => {
                        let _ = io::stdout().write_all(&rx_line[..t]);
                        let _ = io::stdout().flush();
                    }
                    // Timeout that allows checking the stop flag every 10ms
                    Err(ref e) if e.kind() == io::ErrorKind::TimedOut => (),
                    Err(e) => {
                        eprintln!("Error: Failed to read: {}", e);
                        break;
                    }
                }
            }
        });

        // MAIN(write) thread
        let mut reconnect = false;
        println!("Type a command and press Enter to send the line");
        
        // Line input
        for line in io::stdin().lines() {
            let line = match line {
                Ok(line) => line.to_ascii_uppercase(), // make input less redundant
                Err(_) => break, // stdin closed
            };

            // Compute the checksum and send it with the input line for next validation
            let checksum = crc16(line.as_bytes());
            let formatted_line = format!("{}*{:04X}\r", line, checksum);
            
            // Send the formatted line
            match serial.write_all(formatted_line.as_bytes()) {
                Ok(_) => {
                    if let Err(e) = serial.flush() {
                        eprintln!("Error: Failed to flush: {}", e);
                        RELOAD_ERROR_FLAG.store(true, Relaxed);
                    }
                }
                Err(e) => {
                    eprintln!("Error: Failed to write: {}. Reconnecting...", e);
                    RELOAD_ERROR_FLAG.store(true, Relaxed);
                    reconnect = true;   
                    break;
                }
            }
  
        }

        // Stop thread before restarting or exiting  
        stop_reader.store(true, Relaxed);
        let _ = reader_handle.join(); // wait for it to terminate

        if !reconnect {
            println!("Input closed, exiting.");
            break;
        }
    }
}