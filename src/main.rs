use std::io;
use std::process::ExitCode;
use std::thread::sleep;
use std::time::Duration;
use log::{error, info};
use crate::zedmd_comm::connect;

mod zedmd_comm;


fn main() -> ExitCode {
    // Initialize the logger with color support and debug level
    env_logger::Builder::new()
        .format_timestamp(None)
        .format_module_path(false)
        .format_target(false)
        .filter_level(log::LevelFilter::Debug)
        .init();

    // Test the connection to the ZeDMD device
    match test_connect() {
        Ok(_) => ExitCode::SUCCESS,
        Err(e) => {
            error!("Error: {}", e);
            ExitCode::FAILURE
        }
    }
}

fn test_connect() -> io::Result<()> {
    let mut comm = connect()?;
    info!("Connected to ZeDMD device: {:?}", comm);
    // Send keepalive command
    comm.run()?;
    
    sleep(Duration::from_secs(10));
    
    comm.stop()?;
    
    // // Perform a LED test
    // // comm.led_test()?;
    // // info!("LED test performed successfully");
    // // sleep for a while to allow the LED test to complete
    // sleep(std::time::Duration::from_secs(10));
    // Reset the device
    //comm.reset()?;
    Ok(())
}
