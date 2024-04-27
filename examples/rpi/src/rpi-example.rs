use anyhow::Result;
use linux_embedded_hal::{Delay, I2cdev};

use sths34pf80::{data::{Measurements}, Sths34pf80};

use std::thread;
use std::time::Duration;

use env_logger::Builder;
use log::{LevelFilter, error, info};
use std::io::Write;

fn main() -> Result<()> {

    let mut builder = Builder::from_default_env();

    builder
        .format(|buf, record| writeln!(buf, "{} - {}", record.level(), record.args()))
        .filter(None, LevelFilter::Info)
        .init();


    info!("Hello, linux Rust and ENS160-AQ world!");

    let dev_i2c = I2cdev::new("/dev/i2c-1").unwrap();
    let delayer = Delay {};

    let mut sths34pf80 = Sths34pf80::new(dev_i2c, delayer);
    sths34pf80.initialize().unwrap();


    loop {
        info!("looping");
        thread::sleep(Duration::from_secs(10));
        if let Ok(measurements) = sths34pf80.get_measurements_timeout(10) {
            match measurements.presence_value {
                Some(presence_value) => info!("presence detected, value is {}", presence_value),
                _ => (),
            }
            match measurements.motion_value {
                Some(motion_value) => info!("motion detected, value is {}", motion_value),
                _ => (),
            }
            match measurements.ambient_shock_value {
                Some(abmient_shock_value) => info!("ambient shock temp detected, value is {}", abmient_shock_value),
                _ => (),
            }
        } else {
            info!("timeout with get_measurements_timeout call");
        }

    
    }

}
