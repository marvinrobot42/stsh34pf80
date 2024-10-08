# STHS34PF80 &emsp; 
[![crates.io](https://img.shields.io/crates/v/sths34pf80)](https://crates.io/crates/sths34pf80)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](https://github.com/marvinrobot42/sths34pf80)
[![Documentation](https://docs.rs/sths34pf80/badge.svg)](https://docs.rs/sths34pf80)

## A Rust crate for ST Microelectronics STHS34PF80 presence and motion detection sensor 

<https://github.com/marvinrobot42/stsh34pf80.git>

[STHS34PF80]: https://www.st.com/en/mems-and-sensors/sths34pf80.html

The STHS34PF80 sensor detects infrared presence, meaning a warm "body" stationary object.  It also does ambient 
shock temperature detection (excessive heat change) and motion detection.

Many configurable settings for the detection thresholds, data output rates, interrupt pin, low pass filters,
sensitivity. 

### Features

- updated to use embedded-hal version 1.0.x
- async support included as a feature
- designed for embedded use (ESP32-C3, -C6 and -S3) and Raspberry Pi 
- configurable interrupt pin
- data ready status based presence, motion and ambient shock measurement reads 
- low pass filter configuration if needed
- hysteresis getters and setters
- sensitivity getters and setters
- gain getters and setters
- data output rate, average trim getters and setters
- an easy to use Measurements struct
- an easy to use initialize function
- easy to use get_measurement_blocking and get_measurement_timeout (with timeout parameter)
- no_std embedded compatible

  

#### Notes

This driver is loosly based on Sparkfun Arduino STHS34PF80 driver.  A Sparkfun STHS34PF80
https://www.sparkfun.com/products/22494 sensor board was used for this driver development.

In version 0.1.9 the error enum was changed to a simpler embedded-hal 1.0 I2C style.  Hopefully
it does not break your code (consider using anyerror create).

Unfortunately the github repository name is not spelled correctly but now stuck with it.
The proper device and crate name is sths34pf80 while the mis-spelled repo name is stsh34pf80.

### Recent version history
  - 0.1.11 Fixed more typos
  - 0.1.10 Fixed typo
  - 0.1.9  Added async support
  - 0.1.8  Updated README.md
  - 0.1.7  fixed the example below, added Raspberry Pi example
  - 0.1.5  github repo name fixed in links
  - 0.1.4  Documentation improvements.
  - 0.1.0  Initial release.


## Usage
----

Add the dependency to `Cargo.toml`.

~~~~toml
[dependencies.sths34pf80]
version = "0.1"
~~~~

or for async
~~~~toml
[dependencies.sths34pf80]
version = "0.1", features = ["async"]
~~~~

Create a hardward specific I²C driver interface and delay function
Create an Sths34pf80 struct from the I²C interface and a delay function.
Configure interrupt pin properties if required.  
Initialize Sths34pf80  (initialize() fn sets "standard" trims and ODR paramaters).
Set presence, motion or ambient shock threshold only if required for more/less detection range
Read the STHS34PF80 status and check if new data is ready, then get_func_status and match
the returned FUNC_STATUS enum to see which of the three values changed, then get those values.  
 


### Simple Example

A more complete example is in the repository examples path.

For async see async-embassy folder in examples folder of repository.
~~~~rust


use sths34pf80::{Sths34pf80};

...


fn main() -> Result<()> {

  ...

  let peripherals = Peripherals::take().unwrap();
  let pins = peripherals.pins;
  let sda = pins.gpio0;
  let scl = pins.gpio1;
  let i2c = peripherals.i2c0;
  let config = I2cConfig::new().baudrate(100.kHz().into());
  let i2c_dev = I2cDriver::new(i2c, sda, scl, &config)?;

  let mut sths34pf80 = Sths34pf80::new(i2c_dev, Ets{});  // Ets is ESP32 IDF delay function

  sths34pf80.initialize().unwrap();  

  loop {

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

    FreeRtos::delay_ms(10000);
  }

}
    
~~~~


### License
----

You are free to copy, modify, and distribute this application with attribution under the terms of either

 * Apache License, Version 2.0
   ([LICENSE-Apache-2.0](./LICENSE-Apache-2.0) or <https://opensource.org/licenses/Apache-2.0>)
 * MIT license
   ([LICENSE-MIT](./LICENSE-MIT) or <https://opensource.org/licenses/MIT>)

at your option.

This project is not affiliated with nor endorsed in any way by STMicroelectronics or Sparkfun.
