#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    gpio::{Io, Level, Output},
    i2c::I2C,
    peripherals::Peripherals,
    prelude::*,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer},
};


use sths34pf80::{data::{InterruptPinConfig, Measurements}, Sths34pf80};

use log::{info, debug};

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[main]
async fn main(_spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let i2c0 = I2C::new_async(
        peripherals.I2C0,
        io.pins.gpio6,   // Sparkfun ESP32-c6 mini, change for your hardware
        io.pins.gpio7,   // Sparkfun ESP32-c6 mini, change for your hardware
        400.kHz(),
        &clocks,
    );

    //esp_println::logger::init_logger_from_env();
    esp_println::logger::init_logger(log::LevelFilter::Debug);
    info!("info logging!");
    debug!("debug logging");

    // Sths34pf80
    let mut sths34pf80 = Sths34pf80::new(i2c0, embassy_time::Delay);
    sths34pf80.initialize().await.unwrap();

 
    info!("     calling get_presence_threshold()");
    let presence_threshold = sths34pf80.get_presence_threshold().await.unwrap();
    info!(" presence threshold is {}", presence_threshold);
  
    

    sths34pf80.set_tmotion_threshold_new(500).await.unwrap();  // default is 200
    let tmotion_threshold = sths34pf80.get_tmotion_threshold().await.unwrap();
    info!(" tmotion threshold is {}",tmotion_threshold);

    /**** more function calls if needed ***************************
    sths34pf80.set_tambient_shock_threshold_new(253).await.unwrap();
    let tamb_shock_threshold = sths34pf80.get_tambient_shock_threshold().await.unwrap();
    info!(" tambient shock threshold is {}", tamb_shock_threshold);

   
    sths34pf80.set_presence_threshold_new(1200).await.unwrap();
    presence_threshold = sths34pf80.get_presence_threshold().await.unwrap();
    info!(" after set_presence_threshold(1200) presence threahold is {}", presence_threshold);

    sths34pf80.set_gain_mode(Gain::GainDefault).await;
    let gain_mode: Gain = sths34pf80.get_gain_mode().await.unwrap();
    info!(" gain mode is {:#?}", gain_mode);

    sths34pf80.set_avg_tambient_num(AverageTrimT1Flag::AVG_T8).await.unwrap();
    let avg_tambient_num: AverageTrimT1Flag = sths34pf80.get_avg_tambient_num().await.unwrap();
    info!(" avg_tambient_num is {:#?}", avg_tambient_num);

    let tmos_sens = sths34pf80.get_tmos_sensitivty().await.unwrap();
    info!("get_tmos_sensivity is : {}", tmos_sens);

    sths34pf80.set_tmos_sensitivity(0.0).await.unwrap();  // 0 -to- +4080
    
    ********/

    // sths34pf80.set_avg_tmos_num(AverageTrimTMOSFlag::AVG_TMOS32).unwrap();

    let int_pin_config = InterruptPinConfig::builder()
        .active_high()
        .open_drain()
        //.on_new_any()
        .on_new_presence()
        .latched_pin()  // testing: the method ien_int_or will clear this bit
        .ien_int_or()
        .build();  // build method just returns this struc's .0 (u8) property
        
    info!("setting to int_pin_config value {:#04x}", int_pin_config);
    if let Ok(new_int_config) = sths34pf80.set_config_interrupt_pin(int_pin_config).await {
        if (new_int_config.get_u8_value() == int_pin_config) {
            info!("interrupt pin config changed as expected to {:#04x}", new_int_config.0);
        } else {
            log::error!("interrupt pin config change problem, expected {:#04x}, got {:#04x}", 
            int_pin_config, new_int_config.get_u8_value());
        }
    }



    let mut led_pin23 = Output::new(io.pins.gpio23, Level::High);
    led_pin23.toggle();
    Timer::after(Duration::from_millis(2000)).await;
    led_pin23.toggle();

    info!("sleeping 5 second to wait for STHS34PF80 to become ready for measurements");
    Timer::after(Duration::from_millis(5000)).await;

    loop {

        // sth34pf80
        if let Ok(measurements) = sths34pf80.get_measurements_timeout(10).await {
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

        led_pin23.toggle();
        Timer::after(Duration::from_millis(2000)).await;
    }

}
