#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::ledc::*;
use esp_hal::timer::timg::TimerGroup;
use l298n_driver::l298n_control::{self, L298n};
use log::info;
use tcrt5000_driver::tcrt5000::{self, Tcrt5000};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 1.2.0

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let tcrt_center_pin = tcrt5000::initialize_tcrt5000(peripherals.GPIO2);
    let tcrt_center = Tcrt5000::new(tcrt_center_pin, true);

    let tcrt_mid_left_pin = tcrt5000::initialize_tcrt5000(peripherals.GPIO4);
    let tcrt_mid_left = Tcrt5000::new(tcrt_mid_left_pin, true);

    let tcrt_left_pin = tcrt5000::initialize_tcrt5000(peripherals.GPIO5);
    let tcrt_left = Tcrt5000::new(tcrt_left_pin, true);

    let tcrt_mid_right_pin = tcrt5000::initialize_tcrt5000(peripherals.GPIO18);
    let tcrt_mid_right = Tcrt5000::new(tcrt_mid_right_pin, true);

    let tcrt_right_pin = tcrt5000::initialize_tcrt5000(peripherals.GPIO19);
    let tcrt_right = Tcrt5000::new(tcrt_right_pin, true);

    let l298n_ledc = l298n_control::initialize_ledc(peripherals.LEDC);
    let l298n_lstimer = l298n_control::initialize_lstimer(&l298n_ledc);

    let l298n_module_left = L298n::new(
        &l298n_ledc,
        &l298n_lstimer,
        channel::Number::Channel0,
        peripherals.GPIO12.into(),
        channel::Number::Channel1,
        peripherals.GPIO13.into(),
    );

    let l298n_module_rightt = L298n::new(
        &l298n_ledc,
        &l298n_lstimer,
        channel::Number::Channel2,
        peripherals.GPIO27.into(),
        channel::Number::Channel3,
        peripherals.GPIO14.into(),
    );

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        info!("Hello world!");
        Timer::after(Duration::from_secs(1)).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}
