#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::AnyPin;
use esp_hal::ledc::*;
use esp_hal::timer::timg::TimerGroup;
use l298n_driver::l298n_control::{self, L298n};
use log::info;
use static_cell::StaticCell;
use tcrt5000_driver::tcrt5000::{self, Tcrt5000};

#[derive(Clone, Copy)]
pub struct MotorCommand {
    pub direction: Direction,
}

pub struct Tcrt5000Array<'d> {
    pub left: Tcrt5000<'d>,
    pub mid_left: Tcrt5000<'d>,
    pub middle: Tcrt5000<'d>,
    pub mid_right: Tcrt5000<'d>,
    pub right: Tcrt5000<'d>,
}

pub struct L298nInitStruct {
    pub ledc: esp_hal::peripherals::LEDC<'static>,
    pub left_anode: AnyPin<'static>,
    pub left_cathode: AnyPin<'static>,
    pub right_anode: AnyPin<'static>,
    pub right_cathode: AnyPin<'static>,
}

#[derive(Clone, Copy)]
pub enum Direction {
    Straight,
    SlightLeft,
    FullLeft,
    SlightRight,
    FullRight,
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

static MOTOR_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, 2> = Channel::new();

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

    let l298n_init_struct = L298nInitStruct {
        ledc: peripherals.LEDC,
        left_anode: peripherals.GPIO12.into(),
        left_cathode: peripherals.GPIO13.into(),
        right_anode: peripherals.GPIO27.into(),
        right_cathode: peripherals.GPIO14.into(),
    };

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

    let tcrt5000_array = Tcrt5000Array {
        left: tcrt_left,
        mid_left: tcrt_mid_left,
        middle: tcrt_center,
        mid_right: tcrt_mid_right,
        right: tcrt_right,
    };

    static TCRT5000_SENSOR_ARRAY: StaticCell<Tcrt5000Array> = StaticCell::new();
    let tcrt5000_array_static = TCRT5000_SENSOR_ARRAY.init(tcrt5000_array);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // TODO: Spawn some tasks
    let _ = spawner;

    loop {
        //TODO Implement motor control logic
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples
}

#[embassy_executor::task]
async fn tcrt5000_task(sensor_array: &'static mut Tcrt5000Array<'static>) {
    //TODO Implement polling logic
}

#[embassy_executor::task]
async fn l298n_task(init_struct: L298nInitStruct) {
    let l298n_ledc = l298n_control::initialize_ledc(init_struct.ledc);
    let l298n_lstimer = l298n_control::initialize_lstimer(&l298n_ledc);

    let l298n_module_left = L298n::new(
        &l298n_ledc,
        &l298n_lstimer,
        channel::Number::Channel0,
        init_struct.left_anode,
        channel::Number::Channel1,
        init_struct.left_cathode,
    );

    let l298n_module_right = L298n::new(
        &l298n_ledc,
        &l298n_lstimer,
        channel::Number::Channel2,
        init_struct.right_anode,
        channel::Number::Channel3,
        init_struct.right_cathode,
    );

    loop {
        //TODO implement motor control logic
    }
}
