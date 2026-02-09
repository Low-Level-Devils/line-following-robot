#![no_std]

pub mod line_following_robot {
    use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
    use embassy_sync::channel::Channel;
    use embassy_time::{Duration, Timer};
    use esp_hal::gpio::AnyPin;
    use esp_hal::ledc::*;
    use l298n_driver::l298n_control::{self, L298n};
    use log::info;
    use tcrt5000_driver::tcrt5000::{self, Tcrt5000};

    static MOTOR_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, Direction, 2> = Channel::new();

    pub struct Tcrt5000ArrayInitStruct {
        pub left_pin: AnyPin<'static>,
        pub mid_left_pin: AnyPin<'static>,
        pub middle_pin: AnyPin<'static>,
        pub mid_right_pin: AnyPin<'static>,
        pub right_pin: AnyPin<'static>,
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
        FullLeft,
        SlightLeft,
        Straight,
        SlightRight,
        FullRight,
    }

    #[embassy_executor::task]
    pub async fn tcrt5000_task(tcrt5000_array_init_struct: Tcrt5000ArrayInitStruct) {
        let tcrt_left_pin = tcrt5000::initialize_tcrt5000(tcrt5000_array_init_struct.left_pin);
        let tcrt_left = Tcrt5000::new(tcrt_left_pin, true);

        let tcrt_mid_left_pin =
            tcrt5000::initialize_tcrt5000(tcrt5000_array_init_struct.mid_left_pin);
        let tcrt_mid_left = Tcrt5000::new(tcrt_mid_left_pin, true);

        let tcrt_center_pin = tcrt5000::initialize_tcrt5000(tcrt5000_array_init_struct.middle_pin);
        let tcrt_center = Tcrt5000::new(tcrt_center_pin, true);

        let tcrt_mid_right_pin =
            tcrt5000::initialize_tcrt5000(tcrt5000_array_init_struct.mid_right_pin);
        let tcrt_mid_right = Tcrt5000::new(tcrt_mid_right_pin, true);

        let tcrt_right_pin = tcrt5000::initialize_tcrt5000(tcrt5000_array_init_struct.right_pin);
        let tcrt_right = Tcrt5000::new(tcrt_right_pin, true);

        let trct5000_array = [
            tcrt_left,
            tcrt_mid_left,
            tcrt_center,
            tcrt_mid_right,
            tcrt_right,
        ];

        info!("TCRT5000 Array initialized. Begin polling...");

        loop {
            for (index, sensor) in trct5000_array.iter().enumerate() {
                if sensor.poll_sensor() {
                    let pos = match index {
                        0 => Direction::FullLeft,
                        1 => Direction::SlightLeft,
                        2 => Direction::Straight,
                        3 => Direction::SlightRight,
                        4 => Direction::FullRight,
                        _ => continue,
                    };

                    MOTOR_COMMAND_CHANNEL.send(pos).await;

                    break;
                }
            }

            Timer::after(Duration::from_millis(1)).await;
        }
    }

    #[embassy_executor::task]
    pub async fn l298n_task(init_struct: L298nInitStruct) {
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

        l298n_module_left.change_speed(50);
        l298n_module_right.change_speed(50);

        info!("l298n modules initialized! Searching for path");

        loop {
            let command = MOTOR_COMMAND_CHANNEL.receive().await;

            match command {
                Direction::FullLeft => {
                    l298n_module_left.change_speed(75);
                    l298n_module_right.change_speed(0);
                }
                Direction::SlightLeft => {
                    l298n_module_left.change_speed(75);
                    l298n_module_right.change_speed(25);
                }
                Direction::Straight => {
                    l298n_module_left.change_speed(50);
                    l298n_module_right.change_speed(50);
                }
                Direction::SlightRight => {
                    l298n_module_left.change_speed(25);
                    l298n_module_right.change_speed(75);
                }
                Direction::FullRight => {
                    l298n_module_left.change_speed(0);
                    l298n_module_right.change_speed(75);
                }
            }
        }
    }
}
