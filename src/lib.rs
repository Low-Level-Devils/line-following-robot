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

    static MOTOR_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, 2> =
        Channel::new();

    #[derive(Clone, Copy)]
    pub struct MotorCommand {
        pub direction: Direction,
    }

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
        Straight,
        SlightLeft,
        FullLeft,
        SlightRight,
        FullRight,
    }

    #[embassy_executor::task]
    async fn tcrt5000_task(tcrt5000_array_init_struct: Tcrt5000ArrayInitStruct) {
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

        loop {
            //TODO Implement polling logic
        }
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
}
