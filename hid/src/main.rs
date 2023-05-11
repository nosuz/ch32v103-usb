#![no_std]
#![no_main]

// Ref
// https://github.com/wagiminator/CH552-USB-OLED/tree/main/software/hid_i2c_bridge/src

use core::cell::RefCell;
use critical_section::Mutex;

// provide implementation for critical-section
use ch32v_rt::entry;
use ch32v_rt::interrupt;
use panic_halt as _;

// use ch32v1::ch32v103; // PAC for CH32V103
use ch32v1::ch32v103::Peripherals;
use ch32v1::ch32v103::{ RCC, TIM1, PFIC };
use ch32v1::ch32v103::interrupt::Interrupt;

use ch32v103_hal::prelude::*;
use ch32v103_hal::rcc::*;
use ch32v103_hal::gpio::*;
use ch32v103_hal::delay::*;
use ch32v103_hal::gpio::gpiob::PB15;
use ch32v103_hal::gpio::gpioa::PA7;

use core::fmt::Write; // required for writeln!
use ch32v103_hal::serial::*;

mod usb;
use usb::handler::{ init_usb, usb_interrupt_handler };
use usb::hid_mouse::{ MouseStatus, update_mouse };

use rand::SeedableRng;
use rand::Rng;

type LedPin = PB15<Output<PushPull>>;
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

type TriggerPin = PA7<Output<PushPull>>;
static TRIGGER: Mutex<RefCell<Option<TriggerPin>>> = Mutex::new(RefCell::new(None));

interrupt!(TIM1_UP, tim1_up);
fn tim1_up() {
    unsafe {
        (*TIM1::ptr()).intfr.modify(|_, w| w.uif().clear_bit());
    }

    critical_section::with(|cs| {
        let mut led = LED.borrow(cs).borrow_mut();
        led.as_mut().unwrap().toggle().unwrap();
    });
}

interrupt!(USBHD, usb_interrupt_handler);

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let rcc = peripherals.RCC.constrain();

    // let clocks = rcc.cfgr.freeze();
    // 72MHz not worked for me
    let clocks = rcc.cfgr.use_pll((48).mhz(), PllClkSrc::Hsi).freeze();

    let gpioa = peripherals.GPIOA.split();
    let mut trigger = gpioa.pa7.into_push_pull_output();

    let gpiob = peripherals.GPIOB.split();
    let led = gpiob.pb15.into_push_pull_output();

    let mut delay = Delay::new(&clocks);

    // https://docs.rs/critical-section/latest/critical_section/#
    critical_section::with(|cs| {
        LED.borrow(cs).replace(Some(led));
    });

    setup_timer1(&clocks);

    // Serial
    let pa9 = gpioa.pa9.into_multiplex_push_pull_output();
    let pa10 = gpioa.pa10.into_floating_input();
    //  remapped ports
    // let pb6 = gpiob.pb6.into_multiplex_push_pull_output();
    // let pb7 = gpiob.pb7.into_floating_input();

    let usart = Serial::usart1(peripherals.USART1, (pa9, pa10), (115200).bps(), &clocks);
    let (tx, _) = usart.split();
    let mut log = SerialWriter::new(tx);

    delay.delay_ms(200);
    trigger.toggle().unwrap();
    critical_section::with(|cs| {
        TRIGGER.borrow(cs).replace(Some(trigger));
    });

    init_usb();

    let mut rng = rand::rngs::SmallRng::from_seed([0; 16]);
    loop {
        let mouse_status = MouseStatus {
            right_button: false,
            middle_button: false,
            left_button: false,
            x_axis: rng.gen::<i8>() / 8,
            y_axis: rng.gen::<i8>() / 8,
        };
        update_mouse(&mouse_status);
        writeln!(&mut log, "{},{}", mouse_status.x_axis, mouse_status.y_axis).unwrap();
        delay.delay_ms(500);
    }
}

fn setup_timer1(clocks: &Clocks) {
    unsafe {
        (*RCC::ptr()).apb2pcenr.modify(|_, w| w.tim1en().set_bit());

        let prescale = (clocks.pclk2().0 / 1_000_000) * 100 - 1; // count for 0.1ms
        (*TIM1::ptr()).psc.write(|w| w.bits(prescale as u16));
        let down_count: u16 = 300 * 10 - 1; // 0.1ms * 10 * 300 = 300ms
        // (*TIM1::ptr()).cnt.write(|w| w.bits(down_count));
        (*TIM1::ptr()).atrlr.write(|w| w.bits(down_count));
        (*TIM1::ptr()).ctlr1.modify(|_, w| w.arpe().set_bit().cen().set_bit());

        // clear interupt requist by the above counter update
        // (*TIM1::ptr()).intfr.modify(|_, w| w.uif().clear_bit());
        // (*PFIC::ptr()).iprr2.write(|w| w.bits(0b1 << ((Interrupt::TIM1_UP as u32) - 32)));

        // enable interrupt on Update. All 3 lines are require to enable correct interrupt.
        (*PFIC::ptr()).ienr2.modify(|_, w| w.bits(0b1 << ((Interrupt::TIM1_UP as u32) - 32)));
        (*TIM1::ptr()).dmaintenr.modify(|_, w| w.uie().set_bit());
        riscv::interrupt::enable();
    }
}