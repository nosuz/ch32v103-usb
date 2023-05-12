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
use ch32v1::ch32v103::{ RCC, TIM1, TIM3, PFIC, USART1 };
use ch32v1::ch32v103::interrupt::Interrupt;

use ch32v103_hal::prelude::*;
use ch32v103_hal::rcc::*;
use ch32v103_hal::gpio::*;
use ch32v103_hal::delay::*;
use ch32v103_hal::gpio::gpiob::{ PB15, PB2 };
use ch32v103_hal::gpio::gpioa::PA7;

use ch32v103_hal::serial::*;

mod usb;
use usb::handler::{ init_usb, usb_interrupt_handler };

type LedPin = PB15<Output<PushPull>>;
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

type ActivityPin = PB2<Output<PushPull>>;
static ACTIVITY: Mutex<RefCell<Option<ActivityPin>>> = Mutex::new(RefCell::new(None));

type TriggerPin = PA7<Output<PushPull>>;
static TRIGGER: Mutex<RefCell<Option<TriggerPin>>> = Mutex::new(RefCell::new(None));

pub mod ring_buffer;
use ring_buffer::RingBuffer;

use crate::usb::handler::MAX_LEN;
const RING_BUFFER_SIZE: usize = MAX_LEN * 4;
pub static mut TX_BUFFER: RingBuffer<u8, RING_BUFFER_SIZE> = RingBuffer::new();
pub static mut RX_BUFFER: RingBuffer<u8, RING_BUFFER_SIZE> = RingBuffer::new();

pub static mut SPEED_BUFFER: RingBuffer<u32, 4> = RingBuffer::new();
pub static mut CONTROL_BUFFER: RingBuffer<u8, 4> = RingBuffer::new();

static mut ACTIVE_SERIAL: bool = false;
static mut ACTIVITY_TICK: bool = false;

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
    let led1 = gpiob.pb2.into_push_pull_output();
    let led2 = gpiob.pb15.into_push_pull_output();

    let mut delay = Delay::new(&clocks);

    // https://docs.rs/critical-section/latest/critical_section/#
    critical_section::with(|cs| {
        LED.borrow(cs).replace(Some(led2));
    });

    setup_timer1(&clocks);

    critical_section::with(|cs| {
        ACTIVITY.borrow(cs).replace(Some(led1));
    });

    setup_timer3(&clocks);

    // Serial
    let pa9 = gpioa.pa9.into_multiplex_push_pull_output();
    let pa10 = gpioa.pa10.into_floating_input();
    //  remapped ports
    // let pb6 = gpiob.pb6.into_multiplex_push_pull_output();
    // let pb7 = gpiob.pb7.into_floating_input();
    let usart = Serial::usart1(peripherals.USART1, (pa9, pa10), (115200).bps(), &clocks);
    let usart_base_freq = usart.get_base_freq();
    let (mut tx, mut rx) = usart.split();

    let mut dtr = gpiob.pb8.into_push_pull_output();
    let mut rts = gpiob.pb9.into_push_pull_output();
    dtr.set_high().unwrap();
    rts.set_high().unwrap();
    let mut control_state: u8 = 0;

    delay.delay_ms(200);
    trigger.toggle().unwrap();
    critical_section::with(|cs| {
        TRIGGER.borrow(cs).replace(Some(trigger));
    });

    init_usb();

    loop {
        let popped_speed = unsafe { SPEED_BUFFER.pop() };
        match popped_speed {
            Some(speed) => {
                set_usart1_bps(speed.bps(), usart_base_freq);
            }
            None => {}
        }

        let popped_control = unsafe { CONTROL_BUFFER.pop() };
        match popped_control {
            Some(control) => {
                // control GPIOs for DTR and RTS
                // bit 0: DTR
                // bit 1: RTS
                match (control & 0x03) ^ control_state {
                    0b00 => {
                        // no state changed
                    }
                    0b01 => {
                        dtr.toggle().unwrap();
                    }
                    0b10 => {
                        rts.toggle().unwrap();
                    }
                    0b11 => {
                        dtr.toggle().unwrap();
                        rts.toggle().unwrap();
                    }
                    _ => {
                        unreachable!();
                    }
                }
                control_state = control & 0x03;
            }
            None => {}
        }
        if tx.is_ready() {
            let popped_tx = unsafe { TX_BUFFER.pop() };
            match popped_tx {
                Some(val) => {
                    // send from USART
                    let _ = tx.write(val);
                    unsafe {
                        ACTIVE_SERIAL = true;
                    }
                }
                None => {}
            }
        }

        // check USART and push to RX_BUFFER
        if !rx.is_empty() {
            match rx.read() {
                Ok(data) => {
                    unsafe {
                        let _ = RX_BUFFER.push(data);
                        ACTIVE_SERIAL = true;
                    }
                }
                _ => {}
            }
        }
    }
}

fn set_usart1_bps(baud_rate: Bps, base_freq: Hertz) {
    let brr_div: u32 = base_freq.0 / baud_rate.0;
    unsafe {
        (*USART1::ptr()).brr.write(|w| w.bits(brr_div));
    }
}

fn setup_timer1(clocks: &Clocks) {
    unsafe {
        (*RCC::ptr()).apb2pcenr.modify(|_, w| w.tim1en().set_bit());

        let prescale = (clocks.pclk2().0 / 1_000_000) * 100 - 1; // count for 0.1ms
        (*TIM1::ptr()).psc.write(|w| w.bits(prescale as u16));
        let down_count: u16 = 300 * 10 - 1; // 0.1ms * 10 * 300 = 300ms
        (*TIM1::ptr()).cnt.write(|w| w.bits(down_count));
        (*TIM1::ptr()).atrlr.write(|w| w.bits(down_count));
        (*TIM1::ptr()).ctlr1.modify(|_, w| w.arpe().set_bit().cen().clear_bit());
        // apply setting
        (*TIM1::ptr()).swevgr.write(|w| w.ug().set_bit());

        // enable interrupt on Update. All 3 lines are require to enable correct interrupt.
        (*PFIC::ptr()).ienr2.modify(|_, w| w.bits(0b1 << ((Interrupt::TIM1_UP as u32) - 32)));
        // clear interrupt flag
        (*TIM1::ptr()).intfr.modify(|_, w| w.uif().clear_bit());
        // enable update interrupt
        (*TIM1::ptr()).dmaintenr.modify(|_, w| w.uie().set_bit());
        riscv::interrupt::enable();

        // start timer
        (*TIM1::ptr()).ctlr1.modify(|_, w| w.cen().set_bit());
    }
}

interrupt!(USBHD, usb_interrupt_handler);

fn setup_timer3(clocks: &Clocks) {
    // APB1
    unsafe {
        (*RCC::ptr()).apb1pcenr.modify(|_, w| w.tim3en().set_bit());

        (*TIM3::ptr()).smcfgr.modify(|_, w| w.sms().bits(0)); // Driven by the internal clock CK_INT

        let prescale = (clocks.pclk1().0 / 1_000_000) * 100 - 1; // count for 0.1ms
        (*TIM3::ptr()).psc.write(|w| w.bits(prescale as u16));

        let down_count: u16 = 10 * 50 - 1; // 0.1ms * 10 * 50 = 50ms
        // set cnt make interrupt
        (*TIM3::ptr()).cnt.write(|w| w.bits(down_count));
        (*TIM3::ptr()).atrlr.write(|w| w.bits(down_count));

        (*TIM3::ptr()).ctlr1.modify(|_, w| w.arpe().set_bit().cen().clear_bit());

        // apply setting
        (*TIM3::ptr()).swevgr.write(|w| w.ug().set_bit());

        // enable interrupt on Update. All 3 lines are require to enable correct interrupt.
        (*PFIC::ptr()).ienr2.modify(|r, w|
            w.bits(r.bits() | (0b1 << ((Interrupt::TIM3 as u32) - 32)))
        );
        // clear interrupt flag
        (*TIM3::ptr()).intfr.modify(|_, w| w.uif().clear_bit());
        // enable update interrupt
        (*TIM3::ptr()).dmaintenr.modify(|_, w| w.uie().set_bit());
        riscv::interrupt::enable();

        // start counter
        (*TIM3::ptr()).ctlr1.modify(|_, w| w.cen().set_bit());
    }
}

interrupt!(TIM3, timer3_handler);
fn timer3_handler() {
    unsafe {
        (*TIM3::ptr()).intfr.modify(|_, w| w.uif().clear_bit());
        ACTIVITY_TICK = !ACTIVITY_TICK;

        critical_section::with(|cs| {
            let mut led = ACTIVITY.borrow(cs).borrow_mut();
            match ACTIVITY_TICK {
                true => {
                    if ACTIVE_SERIAL {
                        led.as_mut().unwrap().set_low().unwrap();
                        ACTIVE_SERIAL = false;
                    }
                }
                false => {
                    led.as_mut().unwrap().set_high().unwrap();
                }
            }
        });
    }
}