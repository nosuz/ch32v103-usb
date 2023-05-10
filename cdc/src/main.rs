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
use ch32v1::ch32v103::{ RCC, TIM1, PFIC, USART1 };
use ch32v1::ch32v103::interrupt::Interrupt;

use ch32v103_hal::prelude::*;
use ch32v103_hal::rcc::*;
use ch32v103_hal::gpio::*;
use ch32v103_hal::delay::*;
use ch32v103_hal::gpio::gpiob::PB15;
use ch32v103_hal::gpio::gpioa::PA7;

use ch32v103_hal::serial::*;

mod usb;
use usb::handler::{ init_usb, usb_interrupt_handler };

type LedPin = PB15<Output<PushPull>>;
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

type TriggerPin = PA7<Output<PushPull>>;
static TRIGGER: Mutex<RefCell<Option<TriggerPin>>> = Mutex::new(RefCell::new(None));

pub mod ring_buffer;
use ring_buffer::RingBuffer;

use crate::usb::handler::MAX_LEN;
const RING_BUFFER_SIZE: usize = MAX_LEN * 4;
pub static mut TX_BUFFER: RingBuffer<u8, RING_BUFFER_SIZE> = RingBuffer::new();
pub static mut RX_BUFFER: RingBuffer<u8, RING_BUFFER_SIZE> = RingBuffer::new();

pub static mut SPEED_BUFFER: RingBuffer<u32, 4> = RingBuffer::new();

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
    let mut led1 = gpiob.pb2.into_push_pull_output();
    let led2 = gpiob.pb15.into_push_pull_output();

    let mut delay = Delay::new(&clocks);

    // https://docs.rs/critical-section/latest/critical_section/#
    critical_section::with(|cs| {
        LED.borrow(cs).replace(Some(led2));
    });

    setup_timer1(&clocks);

    // Serial
    let pa9 = gpioa.pa9.into_multiplex_push_pull_output();
    let pa10 = gpioa.pa10.into_floating_input();
    //  remapped ports
    // let pb6 = gpiob.pb6.into_multiplex_push_pull_output();
    // let pb7 = gpiob.pb7.into_floating_input();
    let usart = Serial::usart1(peripherals.USART1, (pa9, pa10), (115200).bps(), &clocks);
    let usart_base_freq = usart.get_base_freq();
    let (mut tx, mut rx) = usart.split();

    delay.delay_ms(200);
    trigger.toggle().unwrap();
    critical_section::with(|cs| {
        TRIGGER.borrow(cs).replace(Some(trigger));
    });

    init_usb();

    led1.set_low().unwrap();

    loop {
        let popped_speed = unsafe { SPEED_BUFFER.pop() };
        match popped_speed {
            Some(speed) => {
                set_usart1_bps(speed.bps(), usart_base_freq);
            }
            None => {}
        }

        if tx.is_ready() {
            let popped_tx = unsafe { TX_BUFFER.pop() };
            match popped_tx {
                Some(val) => {
                    // send from USART
                    let _ = tx.write(val);
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
                    }
                }
                _ => {}
            }
        }
    }
}

fn set_usart1_bps(baud_rate: Bps, base_freq: Hertz) {
    let brr_div: u32 = base_freq.0 / baud_rate.0;
    unsafe { (*USART1::ptr()).brr.write(|w| w.bits(brr_div)) }
}

fn setup_timer1(clocks: &Clocks) {
    unsafe {
        (*RCC::ptr()).apb2pcenr.modify(|_, w| w.tim1en().set_bit());

        let prescale = (clocks.hclk().0 / 1_000_000) * 100 - 1; // count for 0.1ms
        (*TIM1::ptr()).psc.write(|w| w.bits(prescale as u16));
        let down_count: u16 = 300 * 10 - 1; // 0.1ms * 10 * 300 = 300ms
        (*TIM1::ptr()).cnt.write(|w| w.bits(down_count));
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

interrupt!(USBHD, usb_interrupt_handler);