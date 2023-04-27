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
use ch32v1::ch32v103::{ RCC, TIM1, PFIC, USBHD };
use ch32v1::ch32v103::interrupt::Interrupt;

use ch32v103_hal::prelude::*;
use ch32v103_hal::rcc::*;
use ch32v103_hal::gpio::*;
use ch32v103_hal::delay::*;
use ch32v103_hal::gpio::gpiob::PB15;
use ch32v103_hal::gpio::gpioa::PA7;

use core::fmt::Write; // required for writeln!
use ch32v103_hal::serial::*;

use rand::SeedableRng;
use rand::Rng;

type LedPin = PB15<Output<PushPull>>;
static LED: Mutex<RefCell<Option<LedPin>>> = Mutex::new(RefCell::new(None));

type TriggerPin = PA7<Output<PushPull>>;
static TRIGGER: Mutex<RefCell<Option<TriggerPin>>> = Mutex::new(RefCell::new(None));

const DEV_DESC: [u8; 18] = [
    18, // size of descriptor
    0x01, // device descriptor (0x01)
    0x10, // USB 1.10 in BCD
    0x01,
    0, // class code. 0: defined in interface
    0, // subclass. 0: unused
    0, // protocol. 0: unused
    8, // max packet size for endpoint 0
    0x01, // vender ID
    0x23,
    0x45, // product ID
    0x67,
    0, // device version in BCD
    1,
    1, // index for manufacture string
    2, // index for product string
    3, // index for serial number string
    1, // number of possible configs
];

const CONFIG_DESC: [u8; 41] = [
    // configuration descriptor
    9, // size of descriptor
    0x02, // configuration descriptor (0x02)
    41, // total length: conf(9) + iface(9) + hid(9) + ep1(7+7)
    0,
    1, // num of ifaces
    1, // num of configs
    0, // no config string
    0x80, // device attrib: bus powered, no remote wakeup
    49, // max power by 2mA: 49 * 2mA = 08mA

    // interface descriptor
    9, // size of descriptor
    0x04, // interface descriptor (0x04)
    0, // iface #
    0, // Alt string
    2, // num of endpoints
    0x03, // HID class
    0, // no subclass
    2, // iface protrol: mouse
    4, // index of interface string

    // HID descriptor
    9, // size of descriptor
    0x21, // hid descriptor (0x21)
    0x10, // hid version
    0x01,
    0, // no country code
    1, // num of descriptor
    34, // descriptor type: report
    54, // report descriptor length
    0,

    // endpoint 1 OUT descriptor
    7, // size of descriptor
    0x05, // descriptor type: endpoint
    0x01, // endpoint address: endpoint 1 OUT
    0x03, // transfer type: interrupt
    MAX_LEN as u8, // max packet size
    0,
    10, // polling interval in ms

    // endpoint 1 IN descriptor
    7, // size of descriptor
    0x05, // descriptor type: endpoint
    0x81, // endpoint address: endpoint 1 IN
    0x03, // transfer type: interrupt
    MAX_LEN as u8, // max packet size
    0,
    10, // polling interval in ms
];

const RPT_DESC: [u8; 50] = [
    0x05, // USAGE_PAGE (Generic Desktop)
    0x01,
    0x09, // USAGE (Mouse)
    0x02,
    0xa1, // COLLECTION (Application)
    0x01,
    0x09, // USAGE (Pointer)
    0x01,
    0xa1, // COLLECTION (Physical)
    0x00,
    // Byte 0, bit 0~2 :: Mouse Buttons
    0x05, // USAGE_PAGE (Button)
    0x09,
    0x19, // USAGE_MINIMUM (Button 1)
    0x01,
    0x29, // USAGE_MAXIMUM (Button 3)
    0x03,
    0x15, // LOGICAL_MINIMUM (0)
    0x00,
    0x25, // LOGICAL_MAXIMUM (1)
    0x01,
    0x95, // REPORT_COUNT (3)
    0x03,
    0x75, // REPORT_SIZE (1)
    0x01,
    0x81, // INPUT (Data,Var,Abs)
    0x02,
    // Byte 0, bit 3~7: Constant(0)
    0x95, // REPORT_COUNT (1)
    0x01,
    0x75, // REPORT_SIZE (5)
    0x05,
    0x81, // INPUT (Cnst,Var,Abs)
    0x03,
    0x05, // USAGE_PAGE (Generic Desktop)
    0x01,
    // Byte 1, displacement Y
    0x09, //USAGE (X)
    0x30,
    // Byte 2, displacement X
    0x09, //USAGE (Y)
    0x31,
    0x15, //LOGICAL_MINIMUM (-127)
    0x81,
    0x25, //LOGICAL_MAXIMUM (127)
    0x7f,
    0x75, //REPORT_SIZE (8)
    0x08,
    0x95, // EPORT_COUNT (2)
    0x02,
    0x81, // NPUT (Data,Var,Rel)
    0x06,
    0xc0, // END_COLLECTION
    0xc0, // END_COLLECTION
];

const LANG_IDS: [u8; 4] = [
    4, // length
    0x03, // string descriptor (0x03)
    0x09, // 0x0409 English (United States)
    0x04,
    // 0x11, // 0x0411 Japanese
    // 0x04,
];

const STR_1: [u8; 12] = [
    12, // length
    0x03, // string descriptor (0x03)
    b'n',
    0,
    b'o',
    0,
    b's',
    0,
    b'u',
    0,
    b'z',
    0,
];

const STR_2: [u8; 18] = [
    18, // length
    0x03, // string descriptor (0x03)
    b'C',
    0,
    b'H',
    0,
    b'3',
    0,
    b'2',
    0,
    b'V',
    0,
    b'1',
    0,
    b'0',
    0,
    b'3',
    0,
];

const STR_3: [u8; 12] = [
    12, // length
    0x03, // string descriptor (0x03)
    b'1',
    0,
    b'.',
    0,
    b'2',
    0,
    b'.',
    0,
    b'3',
    0,
];

const STR_4: [u8; 22] = [
    22, // length
    0x03, // string descriptor (0x03)
    b'h',
    0,
    b'i',
    0,
    b'd',
    0,
    b' ',
    0,
    b's',
    0,
    b'a',
    0,
    b'm',
    0,
    b'p',
    0,
    b'l',
    0,
    b'e',
    0,
];

const USB_DESC: [&[u8]; 8] = [
    &DEV_DESC,
    &CONFIG_DESC,
    &LANG_IDS,
    &STR_1,
    &STR_2,
    &STR_3,
    &STR_4,
    &RPT_DESC,
];
#[derive(Copy, Clone)]
enum DescIndex {
    Device = 0,
    Configuration = 1,
    LangId = 2,
    String1 = 3,
    String2 = 4,
    String3 = 5,
    String4 = 6,
    Report = 7,
}
static mut DESC_IDX: DescIndex = DescIndex::Device;

const MAX_LEN: usize = 8;
static mut REQ_LEN: usize = 64;
static mut LAST_P: usize = 0; // end of descriptor
static mut BUF_P: usize = 0;

enum SetupState {
    Reset,
    GetDescriptor,
    SetAddress,
}
static mut SETUP_STATE: SetupState = SetupState::Reset;

#[repr(C)]
#[derive(Copy, Clone)]
struct SetupData {
    request_type: u8,
    request: u8,
    value: u16,
    index: u16,
    length: u16,
}

union SetupBuffer {
    bytes: [u8; 64],
    setup: SetupData,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct MouseData {
    _reserved: [u8; 64],
    button: u8,
    x: i8,
    y: i8,
    z: i8,
}

union MouseBuffer {
    bytes: [u8; 128],
    mouse: MouseData,
}

#[repr(C, align(4))]
struct AlignedBuffer {
    ep0: SetupBuffer,
    ep1: MouseBuffer,
}

static mut BUFFER: AlignedBuffer = AlignedBuffer {
    ep0: SetupBuffer { bytes: [0; 64] },
    ep1: MouseBuffer { bytes: [0; 128] },
};

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

    trigger.set_low().unwrap();
    delay.delay_ms(5);
    trigger.set_high().unwrap();
    delay.delay_ms(10);
    trigger.set_low().unwrap();

    critical_section::with(|cs| {
        TRIGGER.borrow(cs).replace(Some(trigger));
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

    // critical_section::with(|cs| {
    //     let mut trigger = TRIGGER.borrow(cs).borrow_mut();
    //     trigger.as_mut().unwrap().toggle().unwrap();
    // });

    unsafe {
        // writeln!(&mut log, "{:p}", (*USBHD::ptr()).udev_ctrl__uhost_ctrl.as_ptr()).unwrap();
        // writeln!(&mut log, "{:p}", (*USBHD::ptr()).uep0_t_len.as_ptr()).unwrap();
        // writeln!(&mut log, "{:p}", (*USBHD::ptr()).uep7_ctrl.as_ptr()).unwrap(); // expected 0x4002344e
        writeln!(&mut log, "{:p}", (*USBHD::ptr()).uep1_t_len.as_ptr()).unwrap();
    }

    init_usb();

    led1.set_low().unwrap();
    unsafe {
        BUFFER.ep1.mouse.x = 0;
        BUFFER.ep1.mouse.y = 0;
    }

    let mut rng = rand::rngs::SmallRng::from_seed([0; 16]);
    loop {
        unsafe {
            BUFFER.ep1.mouse.x = rng.gen::<i8>() / 8;
            BUFFER.ep1.mouse.y = rng.gen::<i8>() / 8;
            // waie USB bus idle
            while !(*USBHD::ptr()).usb_mis_st.read().ums_sie_free().bit_is_set() {}
            (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(3));

            writeln!(
                &mut log,
                "{:X} {},{}",
                BUFFER.ep1.mouse.button,
                BUFFER.ep1.mouse.x,
                BUFFER.ep1.mouse.y
            ).unwrap();

            // let len = (*USBHD::ptr()).uep1_t_len.read().bits() as u8;
            // writeln!(&mut log, "{:X}", len).unwrap();
            // let len = (0x4002_3434 as *mut u8).read_volatile();
            // writeln!(&mut log, "{:X}", len).unwrap();
        }
        delay.delay_ms(500);

        //     unsafe {
        //         writeln!(&mut log, ":").unwrap();

        //         let len = (*USBHD::ptr()).uep0_t_len.read().bits() as u8;
        //         // let len = (0x4002_3430 as *mut u8).read_volatile();
        //         writeln!(&mut log, "{:X}", len).unwrap();

        //         let ctlr = (*USBHD::ptr()).uep0_ctrl.read().bits() as u8;
        //         // let ctlr = (0x4002_3432 as *mut u8).read_volatile();
        //         writeln!(&mut log, "{:X}", ctlr).unwrap();
        //         // let ctlr = (*USBHD::ptr()).usb_ctrl.read().bits() as u8;
        //         // writeln!(&mut log, "{:X}", ctlr).unwrap();
        //     }
    }
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

fn init_usb() {
    //   USB_CTRL    = bUC_DEV_PU_EN               // USB internal pull-up enable
    //               | bUC_INT_BUSY                // Return NAK if USB INT flag not clear
    //               | bUC_DMA_EN;                 // DMA enable
    //   UDEV_CTRL   = bUD_PD_DIS                  // Disable UDP/UDM pulldown resistor
    //               | bUD_PORT_EN;                // Enable port, full-speed

    //   USB_CTRL   |= bUC_LOW_SPEED;
    //   UDEV_CTRL  |= bUD_LOW_SPEED;

    unsafe {
        (*RCC::ptr()).ahbpcenr.modify(|_, w| w.usbhden().set_bit());
        // reset USBHD
        (*RCC::ptr()).ahbrstr.modify(|_, w| w.usbhdrst().set_bit()); // AHBRSTR
        (*RCC::ptr()).ahbrstr.modify(|_, w| w.usbhdrst().clear_bit()); // AHBRSTR
        // (*RCC::ptr()).ahbpcenr.modify(|_, w| w.usbhden().set_bit());

        (*USBHD::ptr()).usb_dev_ad.write(|w| w.bits(0)); // reset USB device address

        (*USBHD::ptr()).usb_ctrl.modify(|_, w|
            w
                .uc_host_mode()
                .clear_bit() // set device mode
                .uc_low_speed()
                .set_bit()
                .mask_uc_sys_ctrl()
                .bits(0b11)
                .uc_int_busy() // *
                .set_bit()
                // both uc_reset_sie and uc_clr_all are cleared.
                .uc_reset_sie()
                .clear_bit()
                .uc_clr_all()
                .clear_bit()
                .uc_dma_en() // *
                .set_bit()
        );

        (*USBHD::ptr()).udev_ctrl__uhost_ctrl.modify(|_, w|
            w
                .ud_pd_dis__uh_pd_dis()
                .set_bit()
                .ud_low_speed__uh_low_speed()
                .set_bit()
                .ud_port_en__uh_port_en()
                .set_bit()
        );

        // setup endpoint 0
        // set DMA buffer for endpoint0
        (*USBHD::ptr()).uep0_dma.write(|w| w.bits(BUFFER.ep0.bytes.as_ptr() as u16));
        // UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK
        (*USBHD::ptr()).uep0_ctrl.modify(
            |_, w|
                w
                    .uep_r_tog()
                    .clear_bit()
                    .uep_t_tog()
                    .clear_bit()
                    .mask_uep_r_res()
                    .bits(0b0) //ACK
                    .mask_uep_t_res()
                    .bits(0b10) // NAK
        );
        (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(0));

        // setup endpoint 1
        (*USBHD::ptr()).uep1_dma.write(|w| w.bits(BUFFER.ep1.bytes.as_ptr() as u16));
        (*USBHD::ptr()).uep4_1_mod.modify(|_, w|
            w.uep1_rx_en().set_bit().uep1_tx_en().set_bit().uep1_buf_mod().clear_bit()
        );
        // UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK
        (*USBHD::ptr()).uep1_ctrl__uh_setup.modify(
            |_, w|
                w
                    .uep_r_tog__uh_pre_pid_en()
                    .set_bit()
                    .uep_t_tog__uh_sof_en()
                    .set_bit()
                    .uep_auto_tog()
                    .set_bit()
                    .mask_uep_r_res()
                    .bits(0b0) //ACK
                    .mask_uep_t_res()
                    .bits(0b0) // ACK
        );
        (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));

        // enable interrupts for USB
        (*USBHD::ptr()).usb_int_en.modify(|_, w|
            w.uie_suspend().set_bit().uie_transfer().set_bit().uie_bus_rst__uie_detect().set_bit()
        );
        (*USBHD::ptr()).usb_int_fg.write(|w| w.bits(0x1f)); // clear all interrupts
        (*PFIC::ptr()).ienr2.modify(|_, w| w.bits(0b1 << ((Interrupt::USBHD as u32) - 32)));
        riscv::interrupt::enable();
    }
}

interrupt!(USBHD, usb_handler);
fn usb_handler() {
    // BUFFER.ep0.setup.request_type,
    // BUFFER.ep0.setup.request,

    unsafe {
        let int_gf = (*USBHD::ptr()).usb_int_fg.read();
        if int_gf.uif_transfer().bit_is_set() {
            // critical_section::with(|cs| {
            //     let mut trigger = TRIGGER.borrow(cs).borrow_mut();
            //     trigger.as_mut().unwrap().toggle().unwrap();
            // });

            //         let rx_len = (*USBHD::ptr()).usb_rx_len.read().bits();
            let mut len = 0;
            let int_st = (*USBHD::ptr()).usb_int_st.read();
            let int_endp = int_st.mask_uis_h_res__mask_uis_endp().bits();
            match int_st.mask_uis_token().bits() {
                0b11 => {
                    // SETUP
                    if int_endp == 0 {
                        match BUFFER.ep0.setup.request {
                            0x05 => {
                                // SET_ADDRESS
                                SETUP_STATE = SetupState::SetAddress;
                                len = 0;
                            }
                            0x06 => {
                                // GET_DESCRIPTOR
                                match BUFFER.ep0.setup.request_type {
                                    0x80 => {
                                        SETUP_STATE = SetupState::GetDescriptor;
                                        DESC_IDX = match BUFFER.ep0.setup.value {
                                            // return device descriptor
                                            0x100 => { DescIndex::Device }
                                            // return configuration descriptor
                                            0x200 => { DescIndex::Configuration }
                                            // return supporting languages list
                                            0x300 => { DescIndex::LangId }
                                            // ignore specified language
                                            0x301 => { DescIndex::String1 }
                                            0x302 => { DescIndex::String2 }
                                            0x303 => { DescIndex::String3 }
                                            0x304 => { DescIndex::String4 }
                                            _ => {
                                                critical_section::with(|cs| {
                                                    let mut trigger =
                                                        TRIGGER.borrow(cs).borrow_mut();
                                                    trigger.as_mut().unwrap().toggle().unwrap();
                                                });

                                                unreachable!();
                                            }
                                        };
                                    }
                                    0x81 => {
                                        SETUP_STATE = SetupState::GetDescriptor;
                                        DESC_IDX = match BUFFER.ep0.setup.value {
                                            // return report descriptor
                                            0x2200 => { DescIndex::Report }
                                            _ => {
                                                critical_section::with(|cs| {
                                                    let mut trigger =
                                                        TRIGGER.borrow(cs).borrow_mut();
                                                    trigger.as_mut().unwrap().toggle().unwrap();
                                                });

                                                DescIndex::Device
                                            }
                                        };
                                    }
                                    _ => {
                                        unreachable!();
                                    }
                                }

                                REQ_LEN = BUFFER.ep0.setup.length as usize;

                                // reset pointer
                                BUF_P = 0;
                                // set max data size
                                LAST_P = USB_DESC[DESC_IDX as usize].len() as usize;
                                if LAST_P > REQ_LEN {
                                    LAST_P = REQ_LEN;
                                }

                                if LAST_P > BUF_P {
                                    // set available max length
                                    len = LAST_P - BUF_P;
                                    if len > MAX_LEN {
                                        len = MAX_LEN;
                                    }
                                    // copy data to endpoint buffer
                                    for i in 0..len {
                                        BUFFER.ep0.bytes[i] = USB_DESC[DESC_IDX as usize][
                                            BUF_P + i
                                        ];
                                    }
                                    // update pointer
                                    BUF_P += len;
                                }
                            }
                            0x09 => {
                                // SET_CONFIGURATION
                                // configure to BUFFER.ep0.setup_value
                                if BUFFER.ep0.setup.request_type == 0x00 {
                                }
                            }
                            0x0a => {
                                if BUFFER.ep0.setup.request_type == 0x21 {
                                }
                            }
                            _ => {
                                unreachable!();
                            }
                        }

                        // set transfer length
                        (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(len as u8));
                        // UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;// Expect DATA1, Answer ACK
                        (*USBHD::ptr()).uep0_ctrl.modify(
                            |_, w|
                                w
                                    .uep_r_tog()
                                    .set_bit()
                                    .uep_t_tog()
                                    .set_bit()
                                    .mask_uep_r_res()
                                    .bits(0) // ACK
                                    .mask_uep_t_res()
                                    .bits(0) // ACK
                            // .bits(0b10) // NAK
                        );
                    }
                }
                0b10 => {
                    // IN
                    match int_endp {
                        0 => {
                            match SETUP_STATE {
                                SetupState::SetAddress => {
                                    (*USBHD::ptr()).usb_dev_ad.write(|w|
                                        w.bits(BUFFER.ep0.setup.value as u8)
                                    );
                                }
                                SetupState::GetDescriptor => {
                                    if LAST_P > BUF_P {
                                        // set available max length
                                        len = LAST_P - BUF_P;
                                        if len > MAX_LEN {
                                            len = MAX_LEN;
                                        }
                                        // copy data to endpoint buffer
                                        for i in 0..len {
                                            BUFFER.ep0.bytes[i] = USB_DESC[DESC_IDX as usize][
                                                BUF_P + i
                                            ];
                                        }
                                        // update pointer
                                        BUF_P += len;
                                    }
                                }
                                _ => {
                                    unreachable!();
                                }
                            }
                            // set transfer length
                            (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(len as u8));

                            // toggle UEP_R_TOG (bit 7) and UEP_T_TOG (bit 6)
                            (*USBHD::ptr()).uep0_ctrl.modify(|r, w| w.bits(r.bits() ^ 0xc0));
                        }
                        1 => {
                            // set transfer length
                            // reset tx length set in main
                            (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));
                        }
                        _ => {
                            unreachable!();
                        }
                    }
                    if int_endp == 0 {
                    }
                }
                0b00 => {
                    // OUT
                }
                _ => {
                    // unhandled
                    unreachable!();
                }
            }

            // clear USB_INT_FG
            // clearing USB_INT_GG makes ACK for transaction
            (*USBHD::ptr()).usb_int_fg.modify(|_, w| w.uif_transfer().set_bit())
        } else if int_gf.uif_suspend().bit_is_set() {
            (*USBHD::ptr()).usb_int_fg.modify(|_, w| w.uif_suspend().set_bit());
            if (*USBHD::ptr()).usb_mis_st.read().ums_suspend().bit_is_set() {
                (*USBHD::ptr()).usb_int_fg.write(|w| w.bits(0x1f)); // clear all interrupts
            }
        } else if int_gf.uif_bus_rst__uif_detect().bit_is_set() {
            (*USBHD::ptr()).uep0_ctrl.modify(
                |_, w|
                    w
                        .uep_r_tog()
                        .clear_bit()
                        .uep_t_tog()
                        .clear_bit()
                        .mask_uep_r_res()
                        .bits(0b0) //ACK
                        .mask_uep_t_res()
                        .bits(0b10) // NAK
            );
            (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(0));

            (*USBHD::ptr()).usb_dev_ad.write(|w| w.bits(0)); // reset USB device address
            (*USBHD::ptr()).usb_int_fg.write(|w| w.bits(0x1f)); // clear all interrupts
        }
    }
}