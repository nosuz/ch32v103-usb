#![no_std]
#![no_main]

// Ref
// https://github.com/wagiminator/CH552-USB-OLED/tree/main/software/hid_i2c_bridge/src

use core::cell::RefCell;
use critical_section::Mutex;

use core::ptr::read_volatile;

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
    0x00, // USB 2.0 in BCD
    0x02,
    0, // class code. 0: defined in interface
    0, // subclass. 0: unused
    0, // protocol. 0: unused
    MAX_LEN as u8, // max packet size for endpoint 0
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

// https://github.com/RoboMaster/DevelopmentBoard-Examples/blob/master/USB/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
// also https://gist.github.com/tai/acd59b125a007ad47767

const USB_DESC_TYPE_CONFIGURATION: u8 = 0x02; // bDescriptorType: Configuration
const USB_DESC_TYPE_INTERFACE: u8 = 0x04; // bDescriptorType: Interface
const CS_INTERFACE: u8 = 0x24;
const USB_DESC_TYPE_ENDPOINT: u8 = 0x05;

const EP_IN: u8 = 0x80;
const EP_OUT: u8 = 0x00;

const CDC_CMD_EP: u8 = 1;
const CDC_OUT_EP: u8 = 2;
const CDC_IN_EP: u8 = 3;

// __ALIGN_BEGIN uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END =
const CONFIG_DESC: [u8; 67] = [
    //Configuration Descriptor
    0x09, // bLength: Configuration Descriptor size
    USB_DESC_TYPE_CONFIGURATION, // bDescriptorType: Configuration
    // wTotalLength:no of returned bytes
    // total length: conf(9) + iface(9) + cdc(5 + 5 + 4 + 5) + ep1(7) + iface(9) +  + ep2(7 + 7)
    67, // or length of CONFIG_DESC
    0x00,
    0x02, // bNumInterfaces: 2 interface
    0x01, // bConfigurationValue: Configuration value
    0x00, // iConfiguration: Index of string descriptor describing the configuration
    0xc0, // bmAttributes: self powered
    0x32, // MaxPower 0 mA

    //---------------------------------------------------------------------------

    //Interface Descriptor
    0x09, // bLength: Interface Descriptor size
    USB_DESC_TYPE_INTERFACE, // bDescriptorType: Interface
    // Interface descriptor type
    0x00, // bInterfaceNumber: Number of Interface
    0x00, // bAlternateSetting: Alternate setting
    0x01, // bNumEndpoints: One endpoints used
    0x02, // bInterfaceClass: Communication Interface Class
    0x02, // bInterfaceSubClass: Abstract Control Model
    0x01, // bInterfaceProtocol: Common AT commands
    0x00, // iInterface:

    //Header Functional Descriptor
    0x05, // bLength: Endpoint Descriptor size
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x00, // bDescriptorSubtype: Header Func Desc
    0x10, // bcdCDC: spec release number
    0x01,

    //Call Management Functional Descriptor
    0x05, // bFunctionLength
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x01, // bDescriptorSubtype: Call Management Func Desc
    0x00, // bmCapabilities: D0+D1
    0x01, // bDataInterface: 1

    //ACM Functional Descriptor
    0x04, // bFunctionLength
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x02, // bDescriptorSubtype: Abstract Control Management desc
    0x02, // bmCapabilities

    //Union Functional Descriptor
    0x05, // bFunctionLength
    CS_INTERFACE, // bDescriptorType: CS_INTERFACE
    0x06, // bDescriptorSubtype: Union func desc
    0x00, // bMasterInterface: Communication class interface
    0x01, // bSlaveInterface0: Data Class Interface

    //Endpoint 1 Descriptor
    0x07, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_ENDPOINT, // bDescriptorType: Endpoint
    CDC_CMD_EP + EP_IN, // bEndpointAddress
    0x03, // bmAttributes: Interrupt
    MAX_LEN as u8, // wMaxPacketSize:
    0,
    0x10, // bInterval:
    //---------------------------------------------------------------------------

    //Data class interface descriptor
    0x09, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_INTERFACE, // bDescriptorType:
    0x01, // bInterfaceNumber: Number of Interface
    0x00, // bAlternateSetting: Alternate setting
    0x02, // bNumEndpoints: Two endpoints used
    0x0a, // bInterfaceClass: CDC
    0x00, // bInterfaceSubClass:
    0x00, // bInterfaceProtocol:
    0x00, // iInterface:

    //Endpoint OUT Descriptor
    0x07, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_ENDPOINT, // bDescriptorType: Endpoint
    CDC_OUT_EP + EP_OUT, // bEndpointAddress
    0x02, // bmAttributes: Bulk
    MAX_LEN as u8, // wMaxPacketSize:
    0,
    0x00, // bInterval: ignore for Bulk transfer

    //Endpoint IN Descriptor
    0x07, // bLength: Endpoint Descriptor size
    USB_DESC_TYPE_ENDPOINT, // bDescriptorType: Endpoint
    CDC_IN_EP + EP_IN, // bEndpointAddress
    0x02, // bmAttributes: Bulk
    MAX_LEN as u8, // wMaxPacketSize:
    0,
    0x00, // bInterval: ignore for Bulk transfer
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
    b'c',
    0,
    b'd',
    0,
    b'c',
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

const QUAL_DESC: [u8; 0] = [];

const USB_DESC: [&[u8]; 8] = [
    &DEV_DESC,
    &CONFIG_DESC,
    &LANG_IDS,
    &STR_1,
    &STR_2,
    &STR_3,
    &STR_4,
    &QUAL_DESC,
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
    Qualifier = 7,
    Stalled = 0xff,
}
static mut DESC_IDX: DescIndex = DescIndex::Stalled;

// FIXME: for full-speed
// const MAX_LEN: usize = 64;
const MAX_LEN: usize = 8;
static mut REQ_LEN: usize = MAX_LEN;
static mut LAST_P: usize = 0; // end of descriptor
static mut BUF_P: usize = 0;

enum SetupState {
    Reset,
    GetDescriptor,
    SetAddress,
    Skip,
    Stalled,
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
    bytes: [u8; MAX_LEN],
    setup: SetupData,
}

union PlainBuffer {
    bytes: [u8; MAX_LEN],
}

#[repr(C, align(4))]
struct AlignedBuffer {
    ep0: SetupBuffer,
    ep1_cmd: PlainBuffer,
    ep2_rx: PlainBuffer,
    ep3_tx: PlainBuffer,
}

static mut BUFFER: AlignedBuffer = AlignedBuffer {
    ep0: SetupBuffer { bytes: [0; MAX_LEN] },
    ep1_cmd: PlainBuffer { bytes: [0; MAX_LEN] },
    ep2_rx: PlainBuffer { bytes: [0; MAX_LEN] },
    ep3_tx: PlainBuffer { bytes: [0; MAX_LEN] },
};

const RINGBUFFER_SIZE: usize = 32;
struct RingBuffer {
    buffer: [u8; RINGBUFFER_SIZE],
    read_idx: usize,
    write_idx: usize,
}

impl RingBuffer {
    fn push(&mut self, value: u8) -> Result<(), u8> {
        let next_write_idx = (self.write_idx + 1) % RINGBUFFER_SIZE;
        if next_write_idx == self.read_idx {
            return Err(value);
        }
        self.buffer[next_write_idx] = value;
        self.write_idx = next_write_idx;
        Ok(())
    }

    fn pop(&mut self) -> Option<u8> {
        if self.read_idx == self.write_idx {
            return None;
        }
        let value = self.buffer[self.read_idx];
        self.read_idx = (self.read_idx + 1) % RINGBUFFER_SIZE;
        Some(value)
    }

    fn is_full(&mut self) -> bool {
        let next_write_idx = (self.write_idx + 1) % RINGBUFFER_SIZE;
        // magic recipe
        let raw_ptr: *mut usize = &mut self.read_idx;
        unsafe {
            return if next_write_idx == read_volatile(raw_ptr) { true } else { false };
        }
    }
}

static mut RINGBUFFER: RingBuffer = RingBuffer {
    buffer: [0; RINGBUFFER_SIZE],
    read_idx: 0,
    write_idx: 0,
};

const HELLO: [u8; 13] = [
    b'H',
    b'e',
    b'l',
    b'l',
    b'o',
    b' ',
    b'w',
    b'o',
    b'r',
    b'l',
    b'd',
    b'\r',
    b'\n',
];

static mut TX_LENGTH: u8 = 0;

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

    // trigger.set_low().unwrap();
    // delay.delay_ms(5);
    // trigger.set_high().unwrap();
    // delay.delay_ms(10);
    // trigger.set_low().unwrap();

    // critical_section::with(|cs| {
    //     TRIGGER.borrow(cs).replace(Some(trigger));
    // });

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

    led1.set_low().unwrap();
    writeln!(&mut log, "-").unwrap();

    let mut rng = rand::rngs::SmallRng::from_seed([0; 16]);
    loop {
        delay.delay_ms(500);
        unsafe {
            while RINGBUFFER.is_full() {}
            let _ = RINGBUFFER.push(rng.gen::<u8>() / 26 + 65);
            while RINGBUFFER.is_full() {}
            let _ = RINGBUFFER.push(b':');
        }

        for i in 0..HELLO.len() {
            unsafe {
                while RINGBUFFER.is_full() {}
                let _ = RINGBUFFER.push(HELLO[i]);
            }
        }
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
                // FIXME: for full-speed
                // .clear_bit() // 12M full-speed
                .set_bit() // 1.5M low-speed
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
                // FIXME: for full-speed
                // .clear_bit() // 12M full-speed
                .set_bit() // 1.5M low-speed
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
                    .bits(0b0) // ACK
                    .mask_uep_t_res()
                    .bits(0b10) // NAK
        );
        (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(0));

        // setup endpoint 1
        (*USBHD::ptr()).uep1_dma.write(|w| w.bits(BUFFER.ep1_cmd.bytes.as_ptr() as u16));
        (*USBHD::ptr()).uep4_1_mod.modify(|_, w|
            w.uep1_rx_en().clear_bit().uep1_tx_en().set_bit().uep1_buf_mod().clear_bit()
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
                    .bits(0b10) // OUT NAK
                    .mask_uep_t_res()
                    .bits(0b0) // IN ACK
        );
        (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));

        (*USBHD::ptr()).uep2_3_mod__uh_ep_mod.modify(|_, w|
            w
                .uep2_rx_en__uh_ep_rx_en()
                .set_bit()
                .uep2_tx_en()
                .clear_bit()
                .uep2_buf_mod__uh_ep_rbuf_mod()
                .clear_bit()
                .uep3_rx_en()
                .clear_bit()
                .uep3_tx_en__uh_ep_tx_en()
                .set_bit()
                .uep3_buf_mod__uh_ep_tbuf_mod()
                .clear_bit()
        );
        // setup endpoint 2 OUT rx
        (*USBHD::ptr()).uep2_dma__uh_rx_dma.write(|w| w.bits(BUFFER.ep2_rx.bytes.as_ptr() as u16));
        (*USBHD::ptr()).uep2_ctrl__uh_rx_ctrl.modify(
            |_, w|
                w
                    .uep_r_tog__uh_r_tog()
                    .set_bit()
                    .uep_t_tog()
                    .set_bit()
                    .uep_auto_tog__uh_r_auto_tog()
                    .set_bit()
                    .mask_uep_r_res()
                    .bits(0b0) // OUT ACK
                    .mask_uep_t_res()
                    .bits(0b10) // IN NAK
        );

        // setup endpoint 3 IN tx
        (*USBHD::ptr()).uep3_dma__uh_tx_dma.write(|w| w.bits(BUFFER.ep3_tx.bytes.as_ptr() as u16));
        (*USBHD::ptr()).uep3_ctrl__uh_tx_ctrl.modify(
            |_, w|
                w
                    .uep_r_tog()
                    .set_bit()
                    .uep_t_tog()
                    .set_bit()
                    .uep_auto_tog()
                    .set_bit()
                    .mask_uep_r_res()
                    .bits(0b10) // OUT NAK
                    .mask_uep_t_res()
                    .bits(0b0) // IN ACK
        );
        (*USBHD::ptr()).uep3_t_len__uh_tx_len.write(|w| w.bits(0));

        // enable interrupts for USB
        (*USBHD::ptr()).usb_int_en.modify(|_, w|
            w.uie_suspend().set_bit().uie_transfer().set_bit().uie_bus_rst__uie_detect().set_bit()
        );
        (*USBHD::ptr()).usb_int_fg.write(|w| w.bits(0x1f)); // clear all interrupts
        (*PFIC::ptr()).ienr2.modify(|_, w| w.bits(0b1 << ((Interrupt::USBHD as u32) - 32)));
        riscv::interrupt::enable();
    }
}

fn make_trigger() {
    unsafe {
        critical_section::with(|cs| {
            let mut trigger = TRIGGER.borrow(cs).borrow_mut();
            trigger.as_mut().unwrap().toggle().unwrap();
        });
    }
}

interrupt!(USBHD, usb_handler);
fn usb_handler() {
    // BUFFER.ep0.setup.request_type,
    // BUFFER.ep0.setup.request,

    // make_trigger();
    unsafe {
        let int_gf = (*USBHD::ptr()).usb_int_fg.read();
        if int_gf.uif_transfer().bit_is_set() {
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
                                            0x600 => {
                                                // return device qualifier for USB 2.0
                                                // DescIndex::Qualifier
                                                DescIndex::Stalled
                                            }
                                            _ => { DescIndex::Stalled }
                                        };
                                    }
                                    0x81 => {
                                        SETUP_STATE = SetupState::GetDescriptor;
                                        DESC_IDX = match BUFFER.ep0.setup.value {
                                            _ => { DescIndex::Stalled }
                                        };
                                    }
                                    _ => {
                                        DESC_IDX = DescIndex::Stalled;
                                    }
                                }

                                match DESC_IDX {
                                    DescIndex::Stalled => {
                                        SETUP_STATE = SetupState::Stalled;
                                    }
                                    _ => {
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
                                }
                            }
                            0x09 => {
                                // SET_CONFIGURATION
                                // configure to BUFFER.ep0.setup_value
                                if BUFFER.ep0.setup.request_type == 0x00 {
                                    SETUP_STATE = SetupState::Skip;
                                }
                            }
                            0x0a => {
                                if BUFFER.ep0.setup.request_type == 0x21 {
                                    SETUP_STATE = SetupState::Skip;
                                }
                            }
                            0x20 => {
                                // SetLineCoding
                                // make_trigger();

                                if BUFFER.ep0.setup.request_type == 0x21 {
                                    SETUP_STATE = SetupState::Skip;
                                }
                            }
                            // 0x21 => {
                            //     // GetLineCoding
                            //     SETUP_STATE = SetupState::Skip;
                            // }
                            0x22 => {
                                if BUFFER.ep0.setup.request_type == 0x21 {
                                    // SetControlLineState
                                    SETUP_STATE = SetupState::Skip;
                                }
                            }
                            _ => {
                                make_trigger();
                                unreachable!();
                            }
                        }

                        // default lenght is 0 and usless
                        // if SETUP_STATE != SetupState::GetDescriptor {
                        //     len = 0;
                        // }

                        // set transfer length
                        (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(len as u8));

                        match SETUP_STATE {
                            SetupState::Stalled => {
                                (*USBHD::ptr()).uep0_ctrl.modify(
                                    |_, w|
                                        w
                                            .uep_r_tog()
                                            .set_bit()
                                            .uep_t_tog()
                                            .set_bit()
                                            .mask_uep_r_res()
                                            .bits(0b11) // STALL
                                            .mask_uep_t_res()
                                            .bits(0b11) // STALL
                                );
                            }
                            _ => {
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
                                );
                            }
                        }
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
                                SetupState::Skip | SetupState::Stalled => {
                                    // do nothing
                                }
                                _ => {
                                    make_trigger();
                                    unreachable!();
                                }
                            }
                            // set transfer length
                            (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(len as u8));

                            // toggle UEP_R_TOG (bit 7) and UEP_T_TOG (bit 6)
                            (*USBHD::ptr()).uep0_ctrl.modify(|r, w| w.bits(r.bits() ^ 0xc0));
                        }
                        1 => {
                            // CMD IN
                            // set transfer length
                            // reset tx length set in main
                            (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));
                        }
                        3 => {
                            // DATA IN
                            // if (*USBHD::ptr()).uep3_t_len__uh_tx_len.read().bits() > 0 {
                            make_trigger();
                            // }
                            // (*USBHD::ptr()).uep3_t_len__uh_tx_len.write(|w| w.bits(0));

                            // FIXME: mean lag is 1.5 interval.
                            let mut count = 0;
                            for i in 0..MAX_LEN as usize {
                                match RINGBUFFER.pop() {
                                    Some(val) => {
                                        BUFFER.ep3_tx.bytes[i] = val;
                                        count += 1;
                                    }
                                    None => {
                                        break;
                                    }
                                }
                            }
                            (*USBHD::ptr()).uep3_t_len__uh_tx_len.write(|w| w.bits(count as u16));
                        }
                        _ => {
                            make_trigger();
                            unreachable!();
                        }
                    }
                }
                0b00 => {
                    // OUT
                    match int_endp {
                        0 => {
                            // just ACK
                        }
                        2 => {
                            // DATA OUT
                            make_trigger();
                        }
                        _ => {
                            make_trigger();
                            unreachable!();
                        }
                    }
                }
                _ => {
                    // unhandled
                    make_trigger();
                    unreachable!();
                }
            }

            // clear USB_INT_FG
            // clearing USB_INT_GG makes ACK for transaction
            (*USBHD::ptr()).usb_int_fg.modify(|_, w| w.uif_transfer().set_bit())
        } else if int_gf.uif_suspend().bit_is_set() {
            // make_trigger();

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