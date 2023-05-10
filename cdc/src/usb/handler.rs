use ch32v1::ch32v103::{ RCC, PFIC, USBHD };
use ch32v1::ch32v103::interrupt::Interrupt;

use ch32v103_hal::prelude::ToggleableOutputPin;
use crate::TRIGGER;

use crate::usb::descriptor::{ USB_DESC, DescIndex };
use crate::usb::usb_serial::{
    init_usb_serial,
    setup_endpoints,
    cdc_command_in,
    cdc_data_in,
    cdc_data_out,
    setup_serial,
    set_control_lines,
};

static mut DESC_IDX: DescIndex = DescIndex::Stalled;

pub const MAX_LEN: usize = if cfg!(feature = "full_speed") { 64 } else { 8 };

static mut REQ_LEN: usize = MAX_LEN;
static mut LAST_P: usize = 0; // end of descriptor
static mut BUF_P: usize = 0;

enum RequestType {
    Standard,
    Class,
    Vendor,
    Reserved,
}

#[derive(Copy, Clone)]
enum Request {
    GetStatus = 0,
    ClearFeature = 1,
    Reserved1 = 2,
    SetFeature = 3,
    Reserved2 = 4,
    SetAddress = 5,
    GetDescriptor = 6,
    SetDescriptor = 7,
    GetConfiguration = 8,
    SetConfiguration = 9,
    GetInterface = 10,
    SetInterface = 11,
    SyncFrame = 12,

    // CDC class specific
    SetLineCoding = 0x20,
    GetLineCoding = 0x21,
    SetControlLineState = 0x22,
}

enum UsbState {
    Ack,
    Nak,
    Stalled,
    GetDescriptor,
    SetAddress,
    SetConfiguration,
    // CDC class specific
    SetLineCoding,
}
static mut USB_STATE: UsbState = UsbState::Ack;

#[repr(C)]
#[derive(Copy, Clone)]
struct SetupData {
    request_type: u8,
    request: Request,
    value: u16,
    index: u16,
    length: u16,
}

#[repr(C)]
#[derive(Copy, Clone)]
pub struct SerialConfigData {
    pub speed: u32,
    pub stop_bits: u8,
    pub parity_type: u8,
    pub data_bits: u8,
}

pub union SetupBuffer {
    bytes: [u8; 64],
    setup: SetupData,
    pub serial_config: SerialConfigData,
}

#[repr(C, align(4))]
pub struct AlignedBuffer {
    pub ep0: SetupBuffer,
    pub ep1_cmd: [u8; MAX_LEN],
    pub ep2_rx: [u8; MAX_LEN],
    pub ep3_tx: [u8; MAX_LEN],
}

pub static mut BUFFER: AlignedBuffer = AlignedBuffer {
    ep0: SetupBuffer { bytes: [0; 64] },
    ep1_cmd: [0; MAX_LEN],
    ep2_rx: [0; MAX_LEN],
    ep3_tx: [0; MAX_LEN],
};

pub fn init_usb() {
    init_usb_serial();

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

        (*USBHD::ptr()).usb_ctrl.modify(|_, w|
            w
                .uc_host_mode()
                .clear_bit() // set device mode
                .uc_low_speed()
                .bit(cfg!(not(feature = "full_speed"))) // true: 1.5M low-speed, false: 12M full-speed
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
                .bit(cfg!(not(feature = "full_speed"))) // true: 1.5M low-speed, false: 12M full-speed
                .ud_port_en__uh_port_en()
                .set_bit()
        );

        setup_ep0();

        // enable interrupts for USB
        (*USBHD::ptr()).usb_int_en.modify(|_, w|
            w.uie_suspend().set_bit().uie_transfer().set_bit().uie_bus_rst__uie_detect().set_bit()
        );
        (*USBHD::ptr()).usb_int_fg.write(|w| w.bits(0x1f)); // clear all interrupts
        (*PFIC::ptr()).ienr2.modify(|_, w| w.bits(0b1 << ((Interrupt::USBHD as u32) - 32)));
        riscv::interrupt::enable();
    }
}

fn setup_ep0() {
    unsafe {
        // reset USB device address
        (*USBHD::ptr()).usb_dev_ad.write(|w| w.bits(0));

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
    }
}

fn get_request_type(bm_request_type: u8) -> RequestType {
    match bm_request_type & 0b0110_0000 {
        0b0000_0000 => { RequestType::Standard }
        0b0010_0000 => { RequestType::Class }
        0b0100_0000 => { RequestType::Vendor }
        0b0110_0000 => { RequestType::Reserved }
        _ => { RequestType::Reserved }
    }
}

pub fn usb_interrupt_handler() {
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
            match int_st.mask_uis_h_res__mask_uis_endp().bits() {
                0 => {
                    // endpoint 0
                    match int_st.mask_uis_token().bits() {
                        0b11 => {
                            // SETUP
                            match get_request_type(BUFFER.ep0.setup.request_type) {
                                RequestType::Standard => {
                                    match BUFFER.ep0.setup.request {
                                        Request::GetDescriptor => {
                                            // GET_DESCRIPTOR
                                            USB_STATE = UsbState::GetDescriptor;
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
                                                    // return STALL for qualifier if the device is full-speed
                                                    DescIndex::Stalled
                                                }
                                                _ => { DescIndex::Stalled }
                                            };

                                            match DESC_IDX {
                                                DescIndex::Stalled => {
                                                    USB_STATE = UsbState::Stalled;
                                                }
                                                _ => {
                                                    REQ_LEN = BUFFER.ep0.setup.length as usize;

                                                    // reset pointer
                                                    BUF_P = 0;
                                                    // set max data size
                                                    LAST_P = USB_DESC[
                                                        DESC_IDX as usize
                                                    ].len() as usize;
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
                                                            BUFFER.ep0.bytes[i] = USB_DESC[
                                                                DESC_IDX as usize
                                                            ][BUF_P + i];
                                                        }
                                                        // update pointer
                                                        BUF_P += len;
                                                    }
                                                }
                                            }
                                        }
                                        Request::SetAddress => {
                                            // SET_ADDRESS
                                            USB_STATE = UsbState::SetAddress;
                                        }
                                        Request::SetConfiguration => {
                                            // SET_CONFIGURATION
                                            USB_STATE = UsbState::SetConfiguration;
                                            setup_endpoints();
                                        }
                                        _ => {
                                            unreachable!();
                                        }
                                    }
                                }
                                RequestType::Class => {
                                    match BUFFER.ep0.setup.request {
                                        Request::GetInterface => {
                                            USB_STATE = UsbState::Ack;
                                        }

                                        // CDC class specific requets
                                        Request::SetLineCoding => {
                                            // SetLineCoding = 0x20
                                            USB_STATE = UsbState::SetLineCoding;
                                            // make_trigger();
                                        }
                                        // Request::GetLineCoding => {
                                        //     // GetLineCoding = 0x21
                                        // }
                                        Request::SetControlLineState => {
                                            // SetControlLineState = 0x22
                                            USB_STATE = UsbState::Ack;
                                            // make_trigger();
                                            set_control_lines();
                                        }
                                        _ => {
                                            unreachable!();
                                        }
                                    }
                                }
                                RequestType::Vendor => {}
                                _ => {
                                    unreachable!();
                                }
                            }

                            // set transfer length
                            (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(len as u8));

                            let res = match USB_STATE {
                                UsbState::Nak => { 0b10 } // NAK
                                UsbState::Stalled => { 0b11 } // STALL
                                _ => { 0 }
                            };
                            (*USBHD::ptr()).uep0_ctrl.modify(|_, w|
                                w
                                    .uep_r_tog()
                                    .set_bit()
                                    .uep_t_tog()
                                    .set_bit()
                                    .mask_uep_r_res()
                                    .bits(res)
                                    .mask_uep_t_res()
                                    .bits(res)
                            );
                        }
                        0b10 => {
                            // IN
                            match USB_STATE {
                                UsbState::SetAddress => {
                                    (*USBHD::ptr()).usb_dev_ad.write(|w|
                                        w.bits(BUFFER.ep0.setup.value as u8)
                                    );
                                }
                                UsbState::GetDescriptor => {
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
                                    // do nothing
                                }
                            }
                            // set transfer length
                            (*USBHD::ptr()).uep0_t_len.write(|w| w.bits(len as u8));

                            // toggle UEP_R_TOG (bit 7) and UEP_T_TOG (bit 6)
                            (*USBHD::ptr()).uep0_ctrl.modify(|r, w| w.bits(r.bits() ^ 0xc0));
                        }
                        0b00 => {
                            // OUT
                            match USB_STATE {
                                UsbState::SetLineCoding => {
                                    setup_serial();
                                }
                                _ => {
                                    // ACK comes here
                                }
                            }
                        }
                        _ => {
                            unreachable!();
                        }
                    }
                }
                1 => {
                    // endpoint 1
                    match int_st.mask_uis_token().bits() {
                        0b11 => {
                            // SETUP
                            unreachable!();
                        }
                        0b10 => {
                            // IN
                            cdc_command_in();
                        }
                        0b00 => {
                            // OUT
                            unreachable!();
                        }
                        _ => {
                            unreachable!();
                        }
                    }
                }
                2 => {
                    // endpoint 2
                    match int_st.mask_uis_token().bits() {
                        0b11 => {
                            // SETUP
                            unreachable!();
                        }
                        0b10 => {
                            // IN
                            unreachable!();
                        }
                        0b00 => {
                            // OUT
                            // make_trigger();
                            cdc_data_out();
                        }
                        _ => {
                            unreachable!();
                        }
                    }
                }
                3 => {
                    // endpoint 3
                    match int_st.mask_uis_token().bits() {
                        0b11 => {
                            // SETUP
                            unreachable!();
                        }
                        0b10 => {
                            // IN
                            cdc_data_in();
                        }
                        0b00 => {
                            // OUT
                            unreachable!();
                        }
                        _ => {
                            unreachable!();
                        }
                    }
                }
                _ => {
                    unreachable!();
                }
            }
            (*USBHD::ptr()).usb_int_fg.modify(|_, w| w.uif_transfer().set_bit())
        } else if int_gf.uif_suspend().bit_is_set() {
            critical_section::with(|cs| {
                let mut trigger = TRIGGER.borrow(cs).borrow_mut();
                trigger.as_mut().unwrap().toggle().unwrap();
            });

            (*USBHD::ptr()).usb_int_fg.modify(|_, w| w.uif_suspend().set_bit());
            if (*USBHD::ptr()).usb_mis_st.read().ums_suspend().bit_is_set() {
                (*USBHD::ptr()).usb_int_fg.write(|w| w.bits(0x1f)); // clear all interrupts
            }
        } else if int_gf.uif_bus_rst__uif_detect().bit_is_set() {
            USB_STATE = UsbState::Ack;
            setup_ep0();
        }
    }
}

fn make_trigger() {
    critical_section::with(|cs| {
        let mut trigger = TRIGGER.borrow(cs).borrow_mut();
        trigger.as_mut().unwrap().toggle().unwrap();
    });
}