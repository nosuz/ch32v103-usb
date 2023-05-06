use ch32v1::ch32v103::{ USBHD };

use crate::usb::handler::BUFFER;

pub struct MouseStatus {
    pub right_button: bool,
    pub middle_button: bool,
    pub left_button: bool,
    pub x_axis: i8,
    pub y_axis: i8,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct MouseData {
    button: u8,
    x: i8,
    y: i8,
    z: i8,
}

pub union MouseBuffer {
    pub bytes: [u8; 64],
    mouse: MouseData,
}

pub fn init_mouse() {
    unsafe {
        BUFFER.ep1.mouse.x = 0;
        BUFFER.ep1.mouse.y = 0;
    }
}

pub fn setup_ep1() {
    // setup endpoint 1
    unsafe {
        (*USBHD::ptr()).uep1_dma.write(|w| w.bits(BUFFER.ep1.bytes.as_ptr() as u16));
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
                    .bits(0b10) // NAK
                    .mask_uep_t_res()
                    .bits(0b0) // ACK
        );
        (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));
    }
}

pub fn update_mouse(mouse_status: &MouseStatus) {
    // wait USB bus idle
    unsafe {
        while !(*USBHD::ptr()).usb_mis_st.read().ums_sie_free().bit_is_set() {}
        BUFFER.ep1.mouse.x = mouse_status.x_axis;
        BUFFER.ep1.mouse.y = mouse_status.y_axis;
        (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(3));
    }
}

pub fn ep1_in() {
    // reset tx length
    unsafe {
        (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));
    }
}