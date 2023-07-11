use ch32v1::ch32v103::{ USBHD };
use ch32v103_hal::prelude::OutputPin;

use crate::usb::handler::BUFFER;
use crate::KEY_BUFFER;
use crate::NUMLOCK;

pub enum KeyModifier {
    None = 0,
    LeftCtrl = 0x01,
    LeftShift = 0x02,
    LeftAlt = 0x04,
    LeftGui = 0x08,
    RightCtrl = 0x10,
    RightShift = 0x20,
    RightAlt = 0x40,
    RightGui = 0x80,
}

#[derive(Copy, Clone)]
pub struct KeyStatus {
    pub modifier: u8,
    pub code: u8,
}

#[repr(C)]
#[derive(Copy, Clone)]
struct KeyboardData {
    modifier: u8,
    _reserved: u8,
    key1: u8,
    key2: u8,
    key3: u8,
    key4: u8,
    key5: u8,
    key6: u8,
}

pub union KeyboardBuffer {
    pub bytes: [u8; 64],
    keyboard: KeyboardData,
}

enum LedStatus {
    NumLock = 0x01,
    CapsLock = 0x02,
    ScrollLock = 0x04,
    Compose = 0x08,
    Kana = 0x10,
}

pub fn init_keyboard() {
    unsafe {
        BUFFER.ep1.keyboard.modifier = 0;
        BUFFER.ep1.keyboard._reserved = 0;
        BUFFER.ep1.keyboard.key1 = 0;
        BUFFER.ep1.keyboard.key2 = 0;
        BUFFER.ep1.keyboard.key3 = 0;
        BUFFER.ep1.keyboard.key4 = 0;
        BUFFER.ep1.keyboard.key5 = 0;
        BUFFER.ep1.keyboard.key6 = 0;
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

pub fn ep1_in() {
    // DATA IN
    unsafe {
        match KEY_BUFFER.pop() {
            Some(val) => {
                BUFFER.ep1.keyboard.modifier = val.modifier;
                BUFFER.ep1.keyboard.key1 = val.code;
                (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(8));
            }
            None => {
                (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));
            }
        }
    }
}

pub fn update_keyboard_led(led_status: u8) {
    critical_section::with(|cs| {
        let mut num_lock = NUMLOCK.borrow(cs).borrow_mut();
        if (led_status & (LedStatus::NumLock as u8)) == 0 {
            num_lock.as_mut().unwrap().set_low().unwrap();
        } else {
            num_lock.as_mut().unwrap().set_high().unwrap();
        }
    });
}

pub fn ascii_to_usb_keycode(ascii_code: u8) -> Option<KeyStatus> {
    // returns JP scan codes.
    // http://hp.vector.co.jp/authors/VA003720/lpproj/others/kbdjpn.htm
    match ascii_code {
        b'a'..=b'z' =>
            Some(KeyStatus { modifier: KeyModifier::None as u8, code: ascii_code - b'a' + 4 }), // Convert lowercase letters (a-z)
        b'A'..=b'Z' =>
            Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: ascii_code - b'A' + 4 }), // Convert uppercase letters (A-Z)
        b'1'..=b'9' =>
            Some(KeyStatus { modifier: KeyModifier::None as u8, code: ascii_code - b'1' + 30 }), // Convert numeric digits (1-9)
        b'0' => Some(KeyStatus { modifier: KeyModifier::None as u8, code: 39 }), // Convert digit 0
        b'\n' => Some(KeyStatus { modifier: KeyModifier::None as u8, code: 40 }), // Convert newline character
        b' ' => Some(KeyStatus { modifier: KeyModifier::None as u8, code: 44 }), // Convert space character
        b'!' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 30 }),
        b'"' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 31 }),
        b'#' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 32 }),
        b'$' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 33 }),
        b'%' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 34 }),
        b'&' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 35 }),
        b'\'' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 36 }),
        b'(' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 37 }),
        b')' => Some(KeyStatus { modifier: KeyModifier::LeftShift as u8, code: 38 }),
        b'@' => Some(KeyStatus { modifier: KeyModifier::None as u8, code: 47 }),
        _ => None, // Ignore other characters
    }
}
