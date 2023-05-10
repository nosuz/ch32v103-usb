use ch32v1::ch32v103::{ USBHD };

use crate::usb::handler::{ BUFFER, MAX_LEN };
use crate::{ TX_BUFFER, RX_BUFFER, SPEED_BUFFER };

static mut WAIT_DATA_OUT: bool = false;

pub fn init_usb_serial() {}

pub fn setup_endpoints() {
    // setup endpoint 1
    unsafe {
        (*USBHD::ptr()).uep1_dma.write(|w| w.bits(BUFFER.ep1_cmd.as_ptr() as u16));
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
        (*USBHD::ptr()).uep2_dma__uh_rx_dma.write(|w| w.bits(BUFFER.ep2_rx.as_ptr() as u16));
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
        (*USBHD::ptr()).uep3_dma__uh_tx_dma.write(|w| w.bits(BUFFER.ep3_tx.as_ptr() as u16));
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
    }
}

pub fn cdc_command_in() {
    // CMD IN
    unsafe {
        // set transfer length
        (*USBHD::ptr()).uep1_t_len.write(|w| w.bits(0));
    }
}

pub fn cdc_data_out() {
    // DATA OUT
    unsafe {
        for i in 0..(*USBHD::ptr()).usb_rx_len.read().bits() as usize {
            let _ = TX_BUFFER.push(BUFFER.ep2_rx[i]);
        }

        if TX_BUFFER.space() < MAX_LEN * 2 {
            // Nak next for wait and retry
            (*USBHD::ptr()).uep2_ctrl__uh_rx_ctrl.modify(
                |_, w| w.mask_uep_r_res().bits(0b10) // OUT NAK
            );
            WAIT_DATA_OUT = true;
        }
    }
}

pub fn cdc_data_in() {
    // DATA IN
    let mut count = 0;
    unsafe {
        for i in 0..MAX_LEN as usize {
            match RX_BUFFER.pop() {
                Some(val) => {
                    BUFFER.ep3_tx[i] = val;
                    count += 1;
                }
                None => {
                    break;
                }
            }
        }
        (*USBHD::ptr()).uep3_t_len__uh_tx_len.write(|w| w.bits(count as u16));

        if WAIT_DATA_OUT & (TX_BUFFER.space() >= MAX_LEN * 2) {
            // Ack next
            (*USBHD::ptr()).uep2_ctrl__uh_rx_ctrl.modify(
                |_, w| w.mask_uep_r_res().bits(0b0) // OUT ACK
            );
            WAIT_DATA_OUT = false;
        }
    }
}

pub fn setup_serial() {
    unsafe {
        let _ = SPEED_BUFFER.push(BUFFER.ep0.serial_config.speed);
    }
}

pub fn set_control_lines() {}