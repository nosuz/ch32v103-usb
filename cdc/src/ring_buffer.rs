#[macro_export]
macro_rules! push_until_ok {
    ($buffer:expr, $value:expr) => {
        loop {
            let result = unsafe {$buffer.push($value)};
            if result == Ok(()) {
                break;
            }
        }
    };
}

use core::sync::atomic::{ AtomicUsize, Ordering };

use crate::usb::handler::MAX_LEN;
pub const RING_BUFFER_SIZE: usize = MAX_LEN * 4;

// Thank you, chatGPT. But some fixes is required.
pub struct RingBuffer {
    pub buffer: [u8; RING_BUFFER_SIZE],
    pub write_pos: AtomicUsize,
    pub read_pos: AtomicUsize,
}

impl RingBuffer {
    fn next_pos(&self, pos: usize) -> usize {
        (pos + 1) % RING_BUFFER_SIZE
    }

    pub fn push(&mut self, item: u8) -> Result<(), u8> {
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let read_pos = self.read_pos.load(Ordering::Relaxed);

        let next_write_pos = self.next_pos(write_pos);

        if next_write_pos == read_pos {
            return Err(item);
        }

        // write_pos is OK.
        self.buffer[write_pos] = item;

        self.write_pos.store(next_write_pos, Ordering::Release);

        Ok(())
    }

    pub fn pop(&self) -> Option<u8> {
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Relaxed);

        if read_pos == write_pos {
            return None;
        }

        let item = unsafe { self.buffer[read_pos] };

        self.read_pos.store(self.next_pos(read_pos), Ordering::Release);

        Some(item)
    }

    pub fn is_full(&self) -> bool {
        let write_pos = self.write_pos.load(Ordering::Relaxed);
        let next_write_pos = self.next_pos(write_pos);
        next_write_pos == self.read_pos.load(Ordering::Acquire)
    }

    pub fn space(&self) -> usize {
        let write_pos = self.write_pos.load(Ordering::Relaxed);
        let read_pos = self.read_pos.load(Ordering::Relaxed);
        (read_pos + RING_BUFFER_SIZE - write_pos - 1) % RING_BUFFER_SIZE
    }
}