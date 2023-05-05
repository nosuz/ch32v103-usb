use core::sync::atomic::{ AtomicUsize, Ordering };
use core::ptr::{ read_volatile, write_volatile };

pub const RINGBUFFER_SIZE: usize = 32;

// Thank you, chatGPT. But some fixes is required.
pub struct RingBuffer {
    pub buffer: [u8; RINGBUFFER_SIZE],
    pub write_index: AtomicUsize,
    pub read_index: AtomicUsize,
}

impl RingBuffer {
    fn next_index(&self, index: usize) -> usize {
        (index + 1) % RINGBUFFER_SIZE
    }

    pub fn push(&mut self, item: u8) -> Result<(), u8> {
        let write_index = self.write_index.load(Ordering::Acquire);
        let read_index = self.read_index.load(Ordering::Relaxed);

        let next_write_index = self.next_index(write_index);

        if next_write_index == read_index {
            return Err(item);
        }

        // write_index is OK.
        let raw_ptr: *mut u8 = &mut self.buffer[write_index];
        unsafe {
            write_volatile(raw_ptr, item);
        }

        self.write_index.store(next_write_index, Ordering::Release);

        Ok(())
    }

    pub fn pop(&self) -> Option<u8> {
        let read_index = self.read_index.load(Ordering::Acquire);
        let write_index = self.write_index.load(Ordering::Relaxed);

        if read_index == write_index {
            return None;
        }

        let item = unsafe { read_volatile(&self.buffer[read_index]) };

        self.read_index.store(self.next_index(read_index), Ordering::Release);

        Some(item)
    }

    pub fn is_full(&self) -> bool {
        let write_index = self.write_index.load(Ordering::Relaxed);
        let next_write_index = self.next_index(write_index);
        next_write_index == self.read_index.load(Ordering::Acquire)
    }

    // TODO: check
    pub fn space(&self) -> usize {
        let write_index = self.write_index.load(Ordering::Relaxed);
        let read_index = self.read_index.load(Ordering::Relaxed);
        let count = read_index.wrapping_sub(write_index + 1);
        if write_index >= read_index {
            self.buffer.len() - count
        } else {
            count
        }
    }
}