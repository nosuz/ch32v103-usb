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

use core::mem::MaybeUninit;
use core::sync::atomic::{ AtomicUsize, Ordering };

// Thank you, chatGPT. But some fixes is required.
pub struct RingBuffer<T, const N: usize> {
    buffer: [MaybeUninit<T>; N],
    write_pos: AtomicUsize,
    read_pos: AtomicUsize,
}

impl<T, const N: usize> RingBuffer<T, N> {
    pub const fn new() -> Self {
        Self {
            buffer: unsafe {
                MaybeUninit::uninit().assume_init()
            },
            write_pos: AtomicUsize::new(0),
            read_pos: AtomicUsize::new(0),
        }
    }
    fn next_pos(&self, pos: usize) -> usize {
        (pos + 1) % N
    }

    pub fn push(&mut self, item: T) -> Result<(), T> {
        let write_pos = self.write_pos.load(Ordering::Acquire);
        let read_pos = self.read_pos.load(Ordering::Relaxed);

        let next_write_pos = self.next_pos(write_pos);

        if next_write_pos == read_pos {
            return Err(item);
        }

        // write_pos is OK.
        unsafe {
            self.buffer[write_pos].as_mut_ptr().write(item);
        }

        self.write_pos.store(next_write_pos, Ordering::Release);

        Ok(())
    }

    pub fn pop(&self) -> Option<T> {
        let read_pos = self.read_pos.load(Ordering::Acquire);
        let write_pos = self.write_pos.load(Ordering::Relaxed);

        if read_pos == write_pos {
            return None;
        }

        let item = unsafe { self.buffer[read_pos].as_ptr().read() };

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
        (read_pos + N - write_pos - 1) % N
    }
}