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

pub struct RingBuffer<T, const SIZE: usize> {
    buffer: [Option<T>; SIZE],
    read_index: usize,
    write_index: usize,
}

impl<T: core::marker::Copy, const SIZE: usize> RingBuffer<T, SIZE> {
    pub const fn new() -> Self {
        Self {
            buffer: [None; SIZE],
            read_index: 0,
            write_index: 0,
        }
    }

    pub fn push(&mut self, item: T) -> Result<(), ()> {
        let next_write_index = (self.write_index + 1) % SIZE;

        if next_write_index == self.read_index {
            Err(()) // Buffer is full
        } else {
            self.buffer[self.write_index] = Some(item);
            self.write_index = next_write_index;
            Ok(())
        }
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.read_index == self.write_index {
            None // Buffer is empty
        } else {
            let item = self.buffer[self.read_index].take();
            self.read_index = (self.read_index + 1) % SIZE;
            item
        }
    }

    pub fn is_empty(&self) -> bool {
        self.read_index == self.write_index
    }

    pub fn is_full(&self) -> bool {
        (self.write_index + 1) % SIZE == self.read_index
    }

    pub fn len(&self) -> usize {
        (SIZE + self.write_index - self.read_index) % SIZE
    }

    pub fn capacity(&self) -> usize {
        SIZE
    }
}
