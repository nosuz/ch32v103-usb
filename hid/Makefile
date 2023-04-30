PKG_NAME=ch32v103_usb_hid
BIN_NAME=firmware.bin

all: release

release:
	cargo build --release && \
	riscv64-unknown-elf-objcopy -O binary target/riscv32imac-unknown-none-elf/release/${PKG_NAME} ${BIN_NAME}

flash: release
	wchisp flash ${BIN_NAME}

clean:
	rm -r target ${BIN_NAME}
