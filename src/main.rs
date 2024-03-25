#![no_std]
#![no_main]

use core::{fmt::Debug, mem::MaybeUninit};

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use rp_pico as bsp;

use bsp::hal::{clocks::init_clocks_and_plls, pac, sio::Sio, watchdog::Watchdog};

use pico_usb_mass_storage::usbd_storage::{
    subclass::{
        scsi::{Scsi, ScsiCommand},
        Command,
    },
    transport::{
        bbb::{BulkOnly, BulkOnlyError},
        TransportError,
    },
};
use usb_device::{class_prelude::*, prelude::*};

/// Not necessarily `'static`. May reside in some special memory location
static mut USB_TRANSPORT_BUF: MaybeUninit<[u8; 512]> = MaybeUninit::uninit();
static mut STORAGE: [u8; (BLOCKS * BLOCK_SIZE) as usize] = [0u8; (BLOCK_SIZE * BLOCKS) as usize];
static mut UNLOGGED_WRITE: bool = false;

static mut STATE: State = State {
    storage_offset: 0,
    sense_key: None,
    sense_key_code: None,
    sense_qualifier: None,
};

const BLOCK_SIZE: u32 = 512;
const BLOCKS: u32 = 200;
const USB_PACKET_SIZE: u16 = 64; // 8,16,32,64
const MAX_LUN: u8 = 0; // max 0x0F

#[derive(Clone, Default)]
struct State {
    storage_offset: usize,
    sense_key: Option<u8>,
    sense_key_code: Option<u8>,
    sense_qualifier: Option<u8>,
}

impl State {
    fn reset(&mut self) {
        *self = Self::default();
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // work around errata 5
    let sio = Sio::new(pac.SIO);
    let _pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    defmt::timestamp!(
        "{=u32}",
        unsafe { &*pac::TIMER::PTR }.timerawl.read().bits()
    );

    let usb_bus = UsbBusAllocator::new(bsp::hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let mut scsi = pico_usb_mass_storage::usbd_storage::subclass::scsi::Scsi::new(
        &usb_bus,
        USB_PACKET_SIZE,
        MAX_LUN,
        unsafe { USB_TRANSPORT_BUF.assume_init_mut().as_mut_slice() },
    )
    .unwrap();

    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabcd, 0xabcd))
        .manufacturer("Chris Price")
        .product("100k of your finest bytes")
        .serial_number("CP4096OYFB")
        .self_powered(false)
        .build();

    loop {
        if !usb_device.poll(&mut [&mut scsi]) {
            unsafe {
                if UNLOGGED_WRITE {
                    log_storage();
                    UNLOGGED_WRITE = false;
                }
            }
            continue;
        }

        // clear state if just configured or reset
        if matches!(usb_device.state(), UsbDeviceState::Default) {
            unsafe {
                STATE.reset();
            };
        }

        let _ = scsi.poll(|command| {
            if let Err(err) = process_command(command) {
                error!("{}", err);
            }
        });
    }
}

fn process_command(
    mut command: Command<ScsiCommand, Scsi<BulkOnly<bsp::hal::usb::UsbBus, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    match command.kind {
        ScsiCommand::TestUnitReady { .. } => {
            command.pass();
        }
        ScsiCommand::Inquiry { .. } => {
            command.try_write_data_all(&[
                0x00, // periph qualifier, periph device type
                0x80, // Removable
                0x04, // SPC-2 compliance
                0x02, // NormACA, HiSu, Response data format
                0x20, // 36 bytes in total
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                b'C', b'H', b'R', b'I', b'S', b'P', b' ', b' ', // 8-byte T-10 vendor id
                b'1', b'0', b'0', b'k', b' ', b'o', b'f', b' ', b'y', b'o', b'u', b'r', b' ', b'f',
                b'i', b'n', // 16-byte product identification
                b'1', b'.', b'2', b'3', // 4-byte product revision
            ])?;
            command.pass();
        }
        ScsiCommand::RequestSense { .. } => unsafe {
            command.try_write_data_all(&[
                0x70,                         // RESPONSE CODE. Set to 70h for information on current errors
                0x00,                         // obsolete
                STATE.sense_key.unwrap_or(0), // Bits 3..0: SENSE KEY. Contains information describing the error.
                0x00,
                0x00,
                0x00,
                0x00, // INFORMATION. Device-specific or command-specific information.
                0x00, // ADDITIONAL SENSE LENGTH.
                0x00,
                0x00,
                0x00,
                0x00,                               // COMMAND-SPECIFIC INFORMATION
                STATE.sense_key_code.unwrap_or(0),  // ASC
                STATE.sense_qualifier.unwrap_or(0), // ASCQ
                0x00,
                0x00,
                0x00,
                0x00,
            ])?;
            STATE.reset();
            command.pass();
        },
        ScsiCommand::ReadCapacity10 { .. } => {
            let mut data = [0u8; 8];
            let _ = &mut data[0..4].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadCapacity16 { .. } => {
            let mut data = [0u8; 16];
            let _ = &mut data[0..8].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            let _ = &mut data[8..12].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadFormatCapacities { .. } => {
            let mut data = [0u8; 12];
            let _ = &mut data[0..4].copy_from_slice(&[
                0x00, 0x00, 0x00, 0x08, // capacity list length
            ]);
            let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCKS)); // number of blocks
            data[8] = 0x01; //unformatted media
            let block_length_be = u32::to_be_bytes(BLOCK_SIZE);
            data[9] = block_length_be[1];
            data[10] = block_length_be[2];
            data[11] = block_length_be[3];

            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::Read { lba, len } => unsafe {
            let lba = lba as u32;
            let len = len as u32;
            if STATE.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + STATE.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;

                // Uncomment this in order to push data in chunks smaller than a USB packet.
                // let end = min(start + USB_PACKET_SIZE as usize - 1, end);

                let count = command.write_data(&STORAGE[start..end])?;
                STATE.storage_offset += count;
            } else {
                command.pass();
                STATE.storage_offset = 0;
            }
        },
        ScsiCommand::Write { lba, len } => unsafe {
            let lba = lba as u32;
            let len = len as u32;
            if STATE.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + STATE.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;
                let count = command.read_data(&mut STORAGE[start..end])?;
                STATE.storage_offset += count;

                UNLOGGED_WRITE = true;

                if STATE.storage_offset == (len * BLOCK_SIZE) as usize {
                    command.pass();
                    STATE.storage_offset = 0;
                }
            } else {
                command.pass();
                STATE.storage_offset = 0;
            }
        },
        ScsiCommand::ModeSense6 { .. } => {
            command.try_write_data_all(&[
                0x03, // number of bytes that follow
                0x00, // the media type is SBC
                0x00, // not write-protected, no cache-control bytes support
                0x00, // no mode-parameter block descriptors
            ])?;
            command.pass();
        }
        ScsiCommand::ModeSense10 { .. } => {
            command.try_write_data_all(&[0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?;
            command.pass();
        }
        ref unknown_scsi_kind => {
            error!("Unknown SCSI command: {}", unknown_scsi_kind);
            unsafe {
                STATE.sense_key.replace(0x05); // illegal request Sense Key
                STATE.sense_key_code.replace(0x20); // Invalid command operation ASC
                STATE.sense_qualifier.replace(0x00); // Invalid command operation ASCQ
            }
            command.fail();
        }
    }

    Ok(())
}

fn log_storage() {
    for i in 0..4 {
        info!(
            "partition {}: {}",
            i,
            defmt::Debug2Format(&read_partition(i))
        );
    }

    // let digits = b"0123456789ABCDEF";
    // let mut bytes = [b' '; 16 * 3];
    // let mut chars = [b' '; 16];
    // let total_bytes = (BLOCKS * BLOCK_SIZE) as usize;
    // for row_start in (0..total_bytes).step_by(16) {
    //     let mut all_zero = true;
    //     for i in 0..16_usize {
    //         if row_start + i < total_bytes {
    //             let cur_byte = unsafe { STORAGE[row_start + i] };
    //             if cur_byte != 0 {
    //                 all_zero = false;
    //             }
    //             bytes[i * 3 + 0] = digits[((cur_byte / 16) & 0x0f) as usize];
    //             bytes[i * 3 + 1] = digits[(cur_byte & 0x0f) as usize];
    //             bytes[i * 3 + 2] = b' ';
    //             chars[i] = sanitize_byte(cur_byte) as u8;
    //         } else {
    //             bytes[i * 3 + 0] = b' ';
    //             bytes[i * 3 + 1] = b' ';
    //             bytes[i * 3 + 2] = b' ';
    //             chars[i] = b' ';
    //         }
    //     }
    //     if !all_zero {
    //         let bytes: &str = unsafe { core::mem::transmute(bytes.as_slice()) };
    //         let chars: &str = unsafe { core::mem::transmute(chars.as_slice()) };
    //         info!("{} @ {}: {}....{}", kind, row_start, bytes, chars);
    //     }
    // }
}

pub fn sanitize_byte(byte: u8) -> char {
    if (0x20..0x7f).contains(&byte) {
        byte as char
    } else {
        '.'
    }
}

#[derive(Clone)]
pub struct Partition {
    /// Partition Status
    pub p_status: u8,
    /// Start cylinder (Legacy CHS)
    pub p_cyl_begin: u8,
    /// Start head (Legacy CHS)
    pub p_head_begin: u8,
    /// Start sector (Legacy CHS)
    pub p_sect_begin: u8,
    /// Partition Type (DOS, Windows, BeOS, etc)
    pub p_type: u8,
    /// End cylinder (Legacy CHS)
    pub p_cyl_end: u8,
    /// End head (Legacy CHS)
    pub p_head_end: u8,
    /// End sector
    pub p_sect_end: u8,
    /// Logical block address to start of partition
    pub p_lba: u32,
    /// Number of sectors in partition
    pub p_size: u32,
}
impl Debug for Partition {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Partition")
            .field("p_type", &format_args!("{:#02x}", self.p_type))
            .field("p_status", &format_args!("{:#02x}", self.p_status))
            .field("p_cyl_begin", &format_args!("{:#02x}", self.p_cyl_begin))
            .field("p_cyl_end", &format_args!("{:#02x}", self.p_cyl_end))
            .field("p_head_begin", &format_args!("{:#02x}", self.p_head_begin))
            .field("p_head_end", &format_args!("{:#02x}", self.p_head_end))
            .field("p_sect_begin", &format_args!("{:#02x}", self.p_sect_begin))
            .field("p_sect_end", &format_args!("{:#02x}", self.p_sect_end))
            .field("p_lba", &format_args!("{:#08x}", self.p_lba))
            .field("p_size", &format_args!("{:#08x}", self.p_size))
            .finish()
    }
}
pub struct ByteReader {
    position: u64,
}
impl ByteReader {
    fn new(position: u64) -> Self {
        Self { position }
    }
    fn read1(&mut self) -> u8 {
        let value = unsafe { STORAGE[self.position as usize] };
        self.position += 1;
        value
    }
    fn read4(&mut self) -> u32 {
        let value = unsafe {
            ((STORAGE[(self.position + 3) as usize] as u32) << 24)
                + ((STORAGE[(self.position + 2) as usize] as u32) << 16)
                + ((STORAGE[(self.position + 1) as usize] as u32) << 8)
                + ((STORAGE[(self.position + 0) as usize] as u32) << 0)
        };
        self.position += 4;
        value
    }
}
fn read_partition(index: u8) -> Partition {
    defmt::assert!(index < 4);

    let position: u64 = 446 + (16 * (index as u64));

    let mut byte_reader = ByteReader::new(position);

    Partition {
        p_status: byte_reader.read1(),
        p_head_begin: byte_reader.read1(),
        p_sect_begin: byte_reader.read1(),
        p_cyl_begin: byte_reader.read1(),
        p_type: byte_reader.read1(),
        p_head_end: byte_reader.read1(),
        p_sect_end: byte_reader.read1(),
        p_cyl_end: byte_reader.read1(),
        p_lba: byte_reader.read4(),
        p_size: byte_reader.read4(),
    }
}
