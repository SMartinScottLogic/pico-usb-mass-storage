mod setup;

use defmt::{error, info, Format};
use embassy_rp::rom_data::memcpy;
pub use setup::init;

use crate::{DisplayState, SIGNAL};

#[derive(Clone, Format)]
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
impl core::fmt::Debug for Partition {
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
pub struct ByteReader<'a> {
    data: &'a [u8],
    position: u64,
}
impl<'a> ByteReader<'a> {
    fn new(data: &'a [u8], position: u64) -> Self {
        Self { data, position }
    }
    fn read1(&mut self) -> u8 {
        let value = self.data[self.position as usize];
        self.position += 1;
        value
    }
    fn read4(&mut self) -> u32 {
        let position = self.position as usize;
        let slice = &self.data[position..position + 4];
        let value = u32::from_le_bytes(slice.try_into().unwrap());
        self.position += 4;
        value
    }
}
pub fn read_partition(data: &[u8], index: u8) -> Partition {
    defmt::assert!(index < 4);

    let position: u64 = 446 + (16 * (index as u64));

    let mut byte_reader = ByteReader::new(data, position);

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

pub fn log_fs(data: &mut [u8], blocks: u64, block_size: u64) {
    let partition = read_partition(data, 0);
    info!("partition {}: {}", 0, partition);

    let options = fatfs::FsOptions::new().update_accessed_date(false);

    let disk = MemFS::new(
        &mut data[(block_size * partition.p_lba as u64) as usize..],
        blocks,
        block_size,
    );
    let fs = match fatfs::FileSystem::new(disk, options) {
        Ok(fatfs) => fatfs,
        Err(e) => {
            let s = match e {
                fatfs::Error::Io(_) => "unknown: MemFSError",
                fatfs::Error::UnexpectedEof => "UnexpectedEof",
                fatfs::Error::WriteZero => "WriteZero",
                fatfs::Error::InvalidInput => "InvalidInput",
                fatfs::Error::NotFound => "NotFound",
                fatfs::Error::AlreadyExists => "AlreadyExists",
                fatfs::Error::DirectoryIsNotEmpty => "DirectoryIsNotEmpty",
                fatfs::Error::CorruptedFileSystem => "CorruptedFileSystem",
                fatfs::Error::NotEnoughSpace => "NotEnoughSpace",
                fatfs::Error::InvalidFileNameLength => "InvalidFileNameLength",
                fatfs::Error::UnsupportedFileNameCharacter => "UnsupportedFileNameCharacter",
                _ => "Unknown",
            };
            error!("Error: {}", s);
            SIGNAL.signal(DisplayState::FileSystem([0; 11], 0));
            return;
        }
    };
    let fat_type = match fs.fat_type() {
        fatfs::FatType::Fat12 => "Fat12",
        fatfs::FatType::Fat16 => "Fat16",
        fatfs::FatType::Fat32 => "Fat32",
    };
    let free_space = match fs.stats() {
        Ok(stats) => stats.free_clusters() * stats.cluster_size(),
        Err(_) => 0,
    };
    let volume_id = fs.volume_id();
    let volume_label: &str = core::str::from_utf8(fs.volume_label_as_bytes()).unwrap();

    let mut volume_lbl: [u8; 11] = [0; 11];
    unsafe {
        memcpy(
            volume_lbl.as_mut_ptr(),
            fs.volume_label_as_bytes().as_ptr(),
            11,
        );
    }
    info!(
        "type = {}, id = {}, label = {}",
        fat_type, volume_id, volume_label
    );
    log_dir("/", &fs.root_dir(), 0);
    SIGNAL.signal(DisplayState::FileSystem(volume_lbl, free_space));
}

fn log_dir<IO, TP, OCC>(parent: &str, dir: &fatfs::Dir<IO, TP, OCC>, depth: usize)
where
    IO: fatfs::ReadWriteSeek,
    TP: fatfs::TimeProvider,
    OCC: fatfs::OemCpConverter,
{
    for d in dir.iter().flatten() {
        let filename = core::str::from_utf8(d.short_file_name_as_bytes()).unwrap();
        // Temporary ugly hackery for UCS2 to UTF8 (probably nonsense for non-Latin1)
        let mut buf = [0_u8; 255];
        let lfn = match d.long_file_name_as_ucs2_units() {
            None => None,
            Some(lfn) => {
                for (i, c) in lfn.iter().enumerate() {
                    buf[i] = (c & 0xff) as u8;
                }
                Some(unsafe { core::str::from_utf8_unchecked(&buf) })
            }
        };
        let size = d.len();

        info!(
            "parent_name = \"{}\", file name = \"{}\", size = {}, depth = {}, is_file: {}, is_dir: {}, attributes: {:#02x}, lfn: {}",
            parent,
            filename,
            size,
            depth,
            d.is_file(),
            d.is_dir(),
            d.attributes().bits(),
            lfn
        );
        if d.is_dir() && filename != "." && filename != ".." {
            log_dir(filename, &d.to_dir(), depth + 1);
        }
    }
}

struct MemFS<'d> {
    blocks: u64,
    block_size: u64,
    data: &'d mut [u8],
    offset: u64,
}
impl<'d> MemFS<'d> {
    fn new(data: &'d mut [u8], blocks: u64, block_size: u64) -> Self {
        Self {
            data,
            blocks,
            block_size,
            offset: 0,
        }
    }
}
impl fatfs::IoBase for MemFS<'_> {
    type Error = MemFSError;
}
impl fatfs::Read for MemFS<'_> {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let limit = usize::min(
            buf.len(),
            ((self.blocks * self.block_size) - self.offset)
                .try_into()
                .unwrap(),
        );

        buf[..limit]
            .copy_from_slice(&self.data[self.offset as usize..self.offset as usize + limit]);
        self.offset += limit as u64;
        Ok(limit)
    }
}
impl fatfs::Write for MemFS<'_> {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let limit = usize::min(
            buf.len(),
            ((self.blocks * self.block_size) - self.offset)
                .try_into()
                .unwrap(),
        );

        self.data[self.offset as usize..self.offset as usize + limit].copy_from_slice(buf);
        self.offset += limit as u64;
        Ok(limit)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        error!("flush unimplemented");
        Ok(())
    }
}
impl fatfs::Seek for MemFS<'_> {
    fn seek(&mut self, pos: fatfs::SeekFrom) -> Result<u64, Self::Error> {
        match pos {
            fatfs::SeekFrom::Start(p) => self.offset = p,
            fatfs::SeekFrom::End(p) => {
                self.offset = ((self.blocks * self.block_size) as i64 - p) as u64
            }
            fatfs::SeekFrom::Current(p) => self.offset = (self.offset as i64 + p) as u64,
        }
        Ok(self.offset)
    }
}

#[derive(Debug)]
struct MemFSError {}
impl fatfs::IoError for MemFSError {
    fn is_interrupted(&self) -> bool {
        false
    }

    fn new_unexpected_eof_error() -> Self {
        unimplemented!()
    }

    fn new_write_zero_error() -> Self {
        unimplemented!()
    }
}
