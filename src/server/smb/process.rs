use binrw::io::{Cursor, Read, SeekFrom};
use binrw::{BinRead, BinWrite};
use defmt::{error, info, warn};
use embassy_net::tcp::TcpSocket;

use super::error::Error;

const HEX_DIGIT: &str = "0123456789ABCDEF";

#[derive(defmt::Format)]
struct MyBinrwError {
    #[defmt(Display2Format)]
    inner: binrw::Error,
}
impl From<binrw::Error> for MyBinrwError {
    fn from(value: binrw::Error) -> Self {
        Self { inner: value }
    }
}

#[derive(BinRead)]
#[br(big, assert(zero == 0, "invalid header: {} != 0", zero))]
struct DirectTCPTransportPacketHeader {
    zero: u8,
    stream_protocol_length: [u8; 3],
}
#[derive(BinRead)]
#[br(big, assert(protocol[0]==b'S' && protocol[1]==b'M' && protocol[2]==b'B'))]
struct SMBHeader {
    protocol0: u8,
    protocol: [u8; 3],
}

#[derive(defmt::Format, BinRead)]
#[br(big)]
struct SMBCommandHeader {
    Command: u8,
    Status: u32,
    Flags: u8,
    Flags2: u16,
    PIDHigh: u16,
    SecurityFeatures: [u8; 8],
    Reserved: u16,
    TID: u16,
    PIDLow: u16,
    UID: u16,
    MID: u16,
}
fn smb_dump(buf: &[u8], count: usize) -> Result<(), MyBinrwError> {
    let mut cursor = Cursor::new(&buf[..count]);
    let header = DirectTCPTransportPacketHeader::read(&mut cursor)?;
    info!("length: {}", header.stream_protocol_length);
    let header = SMBHeader::read(&mut cursor)?;
    info!("protocol: {}", header.protocol);
    match header.protocol0 {
        0xff => {
            let header = SMBCommandHeader::read(&mut cursor)?;
            info!("command header: {}", header);
        }
        o => error!("Invalid protocol {}", o),
    }
    let mut t = [0; 10];
    let mut i = 0;
    loop {
        match cursor.read(&mut t) {
            Ok(n) if n > 0 => {
                info!("residue {}: {}", i, &t[..n])
            }
            Ok(_) => break,
            Err(_) => break,
        }
        i += 1;
    }
    Ok(())
}

pub async fn process<const BUFFER_SIZE: usize, const NUM_OF_HEADERS: usize>(
    socket: &mut TcpSocket<'_>,
) -> Result<(), Error> {
    //let processor = Processor::<BUFFER_SIZE, NUM_OF_HEADERS>;
    // 1. Read
    let mut buf = [0; BUFFER_SIZE];
    let n = match socket.read(&mut buf).await {
        Ok(0) => {
            warn!("read EOF");
            return Err(Error::ReadEof);
        }
        Ok(n) => {
            info!("read {}", n);
            n
        }
        Err(embassy_net::tcp::Error::ConnectionReset) => {
            warn!("read error: ConnectionReset");
            return Err(Error::ConnectionReset);
        }
    };
    match smb_dump(&buf, n) {
        Ok(_) => info!("success"),
        Err(e) => error!("failure: {}", e),
    }
    Ok(())
}
