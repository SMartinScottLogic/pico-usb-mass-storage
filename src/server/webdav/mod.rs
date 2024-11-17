mod error;
mod process;

use defmt::warn;
use embassy_net::tcp::TcpSocket;

use super::SocketServer;

pub struct Server {}

impl Server {
    pub fn new() -> Self {
        Self {}
    }
}

impl SocketServer for Server {
    async fn run(&mut self, mut socket: TcpSocket<'_>) {
        loop {
            if let Err(e) = process::process::<4096, 16>(&mut socket).await {
                warn!("Read error: {:?}", e);
                socket.close();
                break;
            }
        }
    }
}
