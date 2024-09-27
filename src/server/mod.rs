use embassy_net::tcp::TcpSocket;

//pub mod echo;
//pub mod okay;
pub mod webdav;

pub trait SocketServer {
    async fn run(&mut self, socket: TcpSocket)
    where
        Self: Sized;
}
